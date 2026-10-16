"""Shared pieces for the calibration tests started from the web app.

Each test prints JSON lines on stdout: {"progress": "..."} while running and
one {"result": {...}} at the end (or {"error": "..."}). The manager reads them."""
import json
import math
import sys
import time
import warnings

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ired_msgs.msg import IMUData, MotorData


def emit(**kw):
    print(json.dumps(kw), flush=True)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class TestNode(Node):
    """Collects IMU yaw, EKF pose, wheel-integrated motion, wheel data and scans."""

    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.imu = None
        self.ekf = None
        self.ekf_prev, self.ekf_yaw_total = None, 0.0
        self.wheel_x = self.wheel_y = self.wheel_yaw = 0.0
        self.last_speed_t = None
        self.motor = None
        self.scan = None
        self.want_scan = False
        self.create_subscription(IMUData, '/ired/imu/rollpitchyaw', lambda m: setattr(self, 'imu', m.yaw), 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, 10)
        self.create_subscription(TwistStamped, '/ired/speed', self.on_speed, 10)
        self.create_subscription(MotorData, '/ired/motor/data', lambda m: setattr(self, 'motor', m), 10)
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan, qos_profile_sensor_data)

    def on_ekf(self, m):
        self.ekf = m.pose.pose.position
        y = yaw_of(m.pose.pose.orientation)
        if self.ekf_prev is not None:
            self.ekf_yaw_total += wrap(y - self.ekf_prev)
        self.ekf_prev = y

    def on_speed(self, m):
        t = time.monotonic()
        if self.last_speed_t is not None:
            dt = t - self.last_speed_t
            self.wheel_x += m.twist.linear.x * dt
            self.wheel_y += m.twist.linear.y * dt
            self.wheel_yaw += m.twist.angular.z * dt
        self.last_speed_t = t

    def on_scan(self, m):
        if self.want_scan:          # only keep scans while grabbing, saves work
            self.scan = m

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.pub.publish(msg)

    def spin_for(self, sec, vx=0.0, vy=0.0, wz=0.0):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            self.send(vx, vy, wz)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self, need_scan=True, timeout=10.0):
        self.want_scan = need_scan
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.imu is not None and self.ekf is not None and self.last_speed_t is not None \
                    and (self.scan is not None or not need_scan):
                self.want_scan = False
                return True
        self.want_scan = False
        missing = [n for n, v in (('IMU', self.imu), ('EKF', self.ekf), ('wheels', self.last_speed_t),
                                  ('LiDAR', self.scan if need_scan else 1)) if v is None]
        emit(error=f"No data from {', '.join(missing)}. Is Robot mode running?")
        return False

    def grab_scan(self, n=8):
        """Median of n scans, binned to 1 deg over the full circle (NaN where no return)."""
        rows = []
        self.want_scan = True
        while len(rows) < n:
            self.scan = None
            while self.scan is None:
                self.send()
                rclpy.spin_once(self, timeout_sec=0.1)
            s = self.scan
            r = np.array(s.ranges, dtype=float)
            a = s.angle_min + np.arange(len(r)) * s.angle_increment
            ok = np.isfinite(r) & (r > 0.15) & (r < 6.0)
            grid = np.full(360, np.nan)
            grid[(np.round(np.degrees(a[ok])).astype(int)) % 360] = r[ok]
            rows.append(grid)
        self.want_scan = False
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)   # all-NaN bins are expected
            return np.nanmedian(np.array(rows), axis=0)

    def front_clear(self, backwards, stop_dist=0.35):
        s = self.scan
        if s is None:
            return True
        r = np.array(s.ranges, dtype=float)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        if backwards:
            a = a + math.pi
        a = (a + math.pi) % (2 * math.pi) - math.pi
        m = np.isfinite(r) & (np.abs(a) < math.radians(20)) & (r > 0.15)
        return not np.any(r[m] < stop_dist)


def scan_points(grid):
    ang = np.radians(np.arange(360))
    m = np.isfinite(grid)
    return np.c_[grid[m] * np.cos(ang[m]), grid[m] * np.sin(ang[m])]


def scan_rotation(before, after):
    """Rotation (deg, CCW +) between two binned scans, plus the match residual (m)."""
    scores = []
    for k in range(-180, 180):
        b = np.roll(before, -k)         # turned CCW by d: after(a) = before(a + d)
        m = np.isfinite(b) & np.isfinite(after)
        scores.append(float(np.median(np.abs(b[m] - after[m]))) if m.sum() > 100 else np.inf)
    scores = np.array(scores)
    i = int(np.argmin(scores))
    off = 0.0
    if 0 < i < len(scores) - 1 and np.all(np.isfinite(scores[i - 1:i + 2])):
        y0, y1, y2 = scores[i - 1:i + 2]
        den = y0 - 2 * y1 + y2
        off = 0.5 * (y0 - y2) / den if den > 0 else 0.0
    return (i - 180) + off, scores[i]


def icp(P, Q, guess, iters=30):
    """Robot motion (dx, dy, dyaw_deg, residual) in the start frame from before/after points."""
    th, t = 0.0, np.array(guess, dtype=float)
    dist = np.zeros(1)
    keep = np.ones(1, bool)
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        X = (P - t) @ np.array([[c, -s], [s, c]])
        d = ((X[:, None, :] - Q[None, :, :]) ** 2).sum(-1)
        j = d.argmin(1)
        dist = np.sqrt(d[np.arange(len(X)), j])
        keep = dist < max(0.05, np.percentile(dist, 70))
        A, B = P[keep], Q[j[keep]]
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((B - cb).T @ (A - ca))
        R = (U @ Vt).T
        if np.linalg.det(R) < 0:
            Vt[1] *= -1
            R = (U @ Vt).T
        th = math.atan2(R[1, 0], R[0, 0])
        t = ca - R @ cb
    return float(t[0]), float(t[1]), math.degrees(th), float(np.median(dist[keep]))


def run(main):
    """Start rclpy, run main(node_factory), always leave the robot stopped."""
    rclpy.init()
    nodes = []

    def factory(name):
        n = TestNode(name)
        nodes.append(n)
        return n
    try:
        main(factory)
    except KeyboardInterrupt:
        emit(error='Aborted')
    finally:
        for n in nodes:
            for _ in range(5):
                n.send()
                time.sleep(0.02)
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)
