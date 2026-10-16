#!/usr/bin/env python3
"""Straight-line test with LiDAR ground truth.

usage: test_straight [dist=1.0] [speed=0.15]   (negative dist = backwards)
Drives until the wheel odometry says dist metres, stops early if something is
closer than 35 cm ahead, then measures the true motion with 2D ICP."""
import math
import sys
import time

import rclpy

from ired_web.tests.common import emit, icp, run, scan_points

SLOW = 0.06


def main(make_node):
    dist = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
    direction = 1.0 if dist >= 0 else -1.0
    dist = abs(dist)
    timeout = 3.0 * dist / speed + 5.0
    n = make_node('test_straight')
    if not n.wait_ready():
        return
    n.spin_for(1.0)
    emit(progress='Reading LiDAR at the start')
    P = scan_points(n.grab_scan())
    wx0, wy0 = n.wheel_x, n.wheel_y
    e0 = (n.ekf.x, n.ekf.y)
    emit(progress=f"Driving {dist:g} m {'forward' if direction > 0 else 'backward'}")
    n.want_scan = True          # watch for obstacles while driving
    stopped_early = False
    t0 = time.monotonic()
    while math.hypot(n.wheel_x - wx0, n.wheel_y - wy0) < dist and time.monotonic() - t0 < timeout:
        if not n.front_clear(backwards=direction < 0):
            stopped_early = True
            break
        left = dist - math.hypot(n.wheel_x - wx0, n.wheel_y - wy0)
        n.send(vx=direction * (speed if left > 0.10 else SLOW))
        rclpy.spin_once(n, timeout_sec=0.05)
    n.want_scan = False
    dur = time.monotonic() - t0
    n.spin_for(2.0)
    emit(progress='Reading LiDAR at the end')
    Q = scan_points(n.grab_scan())
    wheel = math.hypot(n.wheel_x - wx0, n.wheel_y - wy0)
    ekf = math.hypot(n.ekf.x - e0[0], n.ekf.y - e0[1])
    dx, dy, dyaw, res = icp(P, Q, (direction * wheel, 0.0))
    true = math.hypot(dx, dy)
    emit(result={
        'direction': 'forward' if direction > 0 else 'backward', 'target_m': dist, 'speed': speed,
        'time_s': round(dur, 1), 'stopped_early': stopped_early, 'timeout': dur >= timeout,
        'lidar_m': round(true, 3), 'wheel_m': round(wheel, 3), 'ekf_m': round(ekf, 3),
        'wheel_ratio': round(true / wheel, 4) if wheel else None,
        'ekf_error_m': round(ekf - true, 3),
        'side_drift_cm': round(dy * 100, 1), 'yaw_drift_deg': round(dyaw, 1),
        'scan_residual_cm': round(res * 100, 1),
    })


def entry():
    run(main)


if __name__ == '__main__':
    entry()
