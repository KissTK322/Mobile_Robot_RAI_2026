#!/usr/bin/env python3
"""In-place rotation test with LiDAR ground truth.

usage: test_rotate [turns=1] [speed=0.5]   (negative speed = clockwise)
Spins until the IMU says turns x 360 deg, then compares laser scans before and
after to get the true rotation and reports IMU, wheel-only and EKF errors."""
import math
import sys
import time

import rclpy

from ired_web.tests.common import emit, run, scan_rotation

SLOW = 0.2


def main(make_node):
    turns = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    direction = 1.0 if speed >= 0 else -1.0
    target = turns * 2 * math.pi
    timeout = 45.0 * turns
    n = make_node('test_rotate')
    if not n.wait_ready():
        return
    n.spin_for(1.0)
    emit(progress='Reading LiDAR before turning')
    before = n.grab_scan()
    imu0, ekf0, wheel0 = n.imu, n.ekf_yaw_total, n.wheel_yaw
    emit(progress=f"Turning {turns:g} rev {'CCW' if direction > 0 else 'CW'}")
    t0 = time.monotonic()
    while abs(n.imu - imu0) < target and time.monotonic() - t0 < timeout:
        left = target - abs(n.imu - imu0)
        n.send(wz=direction * (abs(speed) if left > math.radians(25) else SLOW))
        rclpy.spin_once(n, timeout_sec=0.05)
    dur = time.monotonic() - t0
    n.spin_for(2.5)
    emit(progress='Reading LiDAR after turning')
    after = n.grab_scan()
    imu = math.degrees(n.imu - imu0)
    wheel = math.degrees(n.wheel_yaw - wheel0)
    ekf = math.degrees(n.ekf_yaw_total - ekf0)
    delta, res = scan_rotation(before, after)
    k = round((imu - delta) / 360.0)
    true = min((delta + 360 * j for j in (k - 1, k, k + 1)), key=lambda c: abs(c - imu))
    emit(result={
        'direction': 'CCW' if direction > 0 else 'CW', 'turns': turns, 'speed': abs(speed),
        'time_s': round(dur, 1), 'timeout': dur >= timeout,
        'lidar_deg': round(true, 1), 'imu_deg': round(imu, 1), 'wheel_deg': round(wheel, 1),
        'ekf_deg': round(ekf, 1), 'ekf_error_deg': round(ekf - true, 1),
        'imu_ratio': round(true / imu, 4) if imu else None,
        'wheel_ratio': round(true / wheel, 4) if wheel else None,
        'scan_residual_cm': round(res * 100, 1),
    })


def entry():
    run(main)


if __name__ == '__main__':
    entry()
