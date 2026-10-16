#!/usr/bin/env python3
"""Write wheel PID gains to the controller and confirm them by read-back.

usage: test_pid_set WHEEL kp ki kd   (WHEEL = FL, FR, RL, RR or ALL)
The controller stores the gains, so they stay after a restart; the startup
values in ired_bringup/param/ired.yaml are written again on the next bringup."""
import sys
import time

import numpy as np
import rclpy

from ired_msgs.srv import PIDTuning
from ired_web.tests.common import emit, run

FIELD = {'FL': 'pid_motor_front_left', 'FR': 'pid_motor_front_right',
         'RL': 'pid_motor_rear_left', 'RR': 'pid_motor_rear_right'}


def main(make_node):
    wheel = sys.argv[1].upper()
    gains = [float(v) for v in sys.argv[2:5]]
    wheels = list(FIELD) if wheel == 'ALL' else [wheel]
    n = make_node('test_pid_set')
    cli = n.create_client(PIDTuning, '/ired/pid/tuning')
    if not cli.wait_for_service(timeout_sec=5.0):
        emit(error='PID service not available. Is Robot mode running?')
        return
    for w in wheels:
        emit(progress=f'Setting {w}')
        req = PIDTuning.Request()
        req.motor, req.kp, req.ki, req.kd = w, *gains
        fut = cli.call_async(req)
        while not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        end = time.monotonic() + 4.0
        got = None
        while time.monotonic() < end:     # one wheel at a time: the service keeps only the last request
            rclpy.spin_once(n, timeout_sec=0.05)
            got = list(getattr(n.motor, FIELD[w])) if n.motor else None
            if got and np.allclose(got, gains, atol=1e-3):
                break
        else:
            emit(error=f'{w}: controller did not confirm the new gains (reads {got})')
            return
    emit(result={'wheels': wheels, 'kp': gains[0], 'ki': gains[1], 'kd': gains[2]})


def entry():
    run(main)


if __name__ == '__main__':
    entry()
