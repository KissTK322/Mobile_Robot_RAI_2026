#!/usr/bin/env python3
"""Wheel speed step test. WHEELS MUST BE LIFTED OFF THE FLOOR.

usage: test_wheel
Sends forward, backward and turning steps and reports, per wheel, how well the
speed feedback follows the setpoint."""
import time

import numpy as np
import rclpy

from ired_web.tests.common import emit, run

STEPS = [('fwd 0.10', 0.10, 0.0), ('fwd 0.20', 0.20, 0.0), ('fwd 0.30', 0.30, 0.0),
         ('back 0.20', -0.20, 0.0), ('turn +0.8', 0.0, 0.8), ('turn -0.8', 0.0, -0.8)]
STEP_SEC, GAP = 4.0, 1.5
WHEELS = ['FL', 'FR', 'RL', 'RR']


def main(make_node):
    n = make_node('test_wheel')
    if not n.wait_ready(need_scan=False):
        return
    rows = []
    for name, vx, wz in STEPS:
        emit(progress=f'Step {name}')
        log = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < STEP_SEC:
            n.send(vx=vx, wz=wz)
            last = n.motor
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.motor is not last:
                log.append((time.monotonic() - t0, *n.motor.speed_sp, *n.motor.speed_fb))
        n.spin_for(GAP)
        a = np.array(log)
        t = a[:, 0]
        for w in range(4):
            sp = float(np.median(a[t > 1.0, 1 + w]))
            fb = a[:, 5 + w]
            pct = fb / sp * 100 if abs(sp) > 0.5 else np.zeros_like(fb)
            end = pct[t > STEP_SEC - 1.0]
            reach = np.where(pct >= 90)[0]
            rows.append({'step': name, 'wheel': WHEELS[w], 'set_rpm': round(sp, 1),
                         'got_rpm': round(float(fb[t > STEP_SEC - 1.0].mean()), 1),
                         'track_pct': round(float(end.mean()), 1),
                         'rise90_s': round(float(t[reach[0]]), 2) if len(reach) else None,
                         'overshoot_pct': round(max(0.0, float(pct.max()) - 100), 1),
                         'ripple_rpm': round(float(fb[t > STEP_SEC - 1.0].std()), 2)})
    track = [r['track_pct'] for r in rows]
    by_step = {}
    for r in rows:
        by_step.setdefault(r['step'], []).append(r['track_pct'])
    m = n.motor
    emit(result={
        'gains': {'FL': list(m.pid_motor_front_left), 'FR': list(m.pid_motor_front_right),
                  'RL': list(m.pid_motor_rear_left), 'RR': list(m.pid_motor_rear_right)},
        'track_min_pct': round(min(track), 1), 'track_max_pct': round(max(track), 1),
        'wheel_spread_pct': round(max(max(v) - min(v) for v in by_step.values()), 1),
        'overshoot_max_pct': round(max(r['overshoot_pct'] for r in rows), 1),
        'rows': rows,
    })


def entry():
    run(main)


if __name__ == '__main__':
    entry()
