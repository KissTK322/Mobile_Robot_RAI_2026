"""Offline test of the exploration sweep: fake robot + fake camera, two machines."""
import math
import random

import pytest

from ired_challenge.exploration import MachineEstimator
from ired_challenge.exploration_task import DONE, ExplorationTask, crossings, sweep, border_route
from ired_challenge.field import Field

F = Field()
IDS = {'BS': (121, 122), 'CS1': (101, 102)}          # (output, input)


class FakeRobot:
    def __init__(self, xy, yaw, speed=0.22, turn=1.0):
        self.x, self.y, self.yaw = xy[0], xy[1], yaw
        self.speed, self.turn = speed, turn
        self.via, self.goal_yaw, self._state = [], None, 'succeeded'
        self.trace = []

    def go_via(self, points, yaw):
        self.via, self.goal_yaw, self._state = list(points[1:]), None, 'active'

    def turn_to(self, yaw):
        self.via, self.goal_yaw, self._state = [], yaw, 'active'

    def state(self):
        return self._state

    def cancel(self):
        self.via, self.goal_yaw = [], None
        if self._state == 'active':
            self._state = 'failed'

    def step(self, dt):
        if self._state != 'active':
            return
        if self.goal_yaw is not None:
            e = math.atan2(math.sin(self.goal_yaw - self.yaw), math.cos(self.goal_yaw - self.yaw))
            if abs(e) < 0.03:
                self._state = 'succeeded'
            else:
                self.yaw += max(-self.turn * dt, min(self.turn * dt, e))
            return
        if not self.via:
            self._state = 'succeeded'
            return
        gx, gy = self.via[0]
        d = math.hypot(gx - self.x, gy - self.y)
        if d <= self.speed * dt:
            self.x, self.y = gx, gy
            self.via.pop(0)
        else:
            self.x += (gx - self.x) / d * self.speed * dt
            self.y += (gy - self.y) / d * self.speed * dt
        self.trace.append((self.x, self.y))


def camera(robot, boxes, rng, cam_x=0.23, hfov=math.radians(45.6)):
    """Detections like /aruco/detections: (id, pos_base, marker_yaw_deg)."""
    cx, cy = robot.x + cam_x * math.cos(robot.yaw), robot.y + cam_x * math.sin(robot.yaw)
    out = []
    for kind, zone, rot in boxes:
        c = F.center(zone)
        for side, add in (('input', 0), ('output', math.pi)):
            n = math.radians(rot) + add
            m = (c[0] + 0.1 * math.cos(n), c[1] + 0.1 * math.sin(n))
            d = math.dist(m, (cx, cy))
            b = math.atan2(m[1] - cy, m[0] - cx) - robot.yaw
            v = math.atan2(cy - m[1], cx - m[0]) - n
            if not (0.25 <= d <= 2.0) or abs(math.atan2(math.sin(b), math.cos(b))) > hfov / 2 \
                    or abs(math.atan2(math.sin(v), math.cos(v))) > math.radians(60):
                continue
            sd = 0.01 + 0.02 * d
            mx, my = m[0] + rng.gauss(0, sd), m[1] + rng.gauss(0, sd)
            cs, sn = math.cos(-robot.yaw), math.sin(-robot.yaw)
            pos = (cs * (mx - robot.x) - sn * (my - robot.y), sn * (mx - robot.x) + cs * (my - robot.y))
            face = math.degrees(n - robot.yaw + math.radians(rng.gauss(0, 3 + 2 * d)))
            out.append((IDS[kind][1] if side == 'input' else IDS[kind][0], pos, face))
    return out


def run(boxes, seed=1, max_time=600, robot_cls=None):
    rng = random.Random(seed)
    robot = (robot_cls or FakeRobot)((4.5, 0.5), math.pi)                       # C_Z51 facing -x
    est = MachineEstimator(F, refbox_rules=False)
    reports = []
    task = ExplorationTask(robot, F, est, lambda m, x, y, r: reports.append((m, F.zone_at(x, y), r)) or True,
                           log=lambda s: None, free=lambda p: p[1] > 1.0 or p[0] < 2.0)
    t = 0.0
    while t < max_time:
        if int(t * 10) % 1 == 0:
            for mid, pos, face in camera(robot, boxes, rng):
                est.add((robot.x, robot.y, robot.yaw), mid, pos, face)
        info = task.update(t, True, (robot.x, robot.y, robot.yaw))
        if info['state'] == DONE:
            break
        robot.step(0.1)
        t += 0.1
    return task, reports, robot, t


@pytest.mark.parametrize('boxes', [
    [('BS', 'C_Z33', 90), ('CS1', 'C_Z24', 0)],
    [('BS', 'C_Z45', 180), ('CS1', 'C_Z22', 135)],
    [('BS', 'C_Z14', 270), ('CS1', 'C_Z43', 45)],
])
def test_finds_and_reports_both_machines_correctly(boxes):
    task, reports, robot, t = run(boxes)
    assert task.state == DONE
    truth = {f'C-{k}': (z, r % 360) for k, z, r in boxes}
    assert {m: (z, r) for m, z, r in reports} == truth, reports
    assert len(reports) == 2                                      # one report per machine
    assert t < 600, f'{t:.0f} s'


def test_never_drives_close_to_a_zone_centre_that_may_hold_a_box():
    boxes = [('BS', 'C_Z33', 90), ('CS1', 'C_Z24', 0)]
    task, reports, robot, t = run(boxes)
    for x, y in robot.trace:
        if y < 1.0 or F.zone_at(x, y) == 'C_Z32':
            continue                                              # START and the red zone: no machines there
        for a in range(1, 6):
            for b in range(2, 6):
                assert math.hypot(x - (a - 0.5), y - (b - 0.5)) >= 0.45, (x, y)


def test_sweep_and_border_route():
    pts = crossings(F, lambda p: True)
    assert len(pts) == 16
    order = sweep(pts, (2.5, 1.5))
    assert order[0][1] == 1.0 and len(order) == 16
    for a, b in zip(order, order[1:]):
        route = border_route(a, b)
        for p, q in zip(route, route[1:]):
            assert p[0] == q[0] or p[1] == q[1]                   # along a border line


class AbortingRobot(FakeRobot):
    """Like DWB with a short look-ahead: every other drive is aborted 0.4 m into it."""

    def go_via(self, points, yaw):
        super().go_via(points, yaw)
        self.drives = getattr(self, 'drives', 0) + 1
        self.start, self.done_once = (self.x, self.y), self.drives % 2 == 0

    def step(self, dt):
        super().step(dt)
        if self._state == 'active' and self.goal_yaw is None and not self.done_once \
                and math.hypot(self.x - self.start[0], self.y - self.start[1]) >= 0.4:
            self.done_once = True
            self.via, self._state = [], 'failed'


def test_drives_aborted_on_the_way_are_resent_not_skipped():
    boxes = [('BS', 'C_Z33', 90), ('CS1', 'C_Z24', 0)]
    task, reports, robot, t = run(boxes, robot_cls=AbortingRobot)
    assert task.state == DONE and len(reports) == 2, reports
    for x, y in robot.trace:
        if y < 1.0 or F.zone_at(x, y) == 'C_Z32':
            continue
        for a in range(1, 6):
            for b in range(2, 6):
                assert math.hypot(x - (a - 0.5), y - (b - 0.5)) >= 0.45, (x, y)
