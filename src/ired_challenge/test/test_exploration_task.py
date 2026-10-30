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


def camera(robot, boxes, rng, cam_x=0.23, hfov=math.radians(45.6), miss=0.0):
    """Detections like /aruco/detections: (id, pos_base, marker_yaw_deg). `miss`: share of
    frames lost per m of distance (at most 70 %), like the PC sim camera."""
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
            if miss and rng.random() > max(0.3, 1.0 - miss * d):
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


class ShortRobot(FakeRobot):
    """Like Nav2 on the robot and in the PC sim: stops up to 0.1 m short of the last waypoint."""

    def step(self, dt):
        if self._state == 'active' and self.goal_yaw is None and len(self.via) == 1 \
                and math.hypot(self.via[0][0] - self.x, self.via[0][1] - self.y) <= 0.09:
            self.via, self._state = [], 'succeeded'
            return
        super().step(dt)


@pytest.mark.parametrize('boxes', [
    [('BS', 'C_Z25', 180), ('CS1', 'C_Z22', 0)],                 # PC sim 2026-10-26 E1 seed 3
    [('BS', 'C_Z34', 135), ('CS1', 'C_Z53', 270)],               # seed 5
    [('BS', 'C_Z12', 90), ('CS1', 'C_Z33', 0)],                  # seed 7
])
def test_border_boxes_with_the_real_camera(boxes, monkeypatch):
    """Robot camera (640x480 calibration: HFOV 35.5 deg, lens 0.25 m forward) and a robot that
    stops short: a box on a border zone (conveyor along the wall) shows each marker only
    obliquely to one crossing, off the zone-centre heading."""
    real = camera
    monkeypatch.setitem(globals(), 'camera', lambda r, b, g: real(r, b, g, cam_x=0.25, hfov=math.radians(35.5), miss=0.38))
    task, reports, robot, t = run(boxes, robot_cls=ShortRobot)
    truth = {f'C-{k}': (z, r % 360) for k, z, r in boxes}
    assert {m: (z, r) for m, z, r in reports} == truth, reports
    assert t < 600, f'{t:.0f} s'


def test_three_headings_per_zone():
    task = ExplorationTask(FakeRobot((1.0, 4.0), 0.0), F, MachineEstimator(F), lambda *a: True, log=lambda s: None)
    heads = sorted(round(math.degrees(h)) for h in task._looks_at((1.0, 4.0)))
    assert heads == [-147, -135, -123, -57, -45, -33, 33, 45, 57, 123, 135, 147]
    task = ExplorationTask(FakeRobot((1.0, 4.0), 0.0), F, MachineEstimator(F), lambda *a: True, log=lambda s: None,
                           look_spread_deg=0.0)
    assert sorted(round(math.degrees(h)) for h in task._looks_at((1.0, 4.0))) == [-135, -45, 45, 135]


@pytest.mark.parametrize('final_single_view, n_sightings, expected', [
    (True, 12, [('C-BS', 'C_Z25', 180)]),       # one view, solid: reported when the sweep ends
    (True, 4, []),                                # too few sightings
    (False, 12, []),                              # fallback off
])
def test_one_view_machine_reported_when_the_sweep_ends(final_single_view, n_sightings, expected):
    """C_Z25 rotation 180 seen only from crossing (1, 4): never `ready` (min_views 2)."""
    est = MachineEstimator(F, refbox_rules=False)
    rng = random.Random(3)
    robot = FakeRobot((1.0, 3.92), math.radians(52))
    for _ in range(n_sightings):
        for mid, pos, face in camera(robot, [('BS', 'C_Z25', 180)], rng, cam_x=0.25, hfov=math.radians(35.5)):
            est.add((robot.x, robot.y, robot.yaw), mid, pos, face)
    assert est.estimate('C-BS')['views'] == 1 and not est.estimate('C-BS')['ready']
    reports = []
    task = ExplorationTask(robot, F, est, lambda m, x, y, r: reports.append((m, F.zone_at(x, y), r)) or True,
                           log=lambda s: None, final_single_view=final_single_view)
    task.state = 'LOOK'
    task._next((robot.x, robot.y, robot.yaw), 100.0)            # empty queue: sweep finished
    assert task.state == DONE and reports == expected


class StubNav:
    def __init__(self):
        self.calls, self.st = [], 'succeeded'

    def go_via(self, points, yaw):
        self.calls.append(('go', list(points)))

    def turn_to(self, yaw):
        self.calls.append(('turn', yaw))

    def state(self):
        return self.st

    def cancel(self):
        pass


def seen_task():
    """CS1 in C_Z54 rotation 270: marker 102 seen 20x from crossing (4, 3) only, like the field game."""
    est = MachineEstimator(F)
    for _ in range(20):
        est.add((4.0, 3.0, math.atan2(0.4, 0.5)), 102, (0.64, 0.0), -128.66)
    reports = []
    nav = StubNav()
    t = ExplorationTask(nav, F, est, lambda m, x, y, r: reports.append((m, x, y, r)) or True, log=lambda m: None,
                        expected=['C-CS1'])
    t.red_done, t.here, t.heading, t.queue = True, (4.0, 3.0), 0.0, [(1.0, 1.0)]
    return t, est, nav, reports


def test_marker_seen_from_one_position_gets_a_second_look_at_once():
    t, est, nav, reports = seen_task()
    e = est.estimate('C-CS1')
    assert e['zone'] == 'C_Z54' and e['agreeing'] == 20 and not e['ready']
    t._look(0.0)
    assert t.state == 'MOVE' and nav.calls[-1] == ('go', [(4.0, 3.0), (4.0, 2.68)])
    t.update(1.0, True, (4.0, 2.68, 0.0))                  # arrived: turns to the marker
    assert t.state == 'LOOK' and nav.calls[-1][0] == 'turn'
    est.add((4.0, 2.68, math.atan2(0.72, 0.5)), 102, (0.876, 0.0), -145.2)
    t.update(2.0, True, (4.0, 2.68, 0.0))
    assert [r[0] for r in reports] == ['C-CS1']


def test_only_one_second_look_and_the_route_returns_over_the_crossing():
    t, est, nav, reports = seen_task()
    t._look(0.0)
    t.update(1.0, True, (4.0, 2.68, 0.0))
    t.looks, t.t_still = [], None
    t._look(2.0)                                           # nothing new seen from there
    assert 'C-CS1' in t.relooked and t.state == 'MOVE'
    assert nav.calls[-1] == ('go', [(4.0, 2.68), (4.0, 3.0), (1.0, 3.0), (1.0, 1.0)])


def test_unreachable_second_look_point_moves_on():
    t, est, nav, reports = seen_task()
    t._look(0.0)
    nav.st = 'failed'
    t.resends = 3
    t.update(5.0, True, (4.0, 3.0, 0.0))
    assert t.state == 'MOVE' and t.back is None
