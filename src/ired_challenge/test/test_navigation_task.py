"""Offline tests: field math and a simulated navigation challenge (no ROS)."""
import math

import pytest

from ired_challenge.field import Field, mps_footprint, visit_order
from ired_challenge.navigation_task import DONE, NavigationTask

F = Field()


# ------------------------------------------------------------------ field math
def test_zone_center_matches_refbox_example():
    # refbox example drives to (zone[3] - 0.5, zone[4] - 0.5)
    assert F.center('C_Z32') == (2.5, 1.5)
    assert F.center('C_Z11') == (0.5, 0.5)
    assert F.center('C_Z55') == (4.5, 4.5)


def test_zone_at_and_inside():
    assert F.zone_at(2.5, 1.5) == 'C_Z32'
    assert F.zone_at(0.01, 4.99) == 'C_Z15'
    assert F.zone_at(-0.1, 1.0) is None
    assert F.inside('C_Z32', 2.9, 1.1)
    assert not F.inside('C_Z32', 2.9, 1.1, margin=0.2)


def test_magenta_zones_are_mirrored_to_negative_x():
    both = Field(magenta=True)
    assert both.center('M_Z32') == (-2.5, 1.5)
    assert both.zone_at(-2.5, 1.5) == 'M_Z32'
    with pytest.raises(ValueError):
        F.center('M_Z32')                                 # course field is cyan only


def test_bad_zone_names():
    for bad in ('C_Z60', 'C_Z06', 'Z32', 'C_Z3'):
        with pytest.raises(ValueError):
            F.center(bad)


def test_goal_beside_mps_stays_in_zone_and_faces_robot():
    gx, gy = F.goal_in_zone('C_Z33', from_xy=(0.5, 2.5), blocked_zones=['C_Z33'])
    assert F.inside('C_Z33', gx, gy, margin=0.05)
    assert math.hypot(gx - 2.5, gy - 2.5) >= 0.37      # robot radius + box clears
    assert gx < 2.5                                       # the side the robot comes from


def test_goal_beside_mps_is_deep_in_the_zone_and_off_the_walls():
    # PC sim: a goal 10 cm from the zone edge ended 2 cm inside it and the refbox never
    # counted the zone. Every zone the refbox can put a machine in, from 4 directions.
    from ired_challenge.exploration import allowed_rotations
    zones = [f'C_Z{x}{y}' for x in range(1, 6) for y in range(1, 6) if allowed_rotations(f'C_Z{x}{y}', 'C-BS')]
    for z in zones:
        cx, cy = F.center(z)
        for fx, fy in ((cx + 2, cy), (cx - 2, cy), (cx, cy + 2), (cx, cy - 2)):
            gx, gy = F.goal_in_zone(z, (fx, fy), blocked_zones=[z])
            assert F.inside(z, gx, gy, margin=0.15), (z, gx, gy)
            assert math.hypot(gx - cx, gy - cy) >= 0.39, (z, gx, gy)          # box in any rotation
            assert 0.30 <= gx <= 4.70 and 0.30 <= gy <= 4.70, (z, gx, gy)  # robot radius + 8 cm off walls


def test_mps_footprint_covers_the_box_only():
    pts = mps_footprint(F.center('C_Z23'), 45)
    cx, cy = F.center('C_Z23')
    far = max(math.hypot(x - cx, y - cy) for x, y in pts)
    assert 0.17 < far <= 0.13 * math.sqrt(2) + 1e-9          # corners of the 26 cm square
    assert all(F.zone_at(x, y) == 'C_Z23' for x, y in pts)
    assert len(pts) >= 64


def test_nearest_order_is_shorter_than_given():
    zones = ['C_Z55', 'C_Z11', 'C_Z54', 'C_Z12', 'C_Z53', 'C_Z13']
    def length(order, start=(4.5, 0.5)):
        pts = [start] + [F.center(z) for z in order]
        return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    near = visit_order(zones, (4.5, 0.5), F, 'nearest')
    assert sorted(near) == sorted(zones)
    assert length(near) < length(zones)


# ------------------------------------------------------------ simulated run
class FakeNav:
    """Drives in a straight line at `speed`; zones in `fail` fail that many times."""

    def __init__(self, robot, speed=0.25, fail=None):
        self.robot, self.speed = robot, speed
        self.fail = dict(fail or {})
        self.goal, self._state, self.cancels, self.clears = None, 'succeeded', 0, 0

    def go_to(self, x, y, yaw):
        self.calls = getattr(self, 'calls', 0) + 1
        self.goal, self._state = (x, y), 'active'
        zone = F.zone_at(x, y)
        if self.fail.get(zone, 0) > 0:
            self.fail[zone] -= 1
            self._state = 'failed'

    def state(self):
        return self._state

    def cancel(self):
        self.cancels += 1
        if self._state == 'active':
            self._state = 'failed'
        self.goal = None

    def clear_costmaps(self):
        self.clears += 1

    def step(self, dt):
        if self._state != 'active':
            return
        (x, y), (gx, gy) = self.robot['xy'], self.goal
        d = math.hypot(gx - x, gy - y)
        if d <= self.speed * dt:
            self.robot['xy'] = (gx, gy)
            self._state = 'succeeded'
        else:
            self.robot['xy'] = (x + (gx - x) / d * self.speed * dt, y + (gy - y) / d * self.speed * dt)


class FakeRefbox:
    """Counts a zone once the robot has been inside it for 5 s (only after the red zone)."""

    def __init__(self, zones, confirm=True):
        self.remaining, self.confirm = list(zones), confirm
        self.inside_since = {}
        self.red_seen = False

    def step(self, now, xy):
        if F.inside('C_Z32', *xy):
            self.red_seen = True
        zone = F.zone_at(*xy)
        for z in list(self.remaining):
            if z == zone and self.red_seen:
                self.inside_since.setdefault(z, now)
                if self.confirm and now - self.inside_since[z] >= 5.0:
                    self.remaining.remove(z)
            else:
                self.inside_since.pop(z, None)


def simulate(zones, fail=None, game=lambda t: True, max_time=600, blocked=(), confirm=True, order='given'):
    robot = {'xy': (4.5, 0.5)}                        # START zone corner
    nav = FakeNav(robot, fail=fail)
    ref = FakeRefbox(zones, confirm=confirm)
    logs = []
    task = NavigationTask(nav, F, log=logs.append, blocked_zones=blocked, order=order)
    t, dt = 0.0, 0.1
    while t < max_time:
        ref.step(t, robot['xy'])
        info = task.update(t, game(t), ref.remaining, (*robot['xy'], 0.0))
        if info['state'] == DONE:
            break
        nav.step(dt)
        t += dt
    return task, ref, nav, logs, t


ZONES = ['C_Z11', 'C_Z45', 'C_Z23', 'C_Z52', 'C_Z14', 'C_Z35',
         'C_Z21', 'C_Z44', 'C_Z12', 'C_Z53', 'C_Z25', 'C_Z41']


def test_full_challenge_visits_all_zones_in_time():
    task, ref, nav, logs, t = simulate(ZONES)
    assert task.state == DONE
    assert ref.remaining == []
    assert task.red_done and logs.index('Red zone passed') < next(i for i, l in enumerate(logs) if l.startswith('Visited'))
    assert task.visited == ZONES                       # refbox order kept
    assert t < 600, f'took {t:.0f} s, the limit is 10 min'


def test_nearest_order_is_faster():
    _, _, _, _, t_given = simulate(ZONES)
    task, ref, _, _, t_near = simulate(ZONES, order='nearest')
    assert ref.remaining == [] and t_near < t_given


def test_retry_then_success():
    task, ref, nav, logs, _ = simulate(['C_Z11', 'C_Z45'], fail={'C_Z45': 2})
    assert ref.remaining == []
    assert nav.clears == 2
    assert any('C_Z45: try 2' in l for l in logs)


def test_zone_that_keeps_failing_is_done_last():
    task, ref, nav, logs, _ = simulate(['C_Z11', 'C_Z45', 'C_Z23'], fail={'C_Z11': 3})
    assert ref.remaining == []
    assert task.visited[-1] == 'C_Z11'                 # deferred, then succeeded
    assert any('giving up for now' in l for l in logs)


def test_game_pause_stops_and_resumes():
    paused = lambda t: not (40 <= t < 60)
    task, ref, nav, logs, _ = simulate(ZONES[:4], game=paused)
    assert ref.remaining == []
    assert 'Game stopped, robot stopped' in logs


def test_does_nothing_before_the_game_starts():
    _, _, nav, _, _ = simulate(ZONES[:2], game=lambda t: False, max_time=30)
    assert getattr(nav, 'calls', 0) == 0                # never asked Nav2 to move
    task, ref, _, _, _ = simulate(ZONES[:2], game=lambda t: t >= 30)
    assert ref.remaining == [] and task.visited == ZONES[:2]


def test_mps_zone_is_visited_beside_the_box():
    task, ref, nav, logs, _ = simulate(['C_Z33', 'C_Z11'], blocked=['C_Z33'])
    assert ref.remaining == []


def test_unconfirmed_zone_is_retried_not_skipped_silently():
    task, ref, nav, logs, _ = simulate(['C_Z11'], confirm=False, max_time=60)
    assert task.state != DONE
    assert any('did not confirm' in l for l in logs)


def test_drive_finishing_between_two_state_reads_is_not_a_failure():
    # Seen in the PC simulation: Nav2's result arrived between two state() calls in one
    # update, so a reached zone was retried three times and given up.
    class FlakyReadNav(FakeNav):
        pending = False

        def state(self):
            s = self._state
            if self.pending:                  # result arrives right after this read
                self.pending, self._state = False, 'succeeded'
            return s

        def step(self, dt):
            super().step(dt)
            if self._state == 'succeeded' and self.goal is not None and not self.reported:
                self.reported, self.pending, self._state = True, True, 'active'

        def go_to(self, x, y, yaw):
            super().go_to(x, y, yaw)
            self.reported = False

    robot = {'xy': (2.5, 1.5)}
    nav = FlakyReadNav(robot)
    logs = []
    task = NavigationTask(nav, F, log=logs.append)
    task.red_done, task.state = True, 'PICK'
    ref = FakeRefbox(['C_Z22'])
    ref.red_seen = True
    t = 0.0
    while t < 60 and task.state != DONE:
        ref.step(t, robot['xy'])
        task.update(t, True, ref.remaining, (*robot['xy'], 0.0))
        nav.step(0.1)
        t += 0.1
    assert task.state == DONE
    assert not any('try 2' in l for l in logs), logs
