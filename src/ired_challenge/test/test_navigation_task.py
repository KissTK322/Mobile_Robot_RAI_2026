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


class FakeNavShortAbort(FakeNav):
    """Like DWB with a short look-ahead: the drive aborts `short` metres before the goal."""

    def __init__(self, robot, short):
        super().__init__(robot)
        self.short = short

    def step(self, dt):
        super().step(dt)
        if self._state == 'active' and math.dist(self.robot['xy'], self.goal) <= self.short:
            self._state = 'failed'


def run_short_abort(short, zones=('C_Z11', 'C_Z44')):
    robot = {'xy': (4.5, 0.5)}
    nav = FakeNavShortAbort(robot, short)
    ref, logs = FakeRefbox(list(zones)), []
    task = NavigationTask(nav, F, log=logs.append)
    t = 0.0
    while t < 300 and task.state != DONE:
        ref.step(t, robot['xy'])
        task.update(t, True, ref.remaining, (*robot['xy'], 0.0))
        nav.step(0.1)
        t += 0.1
    return task, ref, logs


def test_drive_aborted_just_short_of_the_stop_point_counts_as_arrived():
    task, ref, logs = run_short_abort(0.15)
    assert ref.remaining == [] and task.state == DONE
    assert not any('try 2' in l for l in logs)
    assert any('short of the stop point' in l for l in logs)


def test_drive_aborted_far_from_the_stop_point_is_still_retried():
    task, ref, logs = run_short_abort(0.6, zones=('C_Z11',))
    mine = [l for l in logs if l.startswith('C_Z11:')]
    assert 'short of the stop point' not in mine[0]                 # not taken for an arrival
    assert 'C_Z11: try 2 of 3' in mine                              # a drive that keeps failing is retried


class FakeNavMidAbort(FakeNav):
    """Aborts every drive once after `after` metres, like DWB's empty transformed plan."""

    def __init__(self, robot, after=0.8):
        super().__init__(robot)
        self.after, self.start, self.aborted = after, None, set()

    def go_to(self, x, y, yaw):
        super().go_to(x, y, yaw)
        self.start = self.robot['xy']

    def step(self, dt):
        super().step(dt)
        if self._state == 'active' and self.goal not in self.aborted \
                and math.dist(self.robot['xy'], self.start) >= self.after:
            self.aborted.add(self.goal)
            self._state = 'failed'


def test_abort_on_the_way_resends_the_route_without_a_retry():
    robot = {'xy': (4.5, 0.5)}
    nav = FakeNavMidAbort(robot)
    ref, logs = FakeRefbox(['C_Z11', 'C_Z14']), []
    task = NavigationTask(nav, F, log=logs.append)
    t = 0.0
    while t < 300 and task.state != DONE:
        ref.step(t, robot['xy'])
        task.update(t, True, ref.remaining, (*robot['xy'], 0.0))
        nav.step(0.1)
        t += 0.1
    assert ref.remaining == [] and task.state == DONE
    assert any('aborted on the way' in l for l in logs)
    assert not any(l.startswith(('C_Z11: try', 'C_Z14: try')) for l in logs)


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


def test_escape_direction_points_away_from_close_obstacles():
    from ired_challenge.escape import escape_direction
    assert escape_direction([(2.0, 0.0)]) is None                      # nothing close
    dx, dy = escape_direction([(0.2, 0.0)])
    assert dx < -0.99 and abs(dy) < 1e-6                               # obstacle ahead -> back
    # fence end ahead-left and box ahead-right: back off, mostly straight
    dx, dy = escape_direction([(0.15, 0.15), (0.15, -0.30)])
    assert dx < 0 and dy < 0                                           # the nearer one (left) pushes harder


def test_robot_stuck_in_inflation_escapes_instead_of_retrying_forever():
    # PC sim: every plan failed at once from inside the fence corner's inflation; the
    # robot retried for 10 minutes. Two instant failures in a row must trigger escape().
    class StuckNav(FakeNav):
        stuck, escapes = True, 0

        def go_to(self, x, y, yaw):
            super().go_to(x, y, yaw)
            if self.stuck:
                self._state = 'failed'

        def escape(self):
            self.escapes += 1
            self.stuck = False

    robot = {'xy': (2.5, 1.5)}
    nav = StuckNav(robot)
    logs = []
    task = NavigationTask(nav, F, log=logs.append)
    task.red_done, task.state = True, 'PICK'
    ref = FakeRefbox(['C_Z22', 'C_Z21'])
    ref.red_seen = True
    t = 0.0
    while t < 120 and task.state != DONE:
        ref.step(t, robot['xy'])
        task.update(t, True, ref.remaining, (*robot['xy'], 0.0))
        nav.step(0.1)
        t += 0.1
    assert nav.escapes == 1, logs
    assert task.state == DONE and ref.remaining == []


# ------------------------------------------------------------ grid route mode
class FakeNavVia(FakeNav):
    """FakeNav that can also follow waypoints (the grid route)."""

    def go_via(self, points, yaw):
        self.calls = getattr(self, 'calls', 0) + 1
        self.via = list(points[1:])
        self.goal, self._state = self.via.pop(0), 'active'
        self.trace = getattr(self, 'trace', []) + [tuple(points[0])] + list(points[1:])

    def step(self, dt):
        super().step(dt)
        if self._state == 'succeeded' and getattr(self, 'via', None):
            self.goal, self._state = self.via.pop(0), 'active'


def test_grid_mode_layout4_visits_all_zones_without_going_back_into_start():
    from test_grid_planner import LAYOUT4, ROUTE4, planner
    g = planner(LAYOUT4)
    robot = {'xy': (4.5, 0.5)}                          # C_Z51, the real start
    nav = FakeNavVia(robot)
    ref = FakeRefbox(ROUTE4)
    logs = []
    task = NavigationTask(nav, F, log=logs.append, router=g, blocked_zones=[z for z, _ in LAYOUT4])
    t, path = 0.0, []
    while t < 600:
        ref.step(t, robot['xy'])
        info = task.update(t, True, ref.remaining, (*robot['xy'], 0.0))
        if info['state'] == DONE:
            break
        nav.step(0.1)
        path.append(robot['xy'])
        t += 0.1
    assert task.state == DONE and ref.remaining == []
    left_start = next(i for i, p in enumerate(path) if p[1] > 1.2)
    assert not any(2.0 < x < 5.0 and y < 0.9 for x, y in path[left_start:])   # never back into START
    assert t < 300, f'{t:.0f} s'


def test_no_progress_is_noticed_long_before_the_drive_timeout():
    class Stalled(FakeNav):
        def step(self, dt):
            pass                                        # Nav2 says active, robot does not move
    robot = {'xy': (2.5, 1.5)}
    nav = Stalled(robot)
    logs = []
    task = NavigationTask(nav, F, log=logs.append, drive_timeout=40.0, stuck_time=8.0)
    task.red_done, task.state = True, 'PICK'
    t = 0.0
    while t < 12:
        task.update(t, True, ['C_Z44'], (*robot['xy'], 0.0))
        t += 0.1
    assert any('no progress' in l for l in logs), logs


class FakeNavNudge:
    """Minimal nav for the fine-approach tests: every drive succeeds at once, nudges recorded."""

    def __init__(self):
        self.nudges, self.last = [], 'succeeded'

    def go_to(self, x, y, yaw):
        self.last = 'active'

    def state(self):
        return self.last

    def cancel(self):
        pass

    def clear_costmaps(self):
        pass

    def nudge(self, vx, vy):
        self.nudges.append((vx, vy))


def test_fine_approach_moves_onto_the_stop_point_during_the_dwell():
    from ired_challenge.navigation_task import NavigationTask
    nav = FakeNavNudge()
    fine = {'tol': 0.03, 'gain': 1.0, 'vmax': 0.08, 'vmin': 0.03, 'time': 3.0}
    t = NavigationTask(nav, F, log=lambda *_: None, order='given', fine=fine)
    t.red_done = True
    t._set('PICK', 0.0)
    t.update(0.0, True, ['C_Z44'], (3.5, 2.5, 0.0))           # PICK -> DRIVE
    t.goal_xy = (3.5, 3.5)
    nav.last = 'succeeded'
    t.update(0.1, True, ['C_Z44'], (3.45, 3.43, math.pi))     # reached 9 cm off -> DWELL
    assert t.state == 'DWELL'
    t.update(0.2, True, ['C_Z44'], (3.45, 3.43, math.pi))     # correction
    vx, vy = nav.nudges[-1]
    # robot faces -x: +x field error (0.05) is -x robot, +y field error (0.07) is -y robot
    assert vx < 0 and vy < 0 and 0.03 <= math.hypot(vx, vy) <= 0.08 + 1e-9
    t.update(0.3, True, ['C_Z44'], (3.49, 3.49, math.pi))     # within 3 cm: stop once
    assert nav.nudges[-1] == (0.0, 0.0)
    n = len(nav.nudges)
    t.update(0.4, True, ['C_Z44'], (3.49, 3.49, math.pi))
    assert len(nav.nudges) == n                               # nothing more after stopping


def test_fine_approach_gives_up_after_its_time_and_stops():
    from ired_challenge.navigation_task import NavigationTask
    nav = FakeNavNudge()
    fine = {'tol': 0.03, 'gain': 1.0, 'vmax': 0.08, 'vmin': 0.03, 'time': 3.0}
    t = NavigationTask(nav, F, log=lambda *_: None, order='given', fine=fine)
    t.red_done = True
    t._set('PICK', 0.0)
    t.update(0.0, True, ['C_Z44'], (3.5, 2.5, 0.0))
    t.goal_xy = (3.5, 3.5)
    nav.last = 'succeeded'
    t.update(0.1, True, ['C_Z44'], (3.45, 3.43, 0.0))
    for k in range(2, 45):                                    # robot never moves (wheels slip)
        t.update(k / 10, True, ['C_Z44'], (3.45, 3.43, 0.0))
    assert nav.nudges[-1] == (0.0, 0.0) and t.state == 'DWELL'


def test_fine_approach_off_without_param_or_nudge():
    from ired_challenge.navigation_task import NavigationTask
    class NoNudge:
        pass
    fine = {'tol': 0.03, 'gain': 1.0, 'vmax': 0.08, 'vmin': 0.03, 'time': 3.0}
    assert NavigationTask(FakeNavNudge(), F, log=lambda *_: None).fine is None       # param off
    assert NavigationTask(NoNudge(), F, log=lambda *_: None, fine=fine).fine is None  # nav cannot nudge


BACKOFF = {'dist': 0.50, 'speed': 0.06, 'time': 4.0}


def box_dwell(nav, backoff=BACKOFF, pose=(2.20, 2.26, math.pi)):
    """Robot dwelling in C_Z33 (box at its centre) at the 2026-10-26 robot stop point."""
    t = NavigationTask(nav, F, log=lambda *_: None, order='given', blocked_zones=['C_Z33'], backoff=backoff)
    t.red_done = True
    t._set('PICK', 0.0)
    t.update(0.0, True, ['C_Z33', 'C_Z34'], (2.0, 1.5, math.pi))
    nav.last = 'succeeded'
    t.update(0.1, True, ['C_Z33', 'C_Z34'], pose)
    assert t.state == 'DWELL'
    t.update(6.0, True, ['C_Z34'], pose)                     # refbox counted C_Z33
    return t


def test_backoff_slides_straight_away_from_the_box_then_picks():
    nav = FakeNavNudge()
    t = box_dwell(nav)
    assert t.state == 'BACKOFF' and 'C_Z33' in t.visited
    t.update(6.1, True, ['C_Z34'], (2.20, 2.26, math.pi))
    vx, vy = nav.nudges[-1]
    # away from (2.5, 2.5) is (-0.30, -0.24) in the field; robot faces -x, so +x and +y robot
    assert vx > 0 and vy > 0 and math.hypot(vx, vy) == pytest.approx(0.06)
    t.update(7.0, True, ['C_Z34'], (2.12, 2.14, math.pi))     # 0.50 m from the box centre
    assert t.state == 'PICK' and nav.nudges[-1] == (0.0, 0.0)


def test_backoff_gives_up_after_its_time_and_near_walls():
    nav = FakeNavNudge()
    t = box_dwell(nav)
    t.update(6.1, True, ['C_Z34'], (2.20, 2.26, math.pi))
    t.update(10.5, True, ['C_Z34'], (2.20, 2.26, math.pi))    # stuck: time is up
    assert t.state == 'PICK' and nav.nudges[-1] == (0.0, 0.0)
    # box in C_Z12 with the robot towards the wall side: the next 0.3 m would reach the wall
    nav = FakeNavNudge()
    t = NavigationTask(nav, F, log=lambda *_: None, blocked_zones=['C_Z12'], backoff=BACKOFF)
    t.backoff_from, t.state, t.t_state = F.center('C_Z12'), 'BACKOFF', 0.0
    t.update(0.1, True, ['C_Z34'], (0.20, 1.5, 0.0))
    assert t.state == 'PICK'


def test_no_backoff_when_far_from_the_box_or_off():
    nav = FakeNavNudge()
    t = box_dwell(nav, pose=(2.08, 2.10, math.pi))           # 0.58 m from the box centre
    assert t.state == 'PICK'
    nav = FakeNavNudge()
    t = box_dwell(nav, backoff=None)
    assert t.state == 'PICK' and not nav.nudges
