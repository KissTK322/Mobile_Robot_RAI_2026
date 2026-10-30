"""One-button field game preparation: order of the steps, the Nav2 restart, failures reported."""
import math

from ired_web.field_prep import FieldPrep


class Clock:
    def __init__(self): self.t = 0.0
    def __call__(self): return self.t
    def sleep(self, dt): self.t += dt


class Hub:
    def __init__(self, nav_ok_after=1, pose=(4.5, 0.5, math.pi)):
        self.calls, self.on = [], set()
        self.nav_checks, self.nav_ok_after, self.pose_value = 0, nav_ok_after, pose
        self.src, self.stop_on = 'field', False
        self.pose_sent = False
    def source(self): return self.src
    def estop(self): return self.stop_on
    def running(self, n): return n in self.on
    def running_nav(self): return next((n for n in self.on if n.startswith('navigation')), None)
    def start(self, n): self.calls.append(('start', n)); self.on.add(n)
    def stop(self, n): self.calls.append(('stop', n)); self.on.discard(n)
    def rates(self): return {'/ired/motor/data': 10, '/scan_filtered': 7} if 'robot' in self.on else {}
    def nav2_active(self):
        self.nav_checks += 1
        return self.nav_ok_after is not None and self.nav_checks >= self.nav_ok_after
    def challenge_ready(self, mode): return any(n.startswith('challenge') for n in self.on)
    def send_start_pose(self, *p): self.calls.append(('pose',)); self.pose_sent = True
    def pose(self): return self.pose_value if self.pose_sent else (0.0, 0.0, 0.0)


def run(hub, mode='navigation'):
    c = Clock()
    p = FieldPrep(hub, clock=c, sleep=c.sleep)
    p.mode, p.busy = mode, True
    p.steps = [{'key': k, 'label': '', 'state': 'pending', 'msg': ''} for k in
               ('estop', 'robot', 'nav', 'camera', 'challenge', 'pose') if k != 'camera' or mode == 'exploration']
    p.run()
    return p.status()


def test_order_and_pose_after_challenge():
    h = Hub()
    s = run(h)
    assert s['done'] and not s['error'] and all(x['state'] == 'ok' for x in s['steps'])
    assert h.calls == [('start', 'robot'), ('start', 'navigation_quick'), ('start', 'challenge_nav'), ('pose',)]


def test_nav2_restarted_once_when_it_hangs():
    h = Hub(nav_ok_after=70)              # never active within the first 60 s of checks
    s = run(h)
    assert s['done'] and ('stop', 'navigation_quick') in h.calls
    assert h.calls.count(('start', 'navigation_quick')) == 2


def test_nav2_never_active_is_reported():
    s = run(Hub(nav_ok_after=None))
    assert not s['done'] and 'did not become active' in s['error']
    assert [x['state'] for x in s['steps']][:3] == ['ok', 'ok', 'bad']


def test_refuses_practice_source_and_stop():
    h = Hub(); h.src = 'practice'
    assert 'Field refbox' in run(h)['error'] and not h.calls
    h = Hub(); h.stop_on = True
    assert 'STOP' in run(h)['error'] and not h.calls


def test_exploration_starts_camera_and_lidar_challenge():
    h = Hub()
    s = run(h, 'exploration')
    assert s['done'] and ('start', 'camera') in h.calls and ('start', 'challenge_exp_lidar') in h.calls


def test_exploration_sweep_stays_available_as_plan_b():
    h = Hub()
    s = run(h, 'exploration_sweep')
    assert s['done'] and ('start', 'camera') in h.calls and ('start', 'challenge_exp') in h.calls


def test_wrong_place_is_reported():
    h = Hub(pose=(3.2, 0.5, math.pi))
    s = run(h)
    assert not s['done'] and 'C_Z51' in s['error']


def test_existing_navigation_profile_is_kept():
    h = Hub(); h.on = {'robot', 'navigation_softstop'}
    run(h)
    assert ('start', 'navigation_quick') not in h.calls
