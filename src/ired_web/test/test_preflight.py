import math
import types

from ired_web.manager import Manager


class P:
    def __init__(self, state, command='', title=''):
        self.state, self.command, self.spec = state, command, {'title': title}

    def info(self):
        return {'state': self.state}


def fake(robot='running', nav='running', pose=(4.5, 0.5, math.pi), rates=None, nodes=('map_server', 'amcl', 'controller_server'),
         active='Field-adj', estop=False, temp=45.0, nav2=True):
    m = Manager.__new__(Manager)
    m.last_viewed = {'status': 0.0, 'gains': 0.0, 'map': 0.0, 'ready': 0.0, 'game': 0.0}
    m.idle_check = lambda: None
    m.profiles = {'robot': P(robot), 'navigation_softstop': P(nav, f'... map_name:={active}.yaml param_name:=x', 'Navigation (soft-stop)'),
                  'challenge_nav': P('stopped')}
    m.node = types.SimpleNamespace(estop=estop, rates=lambda: rates if rates is not None else {'/ired/motor/data': 10, '/scan_filtered': 7.3},
                                   get_node_names=lambda: list(nodes),
                                   mapwatch=types.SimpleNamespace(pose=lambda: pose))
    m.system = types.SimpleNamespace(snapshot=lambda: {'temp_c': temp, 'throttled': '0x0', 'cpu_percent': 20.0})
    m.maps = types.SimpleNamespace(active=lambda: active)
    m.practice = types.SimpleNamespace(source='practice')
    m.field_prep = types.SimpleNamespace(hub=types.SimpleNamespace(nav2_active=lambda: nav2))
    return m


def states(r):
    return {c['key']: c['state'] for c in r['checks']}


def test_all_ready_except_challenge_and_battery():
    s = states(fake().preflight())
    assert all(s[k] == 'ok' for k in ('estop', 'robot', 'motors', 'lidar', 'nav', 'start', 'temp', 'cpu'))
    assert s['challenge'] == 'warn' and s['battery'] == 'unknown'


def test_robot_off_is_red():
    r = fake(robot='stopped', nav='stopped', pose=None, rates={}).preflight()
    s = states(r)
    assert r['overall'] == 'bad' and s['robot'] == 'bad' and s['nav'] == 'bad'
    assert s['motors'] == 'unknown' and s['lidar'] == 'unknown'      # no data expected without Robot


def test_start_pose_off_and_map_changed_warn():
    m = fake(pose=(4.30, 0.60, math.pi - 0.3))
    m.maps = types.SimpleNamespace(active=lambda: 'Other-map')
    s = states(m.preflight())
    assert s['start'] == 'warn' and s['nav'] == 'warn'


def test_hot_pi_and_estop():
    s = states(fake(temp=82.0, estop=True).preflight())
    assert s['temp'] == 'bad' and s['estop'] == 'bad'


def test_nav_bad_when_nav2_lifecycle_hangs():
    # robot 2026-10-27: nodes existed but controller_server stayed unconfigured
    r = fake(nav2=False).preflight()
    c = next(c for c in r['checks'] if c['key'] == 'nav')
    assert c['state'] == 'bad' and 'NOT active' in c['value']
