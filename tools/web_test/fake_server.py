"""Fake ired_web manager for testing the web pages on the PC without the robot.

usage: python3 fake_server.py [PORT]   then open http://127.0.0.1:PORT (any PIN works)
Serves robot_ws/src/ired_web/web and answers the /api/* calls with made-up data:
Navigation (omni) running, the field-aligned map from robot_ws maps, robot at C_Z51.
Saved maps (list, View, Align, Use for Navigation) work on a temporary copy of
robot_ws/src/ired_navigation/maps through the real MapStore. Run after
`source /opt/ros/jazzy/setup.bash`. IRED_WS_SRC=<path to src> serves another checkout.
No rosbridge: buttons that publish to ROS say 'Not connected to ROS'. Stop it with
fuser -k PORT/tcp (not pkill -f)."""
import http.server
import io
import json
import os
import sys
import time
import math

import shutil
import tempfile

import yaml
from PIL import Image

# IRED_WS_SRC: another checkout's src (e.g. a git worktree), default robot_ws/src
SRC = os.environ.get('IRED_WS_SRC') or os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src')
sys.path.insert(0, os.path.join(SRC, 'ired_web'))
sys.path.insert(0, os.path.join(SRC, 'ired_challenge'))
from ired_web.mapping import MapStore  # noqa: E402  (needs `source /opt/ros/jazzy/setup.bash`)

HERE = os.path.dirname(os.path.abspath(__file__))
WS = SRC
WEB = os.path.join(WS, 'ired_web', 'web')
MAP = os.path.join(WS, 'ired_navigation', 'maps', 'Titan-1-cleaned-field')

# saved maps: a throw-away copy of the robot's maps, used through the real MapStore
TMP = tempfile.mkdtemp(prefix='ired_web_test_')
for f in os.listdir(os.path.dirname(MAP)):
    if f.endswith(('.yaml', '.pgm')):
        shutil.copy(os.path.join(os.path.dirname(MAP), f), TMP)
STORE = MapStore(TMP, None, TMP)
meta = yaml.safe_load(open(MAP + '.yaml'))
img = Image.open(MAP + '.pgm')
buf = io.BytesIO(); img.save(buf, format='PNG'); PNG = buf.getvalue()

PROFILES = [
    {'name': 'robot', 'title': 'Robot', 'description': 'fake', 'state': 'running'},
    {'name': 'mapping', 'title': 'Mapping (SLAM)', 'description': 'fake', 'state': 'stopped'},
    {'name': 'navigation', 'title': 'Navigation', 'description': 'fake', 'state': 'stopped'},
    {'name': 'navigation_omni', 'title': 'Navigation (omni)', 'description': 'fake', 'state': 'running'},
]


for profile in PROFILES:
    profile.update(requires=[], uptime=60, exit_code=None)


from ired_web.practice import PracticeGame, ACTIVE, KINDS, ZONES
from ired_challenge.exploration import allowed_rotations


class FakePractice:
    """Real rules/engine/persistence, fake connection and stationary pose only."""
    def __init__(self):
        self.source = 'field'
        self.game = PracticeGame(os.path.join(TMP, 'practice'), lambda *a: None,
                                 lambda: [] if self.source == 'practice' else ['Select Practice first'])

    field = {'address': '172.16.1.4', 'default': '172.16.1.4', 'robot_ip': '172.16.1.226'}

    def status(self):
        self.game.on_pose(4.5, .5, math.pi)
        self.game.tick()
        return dict(self.game.snapshot(), source=self.source, busy=False, error='',
                    fake=True, broker_connected=self.source == 'practice', field_refbox=dict(self.field),
                    ready_reasons=[] if self.source == 'practice' else ['Select Practice first'],
                    layouts=self.game.saved(), history=self.game.history(),
                    placement_rules={name: {z: allowed_rotations(z, name) for z in ZONES if allowed_rotations(z, name)} for name in KINDS[4]})

    def api(self, action, body):
        if action == 'source':
            if body.get('source') not in ('practice', 'field') or self.game.state in ACTIVE:
                raise ValueError('Stop the current game before changing source')
            self.source = body['source']
        elif action == 'prepare': self.game.prepare(body)
        elif action == 'start': self.game.start(body.get('layout_id'), body.get('confirmed', False))
        elif action == 'stop': self.game.stop()
        elif action == 'field_refbox':
            self.field = dict(self.field, address=(body.get('address') or '').strip() or self.field['default'])
            self.field.pop('broker_ok', None)
        elif action == 'field_test': self.field = dict(self.field, broker_ok=False, broker_error='PC demo: no broker')
        # demo only: what the robot's exploration machine_report would send over MQTT
        elif action == 'report': self.game.on_report(body.get('machine'), body.get('x'), body.get('y'), body.get('yaw'))
        else: raise ValueError('Unknown practice action')
        return self.status()


PRACTICE = FakePractice()


class H(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=WEB, **k)

    def log_message(self, *a):
        pass

    def send_json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        p = self.path.split('?')[0]
        if not p.startswith('/api/'):
            return super().do_GET()
        p = p[5:]
        if p == 'config':
            return self.send_json({'rosbridge_port': 9, 'video_port': 9, 'foxglove_port': 9})
        if p == 'practice/status':
            return self.send_json(PRACTICE.status())
        if p == 'status':
            return self.send_json({'profiles': PROFILES, 'estop': False, 'time': time.time(), 'topics': {}, 'nodes': [],
                                   'system': {'cpu_percent': 5, 'load': [0, 0, 0], 'temp_c': 40,
                                              'fan_pwm_percent': 0, 'fan_rpm': None, 'throttled': '0x0',
                                              'ram_percent': 20, 'disk_percent': 30, 'wifi': None,
                                              'uptime_s': 60, 'ips': ['127.0.0.1']}})
        if p == 'preflight':
            ck = lambda k, l, st, v, fix='': {'key': k, 'label': l, 'state': st, 'value': v, 'fix': fix}
            checks = [ck('estop', 'STOP released', 'ok', 'released'), ck('robot', 'Robot mode', 'ok', 'running'),
                      ck('motors', 'Wheel board talking', 'ok', '10 Hz', 'Drive switch cannot be checked without moving: make sure it is ON'),
                      ck('lidar', 'LiDAR', 'ok', '7.3 Hz'),
                      ck('nav', 'Navigation', 'ok', 'Navigation (soft-stop), map Titan-1-cleaned-field'),
                      ck('start', 'Start pose C_Z51', 'warn', '(4.32, 0.61) 21 cm / 4 deg from the start',
                         'Put the robot on C_Z51 facing C_Z41, Map page: "Robot is at C_Z51"'),
                      ck('challenge', 'Challenge program + refbox', 'warn', 'source practice, challenge stopped',
                         'Game page: Prepare (practice) or Launch: Challenge: Navigation (field refbox)'),
                      ck('temp', 'Pi temperature', 'ok', '46 C'), ck('cpu', 'Pi CPU', 'ok', '23 %'),
                      ck('battery', 'Battery', 'unknown', 'no sensor', 'Check the battery voltage by hand before the game')]
            return self.send_json({'time': time.time(), 'overall': 'warn', 'checks': checks})
        if p == 'map/info':
            return self.send_json({'have': True, 'version': 1, 'width': img.width, 'height': img.height,
                                   'resolution': meta['resolution'], 'origin': meta['origin'][:2],
                                   'pose': [4.5, 0.5, 3.14159], 'mapping': 'stopped',
                                   'maps': STORE.list(), 'active': STORE.active()})
        if p == 'map/png':
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', str(len(PNG)))
            self.end_headers()
            return self.wfile.write(PNG)
        if p == 'map/zones':
            return self.send_json({'map': STORE.active(), 'zones': STORE.zones(STORE.active())})
        if p == 'map/preview':
            from urllib.parse import parse_qs, urlparse
            try:
                q = parse_qs(urlparse(self.path).query)
                png = STORE.preview_png(q.get('name', [''])[0], dx=float(q.get('dx', ['0'])[0]),
                                        dy=float(q.get('dy', ['0'])[0]), dyaw_deg=float(q.get('dyaw_deg', ['0'])[0]))
            except ValueError as e:
                return self.send_json({'error': str(e)}, 404)
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.end_headers()
            return self.wfile.write(png)
        if p.startswith('tests'):
            return self.send_json({'tests': [], 'running': None})
        if p.endswith('/log'):
            return self.send_json({'log': ''})
        return self.send_json({})

    def do_POST(self):
        n = int(self.headers.get('Content-Length') or 0)
        body = json.loads(self.rfile.read(n) or b'{}')
        try:
            if self.path.startswith('/api/practice/'):
                if not isinstance(body, dict):
                    raise ValueError('Expected JSON object')
                return self.send_json(PRACTICE.api(self.path.rsplit('/', 1)[-1], body))
            if self.path == '/api/map/nudge':
                return self.send_json(STORE.nudge(body.get('name', ''), body.get('dx', 0), body.get('dy', 0), body.get('dyaw_deg', 0)))
            if self.path == '/api/map/delete':
                return self.send_json(STORE.delete(body.get('name', ''), bool(body.get('original', False))))
            if self.path == '/api/map/clean':
                return self.send_json(STORE.clean(body.get('name', '')))
            if self.path == '/api/map/align':
                return self.send_json(STORE.align(body.get('name', ''), body.get('start', 'auto')))
            if self.path == '/api/map/zones':
                z = STORE.set_zone(STORE.active(), body.get('zone', ''), None if body.get('clear') else body.get('xy'))
                return self.send_json({'map': STORE.active(), 'zones': z})
            if self.path == '/api/map/select':
                STORE.select(body.get('name', ''))
                return self.send_json({'active': body.get('name')})
        except (ValueError, TypeError) as e:
            return self.send_json({'error': str(e)}, 400)
        return self.send_json({'ok': True})


port = int(sys.argv[1]) if len(sys.argv) > 1 else 8090
http.server.ThreadingHTTPServer(('127.0.0.1', port), H).serve_forever()
