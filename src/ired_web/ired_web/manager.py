#!/usr/bin/env python3
"""iRED web manager.

Serves the web app, starts/stops launch profiles (so nobody has to type
`ros2 launch` in a terminal), reports system status and handles the E-STOP.
Every /api/ call except /api/ping needs the PIN from config/web.yaml in the
X-Pin header."""
import csv
import hmac
import io
import json
import os
import signal
import subprocess
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import psutil
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from ired_msgs.msg import MotorData

SHARE = get_package_share_directory('ired_web')
ROS_SETUP = 'source /opt/ros/jazzy/setup.bash && source ~/ros_environment.bash'

# (topic, type, qos) shown with their live rate on the Status page
WATCHED_TOPICS = [
    ('/scan_filtered', LaserScan, qos_profile_sensor_data),
    ('/odometry/filtered', Odometry, 10),
    ('/ired/imu/data', Imu, 10),
    ('/ired/motor/data', MotorData, 10),
    ('/aruco/detections', String, 10),
    ('/cmd_vel', TwistStamped, 10),
]


def load_config():
    path = os.path.join(SHARE, 'config', 'web.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    cfg['log_dir'] = os.path.expanduser(cfg.get('log_dir', '~/.ros/ired_web'))
    os.makedirs(cfg['log_dir'], exist_ok=True)
    return cfg


class Profile:
    """One startable group of ROS nodes, run in its own process group."""

    def __init__(self, name, spec, cfg):
        self.name = name
        self.spec = spec
        self.command = spec['command'].format(**cfg)
        self.log_path = os.path.join(cfg['log_dir'], f'{name}.log')
        self.proc = None
        self.state = 'stopped'
        self.started_at = None
        self.exit_code = None
        self.lock = threading.Lock()

    def start(self):
        with self.lock:
            if self.proc and self.proc.poll() is None:
                return
            log = open(self.log_path, 'w')
            log.write(f'$ {self.command}\n')
            log.flush()
            self.proc = subprocess.Popen(
                ['bash', '-c', f'{ROS_SETUP} && exec {self.command}'],
                stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                cwd=os.path.expanduser('~'), start_new_session=True)
            log.close()
            self.state = 'starting'
            self.started_at = time.time()
            self.exit_code = None

    def stop(self):
        """SIGINT the whole group (every node, not just ros2 launch), then escalate."""
        with self.lock:
            proc = self.proc
            if not proc or proc.poll() is not None:
                return
            self.state = 'stopping'
        for sig, wait in ((signal.SIGINT, 10.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            end = time.time() + wait
            while time.time() < end and self.group_alive(proc.pid):
                time.sleep(0.2)
            if not self.group_alive(proc.pid):
                break
        proc.wait(timeout=5)
        with self.lock:
            self.state = 'stopped'
            self.exit_code = proc.returncode

    @staticmethod
    def group_alive(pgid):
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False

    def refresh(self):
        with self.lock:
            if self.state in ('starting', 'running') and self.proc and self.proc.poll() is not None:
                self.exit_code = self.proc.returncode
                self.state = 'failed' if self.exit_code not in (0, -2) else 'stopped'
            elif self.state == 'starting' and time.time() - self.started_at > 5.0:
                self.state = 'running'

    def info(self):
        self.refresh()
        return {
            'name': self.name,
            'title': self.spec.get('title', self.name),
            'description': self.spec.get('description', ''),
            'hidden': bool(self.spec.get('hidden', False)),
            'requires': self.spec.get('requires', []),
            'conflicts': self.spec.get('conflicts', []),
            'state': self.state,
            'uptime': round(time.time() - self.started_at) if self.state in ('starting', 'running') else 0,
            'exit_code': self.exit_code,
        }

    def tail(self, lines):
        try:
            with open(self.log_path, 'rb') as f:
                f.seek(0, os.SEEK_END)
                f.seek(max(0, f.tell() - 200_000))
                text = f.read().decode('utf-8', 'replace')
        except FileNotFoundError:
            return ''
        return '\n'.join(text.splitlines()[-lines:])


def _num(params, key, lo, hi, default):
    v = float(params.get(key, default))
    if not lo <= v <= hi:
        raise ValueError(f'{key} must be between {lo} and {hi}')
    return v


def test_command(name, params):
    """Validated `ros2 run ired_web ...` arguments for a test, or ValueError."""
    if name == 'rotate':
        turns = _num(params, 'turns', 0.25, 3, 1)
        speed = _num(params, 'speed', 0.2, 0.8, 0.5)
        sign = -1 if params.get('direction') == 'cw' else 1
        return ['test_rotate', f'{turns:g}', f'{sign * speed:g}']
    if name == 'straight':
        dist = _num(params, 'distance', 0.2, 1.5, 1.0)
        speed = _num(params, 'speed', 0.05, 0.3, 0.15)
        sign = -1 if params.get('direction') == 'backward' else 1
        return ['test_straight', f'{sign * dist:g}', f'{speed:g}']
    if name == 'wheel':
        return ['test_wheel']
    if name == 'pid':
        wheel = str(params.get('wheel', 'ALL')).upper()
        if wheel not in ('ALL', 'FL', 'FR', 'RL', 'RR'):
            raise ValueError('wheel must be ALL, FL, FR, RL or RR')
        return ['test_pid_set', wheel, f"{_num(params, 'kp', 0, 100, 0):g}",
                f"{_num(params, 'ki', 0, 200, 0):g}", f"{_num(params, 'kd', 0, 10, 0):g}"]
    raise ValueError('unknown test')


class TestJob:
    """One calibration test run as `ros2 run ired_web test_*`, reporting JSON lines."""

    def __init__(self, name, params, args, results_path):
        self.name, self.params, self.args = name, params, args
        self.results_path = results_path
        self.state = 'running'
        self.progress = 'Starting'
        self.result = None
        self.error = None
        self.started = time.time()
        self.proc = subprocess.Popen(
            ['bash', '-c', f"{ROS_SETUP} && exec ros2 run ired_web {' '.join(args)}"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
            text=True, cwd=os.path.expanduser('~'), start_new_session=True)
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        for line in self.proc.stdout:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if 'progress' in msg:
                self.progress = msg['progress']
            if 'result' in msg:
                self.result = msg['result']
            if 'error' in msg:
                self.error = msg['error']
        self.proc.wait()
        if self.state != 'aborted':
            self.state = 'done' if self.result is not None else 'failed'
        if self.result is not None:
            with open(self.results_path, 'a') as f:
                f.write(json.dumps({'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'test': self.name,
                                    'params': self.params, 'result': self.result}) + '\n')

    def stop(self):
        if self.proc.poll() is not None:
            return
        self.state = 'aborted'
        for sig, wait in ((signal.SIGINT, 5.0), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                return
            end = time.time() + wait
            while time.time() < end and self.proc.poll() is None:
                time.sleep(0.1)
            if self.proc.poll() is not None:
                return

    def info(self):
        return {'test': self.name, 'params': self.params, 'state': self.state, 'progress': self.progress,
                'result': self.result, 'error': self.error, 'elapsed': round(time.time() - self.started)}


class WebNode(Node):
    """ROS side: topic rates, node list and the E-STOP zero-velocity publisher."""

    def __init__(self):
        super().__init__('ired_web_manager')
        self.stamps = {t: [] for t, _, _ in WATCHED_TOPICS}
        self.stamps_lock = threading.Lock()
        self.pid_gains = None
        self.pid_gains_at = 0.0
        self.subs = []
        self.watch_lock = threading.Lock()
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self._estop = False
        self.estop_timer_handle = self.create_timer(0.05, self.estop_timer)
        self.estop_timer_handle.cancel()     # only runs while STOP is latched

    def watch(self, on):
        """Subscribe to the watched topics only while someone has the app open.

        raw=True: messages are only counted, never unpacked; wheel data is unpacked
        once a second for the PID gains shown on the Test page."""
        with self.watch_lock:
            if on and not self.subs:
                self.subs = [self.create_subscription(t, n, lambda m, n=n: self.tick(n, m), q, raw=True)
                             for n, t, q in WATCHED_TOPICS]
            elif not on and self.subs:
                for sub in self.subs:
                    self.destroy_subscription(sub)
                self.subs = []
                with self.stamps_lock:
                    for s in self.stamps.values():
                        s.clear()
                self.pid_gains = None

    def tick(self, topic, raw):
        if topic == '/ired/motor/data' and time.time() - self.pid_gains_at > 1.0:
            self.pid_gains_at = time.time()
            msg = deserialize_message(raw, MotorData)
            self.pid_gains = {'FL': list(msg.pid_motor_front_left), 'FR': list(msg.pid_motor_front_right),
                              'RL': list(msg.pid_motor_rear_left), 'RR': list(msg.pid_motor_rear_right)}
        now = time.time()
        with self.stamps_lock:
            s = self.stamps[topic]
            s.append(now)
            while s and now - s[0] > 3.0:
                s.pop(0)

    def rates(self):
        now = time.time()
        out = {}
        with self.stamps_lock:
            for topic, s in self.stamps.items():
                recent = [t for t in s if now - t <= 2.0]
                out[topic] = round(len(recent) / 2.0, 1)
        return out

    @property
    def estop(self):
        return self._estop

    @estop.setter
    def estop(self, active):
        self._estop = active
        if active:
            self.estop_timer_handle.reset()
        else:
            self.estop_timer_handle.cancel()

    def estop_timer(self):
        # While latched, keep commanding zero so nothing else can move the wheels for long
        if self._estop:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            self.cmd_pub.publish(msg)


def read_file(path, default=''):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def find_hwmon(name):
    base = '/sys/class/hwmon'
    for d in os.listdir(base):
        if read_file(os.path.join(base, d, 'name')) == name:
            return os.path.join(base, d)
    return None


class System:
    """Pi health numbers for the Status page."""

    def __init__(self):
        self.fan = find_hwmon('pwmfan')
        self.throttled = ''
        self.throttled_at = 0.0
        psutil.cpu_percent(None)

    def wifi(self):
        for line in read_file('/proc/net/wireless').splitlines()[2:]:
            parts = line.split()
            if len(parts) > 3:
                return {'iface': parts[0].rstrip(':'), 'signal_dbm': float(parts[3].rstrip('.'))}
        return None

    def snapshot(self):
        if time.time() - self.throttled_at > 5.0:
            try:
                self.throttled = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True,
                                                text=True, timeout=2).stdout.strip().split('=')[-1]
            except (OSError, subprocess.TimeoutExpired):
                self.throttled = ''
            self.throttled_at = time.time()
        vm = psutil.virtual_memory()
        du = psutil.disk_usage('/')
        ips = [a.address for name, addrs in psutil.net_if_addrs().items() if name != 'lo'
               for a in addrs if a.family.name == 'AF_INET']
        return {
            'cpu_percent': psutil.cpu_percent(None),
            'load': [round(x, 2) for x in os.getloadavg()],
            'temp_c': round(int(read_file('/sys/class/thermal/thermal_zone0/temp', '0')) / 1000, 1),
            'ram_percent': vm.percent,
            'disk_percent': du.percent,
            'uptime_s': round(time.time() - psutil.boot_time()),
            'throttled': self.throttled,
            'fan_rpm': int(read_file(os.path.join(self.fan, 'fan1_input'), '0')) if self.fan else None,
            'fan_pwm_percent': round(int(read_file(os.path.join(self.fan, 'pwm1'), '0')) / 2.55) if self.fan else None,
            'wifi': self.wifi(),
            'ips': ips,
        }


class Manager:
    def __init__(self, cfg, node):
        self.cfg = cfg
        self.node = node
        self.system = System()
        self.profiles = {n: Profile(n, spec, cfg) for n, spec in cfg['profiles'].items()}
        self.failures = {}       # ip -> (count, locked_until)
        self.job = None
        self.last_viewed = 0.0
        self.results_path = os.path.join(cfg['log_dir'], 'results.jsonl')

    def check_pin(self, ip, pin):
        count, locked = self.failures.get(ip, (0, 0.0))
        if time.time() < locked:
            return False, 'locked'
        if pin and hmac.compare_digest(str(pin), str(self.cfg['pin'])):
            self.failures.pop(ip, None)
            return True, ''
        count += 1
        self.failures[ip] = (count, time.time() + 30.0 if count >= 5 else 0.0)
        return False, 'locked' if count >= 5 else 'bad_pin'

    def start(self, name):
        p = self.profiles[name]
        for other in p.spec.get('conflicts', []):
            self.profiles[other].stop()
        for dep in p.spec.get('requires', []):
            d = self.profiles[dep]
            if d.info()['state'] not in ('starting', 'running'):
                d.start()
                time.sleep(4.0)
        p.start()

    def stop_all(self):
        threads = [threading.Thread(target=p.stop) for n, p in self.profiles.items() if not p.spec.get('autostart')]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    def set_estop(self, active):
        self.node.estop = active
        if active and self.job:
            threading.Thread(target=self.job.stop, daemon=True).start()

    def run_test(self, name, params):
        if self.job and self.job.proc.poll() is None:
            raise ValueError('Another test is still running')
        if self.node.estop:
            raise ValueError('Release STOP first')
        if self.profiles['robot'].info()['state'] != 'running':
            raise ValueError('Start Robot mode first')
        args = test_command(name, params)
        self.job = TestJob(name, params, args, self.results_path)

    def results(self, limit=None):
        try:
            with open(self.results_path) as f:
                rows = [json.loads(line) for line in f if line.strip()]
        except FileNotFoundError:
            rows = []
        return rows[-limit:] if limit else rows

    def results_csv(self):
        rows = self.results()
        keys = []
        for r in rows:
            for k in r['result']:
                if k != 'rows' and k not in keys:
                    keys.append(k)
        out = io.StringIO()
        w = csv.writer(out)
        w.writerow(['time', 'test', 'params'] + keys)
        for r in rows:
            res = r['result']
            w.writerow([r['time'], r['test'], json.dumps(r['params'])] +
                       [json.dumps(res[k]) if isinstance(res.get(k), (dict, list)) else res.get(k, '') for k in keys])
        return out.getvalue()

    def yaml_settings(self):
        """Startup values from ired_bringup/param/ired.yaml, for the Test page."""
        try:
            path = os.path.join(get_package_share_directory('ired_bringup'), 'param', 'ired.yaml')
            with open(path) as f:
                y = yaml.safe_load(f)
            m = y['iredcr_modbus_node']['ros__parameters']
            return {'pid': {w: m.get('pid_' + w.lower()) for w in ('FL', 'FR', 'RL', 'RR')},
                    'gyro_z_scale': y['imu_node']['ros__parameters'].get('gyro_z_scale')}
        except (OSError, KeyError, TypeError, yaml.YAMLError):
            return {}

    def viewed(self):
        self.last_viewed = time.time()
        self.node.watch(True)

    def idle_check(self):
        if time.time() - self.last_viewed > 5.0:
            self.node.watch(False)

    def tests_info(self):
        self.viewed()
        return {'job': self.job.info() if self.job else None, 'results': self.results(15),
                'gains': self.node.pid_gains, 'yaml': self.yaml_settings()}

    def status(self):
        self.viewed()
        return {
            'time': time.time(),
            'system': self.system.snapshot(),
            'profiles': [p.info() for p in self.profiles.values()],
            'topics': self.node.rates(),
            'nodes': sorted(self.node.get_node_names()),
            'estop': self.node.estop,
        }


def make_handler(manager):
    web_dir = os.path.join(SHARE, 'web')

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=web_dir, **kw)

        def log_message(self, fmt, *args):
            pass

        def end_headers(self):
            self.send_header('Cache-Control', 'no-store')
            super().end_headers()

        def send_json(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def body_json(self):
            n = int(self.headers.get('Content-Length') or 0)
            if not n:
                return {}
            try:
                return json.loads(self.rfile.read(n))
            except ValueError:
                return {}

        def api(self, method):
            url = urlparse(self.path)
            parts = [p for p in url.path.split('/') if p][1:]   # drop 'api'
            if parts == ['ping']:
                return self.send_json(200, {'ok': True})
            pin = self.headers.get('X-Pin') or parse_qs(url.query).get('pin', [None])[0]
            ok, why = manager.check_pin(self.client_address[0], pin)
            if not ok:
                return self.send_json(401, {'error': why})
            if method == 'GET' and parts == ['config']:
                c = manager.cfg
                return self.send_json(200, {k: c[k] for k in ('rosbridge_port', 'video_port', 'foxglove_port')})
            if method == 'GET' and parts == ['status']:
                return self.send_json(200, manager.status())
            if method == 'GET' and len(parts) == 3 and parts[0] == 'profiles' and parts[2] == 'log':
                p = manager.profiles.get(parts[1])
                if not p:
                    return self.send_json(404, {'error': 'no such profile'})
                lines = int(parse_qs(url.query).get('lines', ['200'])[0])
                return self.send_json(200, {'log': p.tail(min(lines, 2000))})
            if method == 'POST' and len(parts) == 3 and parts[0] == 'profiles' and parts[2] in ('start', 'stop'):
                if parts[1] not in manager.profiles:
                    return self.send_json(404, {'error': 'no such profile'})
                action = manager.start if parts[2] == 'start' else manager.profiles[parts[1]].stop
                threading.Thread(target=action, args=(parts[1],) if parts[2] == 'start' else (), daemon=True).start()
                return self.send_json(200, {'ok': True})
            if method == 'POST' and parts == ['stop_all']:
                threading.Thread(target=manager.stop_all, daemon=True).start()
                return self.send_json(200, {'ok': True})
            if method == 'POST' and parts == ['estop']:
                manager.set_estop(bool(self.body_json().get('active', True)))
                return self.send_json(200, {'estop': manager.node.estop})
            if method == 'GET' and parts == ['tests']:
                return self.send_json(200, manager.tests_info())
            if method == 'POST' and parts == ['tests', 'run']:
                body = self.body_json()
                try:
                    manager.run_test(str(body.get('test')), body.get('params') or {})
                except ValueError as e:
                    return self.send_json(400, {'error': str(e)})
                return self.send_json(200, {'ok': True})
            if method == 'POST' and parts == ['tests', 'stop']:
                if manager.job:
                    threading.Thread(target=manager.job.stop, daemon=True).start()
                return self.send_json(200, {'ok': True})
            if method == 'GET' and parts == ['tests', 'results.csv']:
                body = manager.results_csv().encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/csv; charset=utf-8')
                self.send_header('Content-Disposition', 'attachment; filename="ired_test_results.csv"')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            return self.send_json(404, {'error': 'unknown endpoint'})

        def do_GET(self):
            if self.path.startswith('/api/'):
                return self.api('GET')
            return super().do_GET()

        def do_POST(self):
            if self.path.startswith('/api/'):
                return self.api('POST')
            self.send_error(404)

    return Handler


def main(args=None):
    cfg = load_config()
    rclpy.init(args=args)
    node = WebNode()
    manager = Manager(cfg, node)
    for p in manager.profiles.values():
        if p.spec.get('autostart'):
            p.start()

    server = ThreadingHTTPServer(('0.0.0.0', int(cfg['http_port'])), make_handler(manager))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"iRED web app on http://0.0.0.0:{cfg['http_port']}")

    # ROS callbacks run on their own executor thread; the main thread only waits for a stop signal
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while not stop.wait(1.0):
            manager.idle_check()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        server.shutdown()
        if manager.job:
            manager.job.stop()
        threads = [threading.Thread(target=p.stop) for p in manager.profiles.values()]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
