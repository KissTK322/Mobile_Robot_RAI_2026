#!/usr/bin/env python3
"""iRED web manager.

Serves the web app, starts/stops launch profiles (so nobody has to type
`ros2 launch` in a terminal), reports system status and handles the E-STOP.
Every /api/ call except /api/ping needs the PIN from config/web.yaml in the
X-Pin header."""
import hmac
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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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


class WebNode(Node):
    """ROS side: topic rates, node list and the E-STOP zero-velocity publisher."""

    def __init__(self):
        super().__init__('ired_web_manager')
        self.stamps = {t: [] for t, _, _ in WATCHED_TOPICS}
        self.stamps_lock = threading.Lock()
        for topic, msg_type, qos in WATCHED_TOPICS:
            self.create_subscription(msg_type, topic, lambda _m, t=topic: self.tick(t), qos)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.estop = False
        self.create_timer(0.05, self.estop_timer)

    def tick(self, topic):
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

    def estop_timer(self):
        # While latched, keep commanding zero so nothing else can move the wheels for long
        if self.estop:
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

    def status(self):
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
            ok, why = manager.check_pin(self.client_address[0], self.headers.get('X-Pin'))
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
                manager.node.estop = bool(self.body_json().get('active', True))
                return self.send_json(200, {'estop': manager.node.estop})
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

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
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
