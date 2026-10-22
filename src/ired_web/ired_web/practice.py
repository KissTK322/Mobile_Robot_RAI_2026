"""ROS-free practice refbox: explicit prepare/start/stop, immutable layouts and history.

The wire protocol matches tools/refbox_lite: 12 targets, five continuous seconds
inside each 1 m zone, plus ground-truth machines during PRODUCTION only.
No automatic start or recovery after a process restart or MQTT disconnect.
"""
import copy
import json
import math
import random
import re
import threading
import time
import uuid
from pathlib import Path

ZONES = [f'C_Z{x}{y}' for x in range(1, 6) for y in range(1, 6)]
START = {'C_Z31', 'C_Z41', 'C_Z51'}
KINDS = {2: ['C-BS', 'C-CS1'], 3: ['C-BS', 'C-CS1', 'C-RS1'],
         4: ['C-BS', 'C-CS1', 'C-RS1', 'C-RS2']}
ACTIVE = {'countdown', 'running'}


def integer(value, low, high, label):
    if isinstance(value, bool):
        raise ValueError(f'{label} must be an integer')
    try:
        number = int(value)
        if float(value) != number or not low <= number <= high:
            raise ValueError()
    except (ValueError, TypeError, OverflowError):
        raise ValueError(f'{label} must be {low}..{high}') from None
    return number


def near(a, b):
    return abs(int(a[3])-int(b[3])) <= 1 and abs(int(a[4])-int(b[4])) <= 1


def validate_layout(config):
    from ired_challenge.exploration import allowed_rotations
    if not isinstance(config, dict):
        raise ValueError('Expected a layout object')
    n = integer(config.get('n', 2), 2, 4, 'Box count')
    seed = integer(config.get('seed', 1), 0, 2147483647, 'Seed')
    limit = integer(config.get('limit_s', 600), 60, 1800, 'Game duration')
    countdown = integer(config.get('countdown_s', 5), 3, 30, 'Countdown')
    machines = config.get('machines')
    route = config.get('route')
    if not isinstance(machines, list) or len(machines) != n:
        raise ValueError('Selected box count must match the layout')
    if not all(isinstance(m, dict) for m in machines) or sorted(m.get('name', '') for m in machines) != sorted(KINDS[n]):
        raise ValueError('Machine names must match the selected level')
    cleaned = []
    for m in machines:
        zone = m.get('zone')
        if zone not in ZONES:
            raise ValueError('Invalid machine zone')
        angle = integer(m.get('rotation'), 0, 315, 'Rotation')
        if angle not in allowed_rotations(zone, m['name']):
            raise ValueError(f"{m['name']} cannot be placed in {zone} at {angle} degrees")
        if any(near(zone, prev['zone']) for prev in cleaned):
            raise ValueError('Boxes cannot share a zone or neighbouring zones')
        kind = m['name'].split('-')[1].rstrip('12')
        cleaned.append({'name': m['name'], 'type': kind, 'zone': zone,
                        'rotation': angle, 'state': 'IDLE', 'teamColor': 'CYAN'})
    if not isinstance(route, list) or len(route) != 12 or not all(isinstance(z, str) and z in ZONES and z not in START for z in route) or len(set(route)) != 12:
        raise ValueError('Route must have 12 unique target zones outside START')
    return {'n': n, 'seed': seed, 'limit_s': limit, 'countdown_s': countdown,
            'machines': cleaned, 'route': list(route)}


def generate_layout(config):
    from ired_challenge.exploration import allowed_rotations
    n = integer(config.get('n', 2), 2, 4, 'Box count')
    seed = integer(config.get('seed', random.SystemRandom().randrange(2147483647)), 0, 2147483647, 'Seed')
    rng = random.Random(seed)
    for _ in range(1000):
        machines = []
        for name in rng.sample(KINDS[n], n):
            options = [z for z in ZONES if allowed_rotations(z, name) and not any(near(z, m['zone']) for m in machines)]
            if not options:
                break
            zone = rng.choice(options)
            machines.append({'name': name, 'zone': zone, 'rotation': rng.choice(allowed_rotations(zone, name))})
        if len(machines) == n:
            return validate_layout(dict(config, n=n, seed=seed, machines=machines,
                                        route=rng.sample([z for z in ZONES if z not in START], 12)))
    raise ValueError('Could not generate a valid layout; try a different seed')


class PracticeGame:
    def __init__(self, folder, publish, ready=lambda: [], halt=lambda reason: None,
                 clock=time.monotonic, wall=time.time):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.publish = publish
        self.ready = ready
        self.halt = halt
        self.clock, self.wall = clock, wall
        self.lock = threading.RLock()
        self.state = 'idle'
        self.config = None
        self.layout_id = None
        self.run_id = None
        self.remaining = []
        self.done_at = {}
        self.inside = {}
        self.pose_at = None
        self.pose = None
        self.deadline = None
        self.started = None
        self.message = ''
        self.final_elapsed = None
        # Recover durable evidence, never resume motion or a countdown after restart.
        for path in self.folder.glob('run-*.json'):
            try:
                record = json.loads(path.read_text())
                if not isinstance(record, dict):
                    continue
            except (OSError, ValueError):
                continue
            if record.get('state') in ACTIVE:
                record.update(state='interrupted', reason='Web manager restarted', ended_at=self.wall())
                self._write(path.name, record)

    def _write(self, name, value):
        path = self.folder / name
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(value, indent=2))
        temp.replace(path)

    def _documents(self, pattern, limit):
        records = []
        paths = sorted(self.folder.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)
        for path in paths:
            try:
                record = json.loads(path.read_text())
                if not isinstance(record, dict):
                    continue
            except (OSError, ValueError):
                continue
            records.append((path, record))
            if len(records) >= limit:
                break
        return records

    def saved(self):
        with self.lock:
            return [dict(record, id=path.stem[7:]) for path, record in self._documents('layout-*.json', 50)]

    def history(self):
        with self.lock:
            return [record for _, record in self._documents('run-*.json', 30)]

    def prepare(self, body):
        with self.lock:
            if self.state in ACTIVE:
                raise ValueError('Stop the current game before changing the field')
            if body.get('layout_id'):
                ident = str(body['layout_id'])
                if not re.fullmatch(r'[0-9a-f]{32}', ident):
                    raise ValueError('Invalid saved layout')
                try:
                    config = json.loads((self.folder / f'layout-{ident}.json').read_text())
                except FileNotFoundError:
                    raise ValueError('Saved layout not found') from None
                # An explicitly selected level must agree with the loaded layout.
                if 'n' in body and integer(body['n'], 2, 4, 'Box count') != config['n']:
                    raise ValueError('Saved layout box count differs from the selected level')
                for key in ('limit_s', 'countdown_s'):
                    if key in body:
                        config[key] = body[key]
                config = validate_layout(config)
            else:
                config = validate_layout(body) if 'machines' in body else generate_layout(body)
            self.config = config
            self.layout_id = uuid.uuid4().hex
            self._write(f'layout-{self.layout_id}.json', config)
            self.state, self.message = 'prepared', 'Place the boxes, then start when ready.'
            self.run_id, self.started, self.final_elapsed = None, None, None
            self.remaining, self.done_at, self.inside = list(config['route']), {}, {}
            return self.snapshot()

    def start(self, layout_id, confirmed=False):
        with self.lock:
            if self.state != 'prepared' or layout_id != self.layout_id:
                raise ValueError('Prepare the current layout before starting')
            if confirmed is not True:
                raise ValueError('Confirm boxes placed and robot at C_Z51')
            problems = self.start_problems()
            if problems:
                raise ValueError('; '.join(problems))
            self.run_id = uuid.uuid4().hex
            self.state = 'countdown'
            self.deadline = self.clock()+self.config['countdown_s']
            self.message = 'Countdown: robot will move when it reaches zero.'
            self._record()
            self._publish()
            return self.snapshot()

    def start_problems(self):
        with self.lock:
            problems = list(self.ready())
            if self.pose_at is None or self.clock()-self.pose_at > 2:
                problems.append('Waiting for a fresh robot pose from the practice bridge')
            elif math.hypot(self.pose[0]-4.5, self.pose[1]-.5) > .35 or abs((self.pose[2]+2*math.pi) % (2*math.pi)-math.pi) > .52:
                problems.append('Place robot at C_Z51 facing C_Z41 and set its start pose')
            return problems

    def on_pose(self, x, y, yaw):
        with self.lock:
            if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (x, y, yaw)):
                return
            now = self.clock()
            if self.pose_at is None or now-self.pose_at > 1.8:
                self.inside.clear()
            self.pose_at, self.pose = now, (x, y, yaw)
            if self.state != 'running':
                return
            for zone in list(self.remaining):
                a, b = int(zone[3]), int(zone[4])
                if a-1 <= x < a and b-1 <= y < b:
                    since = self.inside.setdefault(zone, now)
                    if now-since >= 5:
                        self.remaining.remove(zone)
                        self.done_at[zone] = round(now-self.started, 2)
                        self.inside.pop(zone, None)
                        self._record()
                else:
                    self.inside.pop(zone, None)

    def tick(self):
        with self.lock:
            if self.state in ACTIVE:
                problems = list(self.ready())
                if self.pose_at is None or self.clock()-self.pose_at > 2:
                    problems.append('Robot pose lost')
                if self.state == 'countdown' and self.pose and (math.hypot(self.pose[0]-4.5,self.pose[1]-.5) > .35 or abs((self.pose[2]+2*math.pi) % (2*math.pi)-math.pi) > .52):
                    problems.append('Robot moved away from its start pose during countdown')
                if problems:
                    self.stop('; '.join(problems), error=True)
                elif self.state == 'countdown' and self.clock() >= self.deadline:
                    self.state, self.started = 'running', self.clock()
                    self.inside.clear()
                    self.message = 'Game running'
                    self._record()
                elif self.state == 'running' and (not self.remaining or self.clock()-self.started >= self.config['limit_s']):
                    self.stop('All targets counted' if not self.remaining else 'Time limit reached', finished=True)
            self._publish()

    def stop(self, reason='Stopped from the Game page', error=False, finished=False):
        with self.lock:
            was_active = self.state in ACTIVE
            if was_active:
                self.final_elapsed = max(0, self.clock()-self.started) if self.started is not None else 0
            self.state = 'error' if error else 'finished' if finished else 'stopped'
            self.message = reason
            try:
                if self.run_id:
                    self._record()
            finally:
                try:
                    self._publish()
                finally:
                    if was_active:
                        self.halt(reason)
            return self.snapshot()

    def _record(self):
        self._write(f'run-{self.run_id}.json', {
            'id': self.run_id, 'layout_id': self.layout_id, 'config': self.config,
            'state': self.state, 'reason': self.message, 'saved_at': self.wall(),
            'elapsed_s': round(self.final_elapsed if self.final_elapsed is not None else max(0, self.clock()-self.started) if self.started is not None else 0, 2),
            'counted': len(self.done_at), 'done_at': self.done_at, 'remaining': self.remaining})

    def _publish(self):
        running = self.state == 'running'
        # Send metadata first; RUNNING is last, so subscribers have field data before moving.
        messages = {'teamColor': 'CYAN', 'phase': 'PRODUCTION' if running else 'SETUP',
                    'private/machine_info': {'teamColor': 'CYAN', 'machines': self.config['machines']} if running else {},
                    'private/navigation_routes': {'teamColor': 'CYAN', 'routes': [{'id': 1, 'route': self.config['route'], 'remaining': list(self.remaining)}]} if running else {},
                    'game_state': 'RUNNING' if running else 'WAIT_START' if self.state in ('idle', 'prepared', 'countdown') else 'PAUSED'}
        for topic, payload in messages.items():
            self.publish(topic, json.dumps(payload) if isinstance(payload, dict) else payload)

    def snapshot(self):
        with self.lock:
            now = self.clock()
            elapsed = self.final_elapsed if self.final_elapsed is not None else max(0, now-self.started) if self.started is not None else 0
            return copy.deepcopy({'state': self.state, 'message': self.message, 'layout_id': self.layout_id,
                                  'config': self.config, 'run_id': self.run_id, 'remaining': self.remaining,
                                  'counted': len(self.done_at), 'elapsed_s': round(elapsed, 1),
                                  'countdown_left': max(0, math.ceil(self.deadline-now)) if self.state == 'countdown' else None,
                                  'time_left_s': max(0, round(self.config['limit_s']-elapsed, 1)) if self.config else None,
                                  'pose': self.pose, 'pose_age_s': round(now-self.pose_at, 2) if self.pose_at is not None else None})
