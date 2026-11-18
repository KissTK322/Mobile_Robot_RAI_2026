"""Game replays from the Record bags in ~/bags for the 3D Replay page.

The web server never reads a bag itself: converting runs as a low-priority child process
(`python3 -m ired_web.replay BAG OUT`) and writes one small JSON per bag into the cache folder.
"""
import json
import os
import re
import subprocess
import sys
import threading

NAME = re.compile(r'^[A-Za-z0-9_.-]+$')
MARKERS = (101, 102, 103, 104, 111, 112, 113, 114, 121, 122)


class ReplayStore:
    def __init__(self, bags_dir, cache_dir):
        self.bags = os.path.expanduser(bags_dir)
        self.cache = os.path.expanduser(cache_dir)
        self.jobs = {}           # bag name -> Popen while converting
        self.errors = {}         # bag name -> last conversion error
        self.lock = threading.Lock()

    def _bag(self, name):
        if not isinstance(name, str) or not NAME.match(name) or name.startswith('.'):
            raise ValueError('Bad bag name')
        path = os.path.join(self.bags, name)
        if not os.path.isfile(os.path.join(path, 'metadata.yaml')):
            raise ValueError('No such bag (still recording, or deleted?)')
        return path

    def _out(self, name):
        return os.path.join(self.cache, name + '.json')

    def _poll(self):
        for name, p in list(self.jobs.items()):
            if p.poll() is not None:
                del self.jobs[name]
                if p.returncode:
                    err = p.stderr.read().decode(errors='replace').strip().splitlines()
                    self.errors[name] = err[-1] if err else f'converter exited with {p.returncode}'
                p.stderr.close()

    def list(self):
        with self.lock:
            self._poll()
            out = []
            if os.path.isdir(self.bags):
                for name in sorted(os.listdir(self.bags), reverse=True):
                    path = os.path.join(self.bags, name)
                    if not NAME.match(name) or not os.path.isfile(os.path.join(path, 'metadata.yaml')):
                        continue
                    size = sum(e.stat().st_size for e in os.scandir(path) if e.is_file())
                    state = ('converting' if name in self.jobs else 'ready' if os.path.isfile(self._out(name))
                             else 'error' if name in self.errors else 'new')
                    out.append({'name': name, 'size_mb': round(size / 1e6, 1), 'state': state, 'error': self.errors.get(name, '')})
            return out

    def convert(self, name):
        path = self._bag(name)
        with self.lock:
            self._poll()
            if name in self.jobs:
                return
            if self.jobs:
                raise ValueError('Another bag is converting; wait for it')
            os.makedirs(self.cache, exist_ok=True)
            self.errors.pop(name, None)
            self.jobs[name] = subprocess.Popen(['nice', '-n', '15', sys.executable, '-m', 'ired_web.replay', path, self._out(name)],
                                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    def game(self, name):
        self._bag(name)
        try:
            with open(self._out(name), 'rb') as f:
                return f.read()
        except FileNotFoundError:
            raise ValueError('Not converted yet') from None


def _zone_xy(zone):
    return int(zone[-2]) - .5, int(zone[-1]) - .5


def build(bag):
    """One game dict (same shape as the PC replay page) from a Record bag."""
    import numpy as np
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import PoseWithCovarianceStamped
    import tf2_ros
    import rclpy.time
    import rclpy.duration

    r = SequentialReader()
    r.open(StorageOptions(uri=bag), ConverterOptions('', ''))
    topics = {t.name for t in r.get_all_topics_and_types()}
    mode = ('exploration' if '/challenge/exploration/state' in topics else
            'navigation' if '/challenge/navigation/state' in topics else None)
    tfbuf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3600))
    states, amcl, scans, seen = [], [], [], {}
    while r.has_next():
        topic, data, ts = r.read_next()
        ts /= 1e9
        if topic in ('/tf', '/tf_static'):
            for tr in deserialize_message(data, TFMessage).transforms:
                (tfbuf.set_transform if topic == '/tf' else tfbuf.set_transform_static)(tr, 'bag')
        elif mode and topic == f'/challenge/{mode}/state':
            states.append((ts, json.loads(deserialize_message(data, String).data)))
        elif topic == '/amcl_pose':
            p = deserialize_message(data, PoseWithCovarianceStamped).pose.pose
            amcl.append((ts, p.position.x, p.position.y, 2 * np.arctan2(p.orientation.z, p.orientation.w)))
        elif topic == '/aruco/detections':
            try:
                ids = {m['id'] for m in json.loads(deserialize_message(data, String).data).get('markers', []) if m.get('id') in MARKERS}
            except (ValueError, AttributeError):
                ids = set()
            if ids:
                seen.setdefault(round(ts * 4) / 4, set()).update(ids)
        elif topic == '/scan_filtered' and (not scans or ts - scans[-1][0] >= 0.5):
            m = deserialize_message(data, LaserScan)
            try:
                tr = tfbuf.lookup_transform('map', m.header.frame_id, rclpy.time.Time())
            except Exception:
                continue
            q = tr.transform.rotation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            rr = np.asarray(m.ranges, float)
            a = m.angle_min + np.arange(len(rr)) * m.angle_increment
            ok = np.isfinite(rr) & (rr > m.range_min) & (rr < min(m.range_max, 6.0))
            x = rr[ok] * np.cos(a[ok] + yaw) + tr.transform.translation.x
            y = rr[ok] * np.sin(a[ok] + yaw) + tr.transform.translation.y
            keep = (x > -0.4) & (x < 5.4) & (y > -0.4) & (y < 5.4)
            scans.append((ts, np.round(np.stack([x[keep], y[keep]], 1) * 100).astype(int)[::2].ravel().tolist()))

    poses = [(ts, *j['pose'][:3]) for ts, j in states if j.get('pose')] or amcl
    if not poses:
        raise RuntimeError('Bag has no robot pose (/challenge/*/state or /amcl_pose)')
    t0 = next((ts for ts, j in states if j.get('game_on')), poses[0][0])
    done = next((ts for ts, j in states if j.get('state') == 'DONE'), None)
    end = (done if done else max(poses[-1][0], scans[-1][0] if scans else 0)) - t0
    ev, boxes = [[0.0, 'state', 'Game start' if mode else 'Recording start']], []
    if mode == 'navigation':
        n = 0
        for ts, j in states:
            if len(j.get('visited', [])) > n:
                n = len(j['visited'])
                ev.append([round(ts - t0, 1), 'report', f'Reached {j["visited"][-1]} ({n}/12)'])
            if j.get('mps') and not boxes:
                boxes = [dict(name=m['name'], zone=m['zone'], rot=m['rotation']) for m in j['mps']]
        result = f'{n}/12 zones' + (f' in {end:.0f} s' if done else '')
    elif mode == 'exploration':
        rep, found, prev = {}, set(), None
        for ts, j in states:
            for e in j.get('estimates', []):
                if e['machine'] not in found:
                    found.add(e['machine'])
                    ev.append([round(ts - t0, 1), 'lidar', f'LiDAR finds a box in {e["zone"]} ({e["machine"]})'])
            if j.get('state') == 'LOOK' and prev != 'LOOK':
                ev.append([round(ts - t0, 1), 'look', 'Stop and look at the marker'])
            for m, v in j.get('reported', {}).items():
                if m not in rep:
                    rep[m] = v
                    ev.append([round(ts - t0, 1), 'report', f'Report {m}: {v["zone"]} {v["rotation"]}°'])
            prev = j.get('state')
        boxes = [dict(name=m, zone=v['zone'], rot=v['rotation']) for m, v in rep.items()]
        result = f'{len(rep)}/2 stations reported' + (f' in {end:.0f} s' if done else '')
    else:
        result = 'Recording without a challenge'
    if done:
        ev.append([round(end, 1), 'state', 'Done'])
    track, last = [], -9
    for p in poses:
        t = p[0] - t0
        if -3 <= t <= end + 3 and t - last >= 0.2:
            track.append([round(t, 2), round(p[1], 3), round(p[2], 3), round(float(p[3]), 3)])
            last = t
    name = os.path.basename(bag.rstrip('/'))
    return dict(id=name, title=(mode or 'recording').capitalize(), sub=name, end=round(end, 1), result=result,
                boxes=[b for b in boxes if re.match(r'^C_Z[1-5][1-5]$', b['zone'])], track=track, ev=ev,
                seen=[[round(k - t0, 2), sorted(v)] for k, v in sorted(seen.items()) if -3 <= k - t0 <= end + 3],
                lidar=[[round(ts - t0, 2), p] for ts, p in scans if -3 <= ts - t0 <= end + 3])


if __name__ == '__main__':
    game = build(sys.argv[1])
    tmp = sys.argv[2] + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(game, f, separators=(',', ':'))
    os.replace(tmp, sys.argv[2])
