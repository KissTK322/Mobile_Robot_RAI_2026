"""Race games of 2026-11-16 (two bags) -> race.json, put first in games.json by build.py.
source /opt/ros/jazzy/setup.bash; python3 race_data.py"""
import json
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros, rclpy.time, rclpy.duration
from scipy.spatial.transform import Rotation as R

BAGS = '/home/ired/ired_work/robot_runs/bags/'
MARKERS = (101, 102, 121, 122)


def read(bag, mode):
    """track/lidar/seen/states with t = seconds since the game started (game_on)"""
    r = SequentialReader(); r.open(StorageOptions(uri=BAGS + bag, storage_id='mcap'), ConverterOptions('', ''))
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=900))
    raw, states, seen, t0 = [], [], {}, None
    while r.has_next():
        topic, data, ts = r.read_next()
        ts /= 1e9
        if topic in ('/tf', '/tf_static'):
            for tr in deserialize_message(data, TFMessage).transforms:
                (buf.set_transform if topic == '/tf' else buf.set_transform_static)(tr, 'bag')
        elif topic == f'/challenge/{mode}/state':
            j = json.loads(deserialize_message(data, String).data)
            if j.get('game_on') and t0 is None:
                t0 = ts
            states.append((ts, j))
        elif topic == '/aruco/detections':
            ids = {m['id'] for m in json.loads(deserialize_message(data, String).data).get('markers', []) if m['id'] in MARKERS}
            if ids:
                seen.setdefault(round(ts * 4) / 4, set()).update(ids)
        elif topic == '/scan_filtered' and (not raw or ts - raw[-1][0] >= 0.5):
            m = deserialize_message(data, LaserScan)
            try:
                tr = buf.lookup_transform('map', m.header.frame_id, rclpy.time.Time())
            except Exception:
                continue
            q = tr.transform.rotation
            yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
            rr = np.asarray(m.ranges, float)
            a = m.angle_min + np.arange(len(rr)) * m.angle_increment
            ok = np.isfinite(rr) & (rr > m.range_min) & (rr < min(m.range_max, 6.0))
            x = rr[ok] * np.cos(a[ok] + yaw) + tr.transform.translation.x
            y = rr[ok] * np.sin(a[ok] + yaw) + tr.transform.translation.y
            keep = (x > -0.4) & (x < 5.4) & (y > -0.4) & (y < 5.4)
            raw.append((ts, np.round(np.stack([x[keep], y[keep]], 1) * 100).astype(int)[::2].ravel().tolist()))
    done = next(ts for ts, j in states if j['state'] == 'DONE') - t0
    track, last = [], -9
    for ts, j in states:
        t = ts - t0
        if -3 <= t <= done + 3 and t - last >= 0.2:
            track.append([round(t, 2), *[round(v, 3) for v in j['pose'][:3]]]); last = t
    lidar = [[round(ts - t0, 2), p] for ts, p in raw if -3 <= ts - t0 <= done + 3]
    seen = [[round(k - t0, 2), sorted(v)] for k, v in sorted(seen.items()) if -3 <= k - t0 <= done + 3]
    return [(ts - t0, j) for ts, j in states if ts >= t0], track, lidar, seen, done


def navigation():
    states, track, lidar, seen, done = read('nav-20261116-095858', 'navigation')
    final = states[-1][1]
    ev, n = [[0, 'state', 'เริ่มเกม ออกจาก START ไปช่องแดง C_Z32']], 0
    for t, j in states:
        if len(j['visited']) > n:
            n = len(j['visited'])
            ev.append([round(t, 1), 'report', f'ถึงช่อง {j["visited"][-1]} ({n}/12)'])
    ev.append([round(done, 1), 'state', f'ครบ 12 ช่อง ใน {done:.0f} วินาที ไม่มี retry'])
    return dict(id='race_nav', title='แข่งจริง: Navigation', sub='16 พ.ย. 2026 โหมด test fast', end=round(done, 1),
                result='12/12 ช่อง 124 วินาที', boxes=[dict(name=m['name'], zone=m['zone'], rot=m['rotation']) for m in final['mps']],
                track=track, ev=ev, seen=[], lidar=lidar)


def exploration():
    states, track, lidar, seen, done = read('nav-20261116-100745', 'exploration')
    ev, prev, rep, est = [[0, 'state', 'เริ่มเกม ออกจาก START ไปช่องแดง C_Z32']], None, {}, set()
    for t, j in states:
        for e in j.get('estimates', []):
            if e['machine'] not in est:
                est.add(e['machine'])
                ev.append([round(t, 1), 'lidar', f'LiDAR เห็นกล่องใน {e["zone"]} กล้องอ่านได้ว่าเป็น {e["machine"]}'])
        if j['state'] == 'LOOK' and prev != 'LOOK':
            ev.append([round(t, 1), 'look', 'หยุดหันกล้องดู marker'])
        for m, r in j['reported'].items():
            if m not in rep:
                rep[m] = r
                ev.append([round(t, 1), 'report', f'รายงาน {m}: {r["zone"]} มุม {r["rotation"]}'])
        prev = j['state']
    ev.append([round(done, 1), 'state', f'รายงานครบ 2 กล่อง ใน {done:.0f} วินาที'])
    return dict(id='race_exp', title='แข่งจริง: Exploration', sub='16 พ.ย. 2026 โหมด LiDAR', end=round(done, 1),
                result='รายงาน 2/2 กล่อง 57 วินาที', boxes=[dict(name=m, zone=r['zone'], rot=r['rotation']) for m, r in rep.items()],
                track=track, ev=ev, seen=seen, lidar=lidar)


games = [navigation(), exploration()]
json.dump(games, open('race.json', 'w'), ensure_ascii=False, separators=(',', ':'))
for g in games:
    print(g['id'], 'track', len(g['track']), 'end', g['end'], 'seen', len(g['seen']), 'lidar', len(g['lidar']), 'ev', len(g['ev']))
    for e in g['ev']: print('  ', e)
