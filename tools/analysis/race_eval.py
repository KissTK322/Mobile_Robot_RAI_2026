"""Error numbers of the two race games (16 Nov) for the Evaluate slide -> race_eval.json
odometry drift (map->odom corrections from AMCL), wheel speed tracking, zone-centre error, time split.
source /opt/ros/jazzy/setup.bash; source ~/ired_ws/install/setup.bash; python3 race_eval.py"""
import json, math
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAGS = '/home/ired/ired_work/robot_runs/bags/'
GAMES = {'navigation': 'nav-20261116-095858', 'exploration': 'nav-20261116-100745'}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(bag, mode):
    r = SequentialReader(); r.open(StorageOptions(uri=BAGS + bag, storage_id='mcap'), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = {'/tf', '/odometry/filtered', '/ired/motor/data', f'/challenge/{mode}/state'}
    m2o, odom, motor, states = [], [], [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic not in want:
            continue
        t /= 1e9
        m = deserialize_message(data, get_message(types[topic]))
        if topic == '/tf':
            for tr in m.transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    p = tr.transform.translation
                    m2o.append((t, p.x, p.y, yaw(tr.transform.rotation)))
        elif topic == '/odometry/filtered':
            p = m.pose.pose.position
            odom.append((t, p.x, p.y, yaw(m.pose.pose.orientation), math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y),
                         m.twist.twist.angular.z))
        elif topic == '/ired/motor/data':
            motor.append((t, list(m.speed_sp), list(m.speed_fb)))
        else:
            states.append((t, json.loads(m.data)))
    return np.array(m2o), np.array(odom), motor, states


def game_window(states):
    t0 = next(t for t, j in states if j.get('game_on'))
    t1 = next(t for t, j in states if j.get('state') == 'DONE')
    return t0, t1


def drift(m2o, odom, t0, t1):
    """Odometry error seen at the robot: AMCL shifts map->odom by the error it found. Robot point p (odom frame)
    jumps by T_new(p) - T_old(p) at each correction; T_start(p) vs T(p) = where odometry alone would put the robot."""
    def apply(T, x, y):
        c, s = math.cos(T[3]), math.sin(T[3])
        return np.array([T[1] + c * x - s * y, T[2] + s * x + c * y])
    o = odom[(odom[:, 0] >= t0) & (odom[:, 0] <= t1)]
    m = m2o[(m2o[:, 0] >= t0) & (m2o[:, 0] <= t1)]
    pos = lambda t: o[min(np.searchsorted(o[:, 0], t), len(o) - 1)]
    jumps, prev = [], m[0]
    for T in m[1:]:
        if abs(T[1] - prev[1]) + abs(T[2] - prev[2]) + abs(T[3] - prev[3]) < 1e-5:
            continue
        p = pos(T[0])
        jumps.append(float(np.hypot(*(apply(T, p[1], p[2]) - apply(prev, p[1], p[2])))))
        prev = T
    jumps = np.array(jumps)
    alone = [float(np.hypot(*(apply(T, *pos(T[0])[1:3]) - apply(m[0], *pos(T[0])[1:3])))) for T in m[::10]]
    cum = np.concatenate([[0], np.cumsum(np.hypot(np.diff(o[:, 1]), np.diff(o[:, 2])))])
    series = [[round(float(T[0] - t0), 1), round(float(cum[min(np.searchsorted(o[:, 0], T[0]), len(o) - 1)]), 2), round(e * 100, 1)]
              for T, e in zip(m[::10], alone)]
    dist = float(np.sum(np.hypot(np.diff(o[:, 1]), np.diff(o[:, 2]))))
    turn = float(np.degrees(np.sum(np.abs(np.angle(np.exp(1j * np.diff(o[:, 3])))))))
    dyaw = float(np.degrees(np.angle(np.exp(1j * (m[-1, 3] - m[0, 3])))))
    return dict(distance_m=round(dist, 2), turned_deg=round(turn), amcl_corrections=len(jumps),
                jump_median_cm=round(float(np.median(jumps)) * 100, 1), jump_p95_cm=round(float(np.percentile(jumps, 95)) * 100, 1),
                jump_max_cm=round(float(jumps.max()) * 100, 1), odom_only_end_error_cm=round(alone[-1] * 100, 1),
                odom_only_max_error_cm=round(max(alone) * 100, 1), odom_only_error_pct=round(alone[-1] / dist * 100, 1),
                odom_yaw_error_deg=round(dyaw, 1), series_t_dist_errcm=series)


def tracking(motor, t0, t1):
    sp = np.array([s for t, s, f in motor if t0 <= t <= t1]); fb = np.array([f for t, s, f in motor if t0 <= t <= t1])
    out = {}
    for i, w in enumerate(('FL', 'FR', 'RL', 'RR')):
        on = np.abs(sp[:, i]) > 5                     # rpm; ignore stand-still
        err = np.abs(fb[on, i] - sp[on, i]) / np.abs(sp[on, i])
        out[w] = round(float(np.median(err)) * 100, 1)
    allerr = np.concatenate([np.abs(fb[np.abs(sp[:, i]) > 5, i] - sp[np.abs(sp[:, i]) > 5, i]) / np.abs(sp[np.abs(sp[:, i]) > 5, i]) for i in range(4)])
    return dict(median_pct=out, all_median_pct=round(float(np.median(allerr)) * 100, 1), all_p90_pct=round(float(np.percentile(allerr, 90)) * 100, 1),
                rate_hz=round(len(sp) / (t1 - t0), 1))


def split(states, t0, t1):
    tot = {}
    for (ta, a), (tb, _) in zip(states, states[1:]):
        if t0 <= ta < t1:
            tot[a['state']] = tot.get(a['state'], 0) + min(tb, t1) - ta
    return {k: round(v, 1) for k, v in sorted(tot.items(), key=lambda kv: -kv[1])}


def zone_error(states, odom, m2o):
    """navigation: mean robot pose during each 5 s stay vs the zone centre (map frame from the challenge state)"""
    res, cur = [], None
    for t, j in states:
        if j['state'] == 'DWELL':
            cur = cur or {'zone': j['target'], 'pts': []}
            cur['pts'].append(j['pose'][:2])
        elif cur:
            p = np.mean(cur['pts'], 0)
            cx, cy = int(cur['zone'][-2]) - .5, int(cur['zone'][-1]) - .5
            res.append((cur['zone'], round(float(np.hypot(p[0] - cx, p[1] - cy)) * 100, 1)))
            cur = None
    e = [x[1] for x in res]
    return dict(per_zone_cm=dict(res), mean_cm=round(float(np.mean(e)), 1), max_cm=round(float(np.max(e)), 1)) if e else {}


def evaluate(mode, bag, strict=False):
    m2o, odom, motor, states = read(bag, mode)
    if strict and not states:
        raise ValueError('no challenge state')
    try:
        t0, t1 = game_window(states)
    except StopIteration:                 # older bags: no challenge state topic, or game not finished
        moving = odom[odom[:, 4] > 0.03, 0]
        t0, t1 = float(moving[0]), float(moving[-1])
        if len(m2o) == 0 or t1 - t0 < 30:
            raise
    o = odom[(odom[:, 0] >= t0) & (odom[:, 0] <= t1)]
    g = dict(bag=bag, game_s=round(t1 - t0, 1), max_speed_mps=round(float(o[:, 4].max()), 2),
             mean_speed_moving_mps=round(float(o[o[:, 4] > 0.03, 4].mean()), 2), max_turn_dps=round(float(np.degrees(np.abs(o[:, 5]).max()))),
             odometry=drift(m2o, odom, t0, t1), wheels=tracking(motor, t0, t1), time_split_s=split(states, t0, t1), has_state=bool(states))
    if mode == 'navigation' and states:
        g['zone_centre'] = zone_error(states, odom, m2o)
    return g


if __name__ == '__main__':
    out = {mode: evaluate(mode, bag) for mode, bag in GAMES.items()}
    json.dump(out, open('race_eval.json', 'w'), indent=1)
    print(json.dumps(out, indent=1))
