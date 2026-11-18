"""Route figures of the 2026-11-16 race games for the slides.
source /opt/ros/jazzy/setup.bash; python3 race_plots.py"""
import json, os
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

BAGS = os.path.expanduser('~/ired_work/robot_runs/bags')
OUT = os.path.expanduser('~/ired_work/docs/final_presentation')
BG, FG, GRID, ACC, RED = '#0a0a0a', '#ffffff', '#3a3a3a', '#f2b33d', '#e5484d'


def states(bag, topic):
    r = SequentialReader(); r.open(StorageOptions(uri=f'{BAGS}/{bag}'), ConverterOptions('', ''))
    r.set_filter(StorageFilter(topics=[topic]))
    while r.has_next():
        _, d, ts = r.read_next()
        yield ts / 1e9, json.loads(deserialize_message(d, String).data)


def centre(z):
    return int(z[-2]) - 0.5, int(z[-1]) - 0.5


def field(ax):
    ax.set_facecolor(BG)
    for k in range(6):
        ax.plot([k, k], [0, 5], color=GRID, lw=1); ax.plot([0, 5], [k, k], color=GRID, lw=1)
    for i in range(1, 6):
        for j in range(1, 6):
            ax.text(i - .95, j - .1, f'Z{i}{j}', color='#777', fontsize=7, va='top')
    ax.add_patch(Rectangle((2, 0), 3, 1, fc='#1d1d1d', ec=FG, lw=1.5))
    ax.text(3.5, .12, 'START', color=FG, ha='center', fontsize=9, weight='bold')
    ax.add_patch(Rectangle((2, 1), 1, 1, fc=RED, alpha=.25, ec=RED))
    ax.set_xlim(-.05, 5.05); ax.set_ylim(-.05, 5.05); ax.set_aspect('equal'); ax.axis('off')


def box(ax, x, y, rot, label, col):
    ax.add_patch(Rectangle((x - .35, y - .25), .7, .5, angle=rot, rotation_point='center', fc=col, alpha=.85, ec=FG))
    ax.text(x, y, label, color=BG, ha='center', va='center', fontsize=8, weight='bold')


def route(ax, pts):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=ACC, lw=2.2)
    ax.plot(*pts[0], 'o', color=FG, ms=7); ax.plot(*pts[-1], 's', color=ACC, ms=8)


def save(fig, name):
    fig.savefig(f'{OUT}/{name}', dpi=200, facecolor=BG, bbox_inches='tight'); plt.close(fig)


def navigation():
    pts, j, order, t_on, t_done = [], None, [], None, None
    for ts, j in states('nav-20261116-095858', '/challenge/navigation/state'):
        if j['game_on'] and t_on is None: t_on = ts
        if t_on and j['state'] != 'DONE': pts.append(tuple(j['pose'][:2]))
        if j['state'] == 'DONE' and t_done is None: t_done = ts; final = j
    fig, ax = plt.subplots(figsize=(6, 6.4), facecolor=BG); field(ax)
    for (x, y, rot), m in zip(final['mps_boxes'], final['mps']):
        box(ax, x, y, rot, m['name'].replace('C-', ''), '#9aa0a6')
    route(ax, pts)
    for n, z in enumerate(final['visited']):
        x, y = centre(z)
        ax.add_patch(plt.Circle((x, y), .13, color=ACC)); ax.text(x, y, str(n + 1), color=BG, ha='center', va='center', fontsize=8, weight='bold')
    ax.set_title(f'Navigation (race)  {len(final["visited"])}/12 zones  {t_done - t_on:.0f} s  0 retries',
                 color=FG, fontsize=13, weight='bold')
    save(fig, 'race_navigation_route.png')
    return t_done - t_on, final


def exploration():
    pts, t_on, t_done, final = [], None, None, None
    for ts, j in states('nav-20261116-100745', '/challenge/exploration/state'):
        if j['game_on'] and t_on is None: t_on = ts
        if t_on and t_done is None: pts.append(tuple(j['pose'][:2]))
        if j['state'] == 'DONE' and t_done is None: t_done = ts; final = j
    fig, ax = plt.subplots(figsize=(6, 6.4), facecolor=BG); field(ax)
    for m, r in final['reported'].items():
        x, y = centre(r['zone']); box(ax, x, y, r['rotation'], m.replace('C-', ''), ACC)
        ax.text(x, y + .38, f"{r['zone'].replace('C_', '')} {r['rotation']}°", color=FG, ha='center', fontsize=8)
    route(ax, pts)
    ax.set_title(f'Exploration (race)  2/2 reported  {t_done - t_on:.0f} s', color=FG, fontsize=13, weight='bold')
    save(fig, 'race_exploration_route.png')
    return t_done - t_on, final


if __name__ == '__main__':
    print('nav', navigation()[0]); t, f = exploration(); print('exp', t, f['reported'])
