"""LiDAR box search: box clusters found in the right zone, walls ignored; exploration looks there first."""
import math

import numpy as np

from ired_challenge.field import Field
from ired_challenge.lidar_boxes import BoxFinder

F = Field()


def wall_cells():
    pts = []
    for i in range(0, 501, 2):
        t = i / 100
        pts += [(t, 0.0), (t, 5.0), (0.0, t), (5.0, t)]
    return pts


def box_scan(robot, box, n=360):
    """Ray-cast a 20 cm box (axis aligned) and the field walls from `robot`."""
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        d = np.array([math.cos(a), math.sin(a)])
        best = 10.0
        lo, hi = np.array(box) - 0.1, np.array(box) + 0.1
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (lo - robot) / d, (hi - robot) / d
        tmin, tmax = np.nanmax(np.minimum(t1, t2)), np.nanmin(np.maximum(t1, t2))
        if tmax >= max(tmin, 0):
            best = max(tmin, 0)
        for axis, val in ((0, 0.0), (0, 5.0), (1, 0.0), (1, 5.0)):
            if abs(d[axis]) > 1e-9:
                t = (val - robot[axis]) / d[axis]
                if 0 < t < best:
                    best = t
        out.append(np.array(robot) + d * best)
    return np.array(out)


def test_finds_the_box_zone_and_ignores_walls():
    f = BoxFinder(F)
    f.set_walls(wall_cells())
    robot = np.array([4.5, 0.5])
    for _ in range(6):
        f.add(box_scan(robot, (2.5, 3.5)), robot)
    assert f.zones() == {'C_Z34'}


def test_needs_several_scans():
    f = BoxFinder(F, min_scans=5)
    f.set_walls(wall_cells())
    robot = np.array([4.5, 0.5])
    f.add(box_scan(robot, (1.5, 2.5)), robot)
    assert f.zones() == set()


def test_exploration_looks_at_lidar_boxes_first():
    from ired_challenge.exploration_task import ExplorationTask

    class Nav:
        def __init__(self): self.sent = []
        def go_via(self, pts, yaw): self.sent.append(pts[-1])
        def state(self): return 'active'
        def cancel(self): pass

    class Est:
        min_sightings = 10
        def all(self): return []

    nav = Nav()
    t = ExplorationTask(nav, F, Est(), lambda *a: True, log=lambda *_: None,
                        candidate=lambda z: z not in ('C_Z31', 'C_Z41', 'C_Z51', 'C_Z32'),
                        box_zones=lambda: {'C_Z13'})
    t.red_done = True
    t.queue = [(4.0, 4.0), (3.0, 4.0)]
    t._next((2.5, 1.5, 0.0), 0.0)
    assert nav.sent[-1] in [(1.0, 3.0), (1.0, 2.0)]          # a corner of C_Z13 first
    assert t.focus == {'C_Z13'}


def test_cluster_away_from_a_zone_centre_is_not_a_box():
    """Field 2026-10-28: leftovers of the top wall and the START fence looked like box-sized clusters."""
    b = BoxFinder(F)
    fence = [(3.30 + i * 0.03, 1.02) for i in range(8)]             # near C_Z42's lower edge
    for _ in range(8):
        b.add(fence, (3.0, 2.5))
    assert b.zones() == set()
    box = [(3.42 + i * 0.03, 3.40) for i in range(6)]               # front face of a box in C_Z44
    for _ in range(8):
        b.add(box, (3.5, 2.5))
    assert b.zones() == {'C_Z44'}
