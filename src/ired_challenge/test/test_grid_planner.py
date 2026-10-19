"""Offline tests for the zone-grid route planner (micromouse style)."""
import itertools
import math
import random

import numpy as np
import pytest

from ired_challenge.field import Field
from ired_challenge.grid_planner import GridPlanner, seg_point_dist

F = Field()


def wall_points(walls, step=0.05):
    pts = []
    for (x0, y0), (x1, y1) in walls:
        n = max(1, int(math.dist((x0, y0), (x1, y1)) / step))
        pts += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]
    return np.array(pts)


# the PC-sim field: 5 x 5 m walls plus the START fence seen in the 2025 Gazebo world
BORDER = [((0, 0), (5, 0)), ((5, 0), (5, 5)), ((5, 5), (0, 5)), ((0, 5), (0, 0))]
FENCE = [((2, 0), (2, 1)), ((2.9, 1), (5, 1))]
# layout 4 of the sim: BS next to the fence end, where Nav2 got stuck for 139 s
LAYOUT4 = [('C_Z25', 180), ('C_Z22', 45), ('C_Z43', 0), ('C_Z45', 180)]
ROUTE4 = ['C_Z12', 'C_Z23', 'C_Z44', 'C_Z45', 'C_Z33', 'C_Z24', 'C_Z21', 'C_Z35', 'C_Z55', 'C_Z15', 'C_Z14', 'C_Z11']
START = (4.5, 0.5)


def planner(boxes=(), fence=True):
    g = GridPlanner(F)
    g.set_static(wall_points(BORDER + (FENCE if fence else [])))
    g.set_boxes(boxes)
    return g


def in_start(p):
    return 2.0 < p[0] < 5.0 and p[1] < 1.0


def assert_clear(g, pts, own_zone=None):
    """Clear of walls by `clear`; boxes too, except the box of the zone the leg starts
    in or ends in, which may be passed at the robot radius (stop points sit beside it)."""
    walls = wall_points(BORDER + FENCE)
    own = {own_zone, F.zone_at(*pts[0])}
    for a, b in zip(pts, pts[1:]):
        assert seg_point_dist(a, b, walls) >= g.clear - 1e-6, (a, b)
        for z, fp in g.boxes.items():
            lim = g.own_clear if z in own else g.clear
            assert seg_point_dist(a, b, fp) >= lim - 1e-6, (a, b, z)


def test_exit_from_start_goes_through_the_gap_into_the_red_zone():
    g = planner()
    pts = g.path(START, 'C_Z32')
    assert pts[0] == START and F.inside('C_Z32', *pts[-1])
    assert_clear(g, pts)
    # leaves START only through the gap (x 2..2.9) and never crosses the fence
    for a, b in zip(pts, pts[1:]):
        if a[1] < 1.0 <= b[1]:
            x = a[0] + (b[0] - a[0]) * (1.0 - a[1]) / (b[1] - a[1])
            assert 2.0 + g.clear - 0.05 <= x <= 2.9 - g.clear + 0.05


def test_no_link_squeezes_past_the_fence_end_or_a_box():
    g = planner(LAYOUT4)
    assert 'C_Z32' not in g.links['C_Z21']          # diagonal over the fence end
    assert 'C_Z31' not in g.links['C_Z21']          # through the fence
    for a, nbrs in g.links.items():
        for b in nbrs:
            own = tuple(z for z in (a, b) if z in g.boxes)
            assert g._clear_seg(g.node[a], g.node[b], own=own)


def test_layout4_never_goes_back_into_start_and_reaches_every_zone():
    g = planner(LAYOUT4)
    order = g.order(F.center('C_Z32'), ROUTE4)
    assert sorted(order) == sorted(ROUTE4)
    cur = F.center('C_Z32')
    for z in order:
        pts = g.path(cur, z)
        assert pts is not None, z
        assert not any(in_start(p) for p in pts[1:]), (z, pts)
        assert_clear(g, pts, own_zone=z if z in g.boxes else None)
        cur = pts[-1]
    # C_Z21 sits behind the fence and BS: it should not be the first stop
    assert order[0] != 'C_Z21'


def test_box_zone_stop_point_is_inside_and_beside_the_box():
    g = planner(LAYOUT4)
    p = g.node['C_Z45']
    assert F.inside('C_Z45', *p, margin=0.15)
    assert seg_point_dist(p, p, g.boxes['C_Z45']) >= g.own_clear


def test_held_karp_matches_brute_force():
    g = planner(LAYOUT4)
    rng = random.Random(3)
    free = [z for z in g.zones if not in_start(F.center(z))]
    for _ in range(4):
        zones = rng.sample(free, 5)
        start = F.center('C_Z32')
        best = min(g.tour_time(start, p) for p in itertools.permutations(zones))
        assert g.tour_time(start, g.order(start, zones)) == pytest.approx(best, rel=1e-6)


def test_order_is_much_faster_than_the_given_order():
    g = planner(LAYOUT4)
    s = F.center('C_Z32')
    assert g.tour_time(s, g.order(s, ROUTE4)) < 0.8 * g.tour_time(s, ROUTE4)


def test_unreachable_zone_goes_last_instead_of_breaking_the_plan():
    g = planner(LAYOUT4)
    g.set_static(np.vstack([g.static, wall_points([((0, 1), (2, 1)), ((1, 0), (1, 1))])]))   # wall C_Z11 off
    order = g.order(F.center('C_Z32'), ['C_Z11', 'C_Z33', 'C_Z44'])
    assert order[-1] == 'C_Z11' and g.path(F.center('C_Z32'), 'C_Z11') is None


def test_corner_cutting_keeps_the_clearance():
    g = planner()
    pts = g.path(START, 'C_Z55')
    assert len(pts) <= 5                               # shortcut, not every zone centre
    assert_clear(g, pts)


def test_routes_never_pass_through_a_box_zone_on_the_way():
    # sim layout 2: the route to C_Z54 went through the stop point beside BS in C_Z53,
    # 0.22 m from the box; DWB refused it and the zone cost 3 tries.
    g = planner([('C_Z53', 90), ('C_Z45', 180), ('C_Z23', 135), ('C_Z25', 180)])
    for start in ('C_Z52', 'C_Z42', 'C_Z33'):
        for goal in g.zones:
            if goal == start or in_start(F.center(goal)):
                continue
            pts = g.path(g.node[start], goal)
            if pts is None:
                continue
            for p in pts[1:-1]:
                z = F.zone_at(*p)
                assert z not in g.boxes or p != g.node[z], (start, goal, pts)


# ------------------------------------------------------------ lanes style (route_mode lanes)
def lanes(boxes=()):
    g = GridPlanner(F, style='lanes', turn_s=1.2)
    g.set_static(wall_points(BORDER + FENCE))
    g.set_boxes(boxes)
    return g


def test_lanes_go_straight_and_only_cut_corners_in_open_space():
    g = lanes(LAYOUT4)
    cur = F.center('C_Z32')
    for z in g.order(cur, ROUTE4):
        pts = g.path(cur, z)
        assert pts is not None, z
        for a, b in zip(pts, pts[1:]):
            axis = abs(a[0] - b[0]) < 1e-6 or abs(a[1] - b[1]) < 1e-6
            into_box = b == pts[-1] and z in g.boxes          # the last bit to the stop point beside a box
            own = {z, F.zone_at(*a)}
            assert axis or into_box or g._margin(a, b, own=own) >= g.diag_clear - 1e-6, (z, a, b)
        assert not any(in_start(p) for p in pts[1:]), (z, pts)
        assert_clear(g, pts, own_zone=z if z in g.boxes else None)
        cur = pts[-1]


def test_lanes_order_is_exact_with_the_risk_costs():
    g = lanes(LAYOUT4)
    rng = random.Random(5)
    free = [z for z in g.zones if not in_start(F.center(z))]
    for _ in range(3):
        zones = rng.sample(free, 5)
        s = F.center('C_Z32')
        best = min(g.tour_time(s, p) for p in itertools.permutations(zones))
        assert g.tour_time(s, g.order(s, zones)) == pytest.approx(best, rel=1e-6)


def test_lanes_avoid_a_narrow_gap_when_a_wider_way_exists():
    # BS in C_Z53 against the right wall (sim layout 2, 7 retries): from C_Z52 to C_Z54 the
    # lanes route must go round through C_Z43 / C_Z44, not past the box along the wall
    g = lanes([('C_Z53', 90), ('C_Z45', 180), ('C_Z23', 135), ('C_Z25', 180)])
    pts = g.path(F.center('C_Z52'), 'C_Z54')
    assert pts is not None
    assert all(p[0] <= 4.0 or F.zone_at(*p) in ('C_Z52', 'C_Z54') for p in pts), pts
