"""Offline tests for the exploration geometry (no ROS)."""
import math
import random

from ired_challenge.exploration import (MachineEstimator, machine_from_marker, marker_in_map,
                                        refbox_rotation)
from ired_challenge.field import Field

F = Field()


def sighting(robot, box_xy, input_dir, side, noise=0.0, rng=None):
    """What the ArUco node would report for a box seen from `robot`."""
    face = input_dir if side == 'input' else input_dir + math.pi
    mx, my = box_xy[0] + 0.10 * math.cos(face), box_xy[1] + 0.10 * math.sin(face)
    rx, ry, ryaw = robot
    dx, dy = mx - rx, my - ry
    bx = math.cos(-ryaw) * dx - math.sin(-ryaw) * dy
    by = math.sin(-ryaw) * dx + math.cos(-ryaw) * dy
    if rng:
        bx += rng.gauss(0, noise)
        by += rng.gauss(0, noise)
    return (bx, by, 0.3), math.degrees(face - ryaw)


def test_marker_straight_ahead_facing_robot():
    m = marker_in_map((1.0, 1.0, 0.0), (0.8, 0.0, 0.3), 180.0)
    assert math.isclose(m[0], 1.8) and math.isclose(m[1], 1.0) and math.isclose(abs(m[2]), math.pi)
    cx, cy, d = machine_from_marker(m, 'input')
    assert math.isclose(cx, 1.9) and math.isclose(cy, 1.0)    # box is behind the marker
    assert math.isclose(abs(d), math.pi)                       # input side faces the robot


def test_output_marker_flips_direction():
    m = (2.0, 2.0, 0.0)
    assert math.isclose(machine_from_marker(m, 'input')[2], 0.0)
    assert math.isclose(abs(machine_from_marker(m, 'output')[2]), math.pi)


def test_refbox_rotation_snaps_to_45():
    assert refbox_rotation(0.0) == 270
    assert refbox_rotation(math.radians(90)) == 180
    assert refbox_rotation(math.radians(-90)) == 0
    assert refbox_rotation(math.radians(47)) == 225      # 223 -> 225


def test_estimator_finds_zone_and_rotation_from_noisy_views():
    rng = random.Random(1)
    est = MachineEstimator(F)
    box, input_dir = (3.5, 2.5), math.radians(90)           # C-CS1 in M_Z43, input facing +y
    views = [((3.0, 1.0, math.radians(70)), 'output'), ((2.0, 3.5, math.radians(-30)), 'input'),
             ((4.4, 3.8, math.radians(-110)), 'input'), ((3.4, 1.2, math.radians(95)), 'output')]
    for robot, side in views:
        pos, yaw = sighting(robot, box, input_dir, side, noise=0.03, rng=rng)
        assert est.add(robot, 102 if side == 'input' else 101, pos, yaw + rng.gauss(0, 4)) == 'C-CS1'
    e = est.estimate('C-CS1')
    assert e['zone'] == 'M_Z43'
    assert e['rotation'] == refbox_rotation(input_dir)
    assert math.hypot(e['x'] - box[0], e['y'] - box[1]) < 0.08
    assert e['ready']


def test_not_ready_until_enough_agreeing_sightings():
    est = MachineEstimator(F)
    pos, yaw = sighting((1.0, 1.0, 0.0), (2.5, 1.5), 0.0, 'input')
    est.add((1.0, 1.0, 0.0), 122, pos, yaw)
    assert not est.estimate('C-BS')['ready']


def test_ignores_unknown_ids_far_markers_and_missing_pose():
    est = MachineEstimator(F)
    assert est.add((0, 0, 0), 999, (1.0, 0.0, 0.3), 180) is None
    assert est.add((0, 0, 0), 102, (3.0, 0.0, 0.3), 180) is None     # beyond 2 m
    assert est.add((0, 0, 0), 102, None, None) is None
    assert est.all() == []
