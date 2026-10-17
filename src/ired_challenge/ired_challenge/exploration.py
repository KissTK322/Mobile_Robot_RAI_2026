"""Exploration challenge geometry (no ROS): ArUco sightings -> MPS zone and rotation.

Each MPS station is a 20 x 20 cm box with an ArUco marker on its input side and
another on its output side. The ArUco node reports, per marker, its position in
base_link and `marker_yaw_deg`: the direction the marker face points (out of
the box) in base_link. With the robot pose that gives the marker in the map,
the box centre is 10 cm behind the marker, and the input side direction is the
face direction (input marker) or its opposite (output marker).

The refbox rotation convention is not documented here. The default follows
ired_refbox, which reports the robot heading as (270 - yaw_deg) % 360; check it
on the field against /refbox/machine_info before trusting the rotation score."""
import math
from collections import defaultdict

# marker id -> (machine name, side). From the challenge slides; add the other
# machines' ids here when the refbox setup is known.
DEFAULT_MARKERS = {
    101: ('C-CS1', 'output'), 102: ('C-CS1', 'input'),
    121: ('C-BS', 'output'), 122: ('C-BS', 'input'),
}


def marker_in_map(robot_pose, pos_base, marker_yaw_deg):
    """(x, y, face_dir_rad) of a marker in the map frame."""
    rx, ry, ryaw = robot_pose
    c, s = math.cos(ryaw), math.sin(ryaw)
    mx = rx + c * pos_base[0] - s * pos_base[1]
    my = ry + s * pos_base[0] + c * pos_base[1]
    face = ryaw + math.radians(marker_yaw_deg)
    return mx, my, math.atan2(math.sin(face), math.cos(face))


def machine_from_marker(marker_xy_face, side, half_depth=0.10):
    """Box centre and input-side direction (rad) from one marker in the map."""
    mx, my, face = marker_xy_face
    cx, cy = mx - half_depth * math.cos(face), my - half_depth * math.sin(face)
    input_dir = face if side == 'input' else face + math.pi
    return cx, cy, math.atan2(math.sin(input_dir), math.cos(input_dir))


def refbox_rotation(input_dir_rad, zero_deg=270.0, sign=-1.0, step=45):
    """Map direction -> refbox rotation degrees, snapped to `step`.

    Default: rotation = (270 - deg) % 360, the convention ired_refbox uses for the
    robot heading. Change zero_deg / sign if the field check shows otherwise."""
    deg = (zero_deg + sign * math.degrees(input_dir_rad)) % 360
    return int(round(deg / step) * step) % 360


def circular_mean(angles):
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


class MachineEstimator:
    """Collects sightings and decides when a machine is known well enough to report.

    A wrong report costs 2 points, so only report after `min_sightings` that agree
    within `max_spread` metres."""

    def __init__(self, field, markers=None, min_sightings=3, max_spread=0.15, max_range=2.0):
        self.field = field
        self.markers = dict(markers or DEFAULT_MARKERS)
        self.min_sightings, self.max_spread, self.max_range = min_sightings, max_spread, max_range
        self.sightings = defaultdict(list)       # machine -> [(cx, cy, input_dir)]

    def add(self, robot_pose, marker_id, pos_base, marker_yaw_deg):
        """Add one detection. Returns the machine name or None if ignored."""
        if marker_id not in self.markers or pos_base is None or marker_yaw_deg is None:
            return None
        if math.hypot(pos_base[0], pos_base[1]) > self.max_range:
            return None                          # far detections are too noisy
        name, side = self.markers[marker_id]
        m = marker_in_map(robot_pose, pos_base, marker_yaw_deg)
        self.sightings[name].append(machine_from_marker(m, side))
        return name

    def estimate(self, name):
        pts = self.sightings.get(name, [])
        if not pts:
            return None
        xs = sorted(p[0] for p in pts)
        ys = sorted(p[1] for p in pts)
        cx, cy = xs[len(xs) // 2], ys[len(ys) // 2]           # median, robust to outliers
        close = [p for p in pts if math.hypot(p[0] - cx, p[1] - cy) <= self.max_spread]
        direction = circular_mean([p[2] for p in close]) if close else circular_mean([p[2] for p in pts])
        zone = self.field.zone_at(cx, cy, prefix='M')
        return {
            'machine': name, 'x': round(cx, 3), 'y': round(cy, 3), 'zone': zone,
            'rotation': refbox_rotation(direction), 'input_dir_deg': round(math.degrees(direction), 1),
            'sightings': len(pts), 'agreeing': len(close),
            'ready': len(close) >= self.min_sightings and zone is not None,
        }

    def all(self):
        return [self.estimate(n) for n in sorted(self.sightings)]
