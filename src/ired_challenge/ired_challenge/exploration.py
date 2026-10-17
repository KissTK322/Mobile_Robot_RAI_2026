"""Exploration challenge geometry (no ROS): ArUco sightings -> MPS zone and rotation.

Each MPS station is a 20 x 20 cm box with an ArUco marker on its input side and
another on its output side. The ArUco node reports, per marker, its position in
base_link and `marker_yaw_deg`: the direction the marker face points (out of
the box) in base_link. With the robot pose that gives the marker in the map,
the box centre is 10 cm behind the marker, and the input side direction is the
face direction (input marker) or its opposite (output marker).

Refbox conventions (rcll-refbox source and RCLL rulebook "Machine Positioning"):
- rotation = direction of the conveyor axis from the output side to the input
  side, in degrees counter-clockwise from the field +x axis, 0..315 in 45 deg
  steps. With the map aligned to the field that is simply the input direction.
- the refbox keeps only the FIRST reported zone and rotation of each machine,
  so a machine is reported only when the estimate is solid.
Check the map/field alignment once on the field against /refbox/machine_info."""
import math
from collections import defaultdict

# marker id -> (machine name, side), RCLL rulebook table "markers":
# even id = input side, odd id = output side; magenta = cyan id + 100.
_CYAN = {'CS1': 101, 'CS2': 103, 'RS1': 111, 'RS2': 113, 'BS': 121, 'DS': 131, 'SS': 141}
DEFAULT_MARKERS = {}
for _name, _out in _CYAN.items():
    for _team, _add in (('C', 0), ('M', 100)):
        DEFAULT_MARKERS[_out + _add] = (f'{_team}-{_name}', 'output')
        DEFAULT_MARKERS[_out + _add + 1] = (f'{_team}-{_name}', 'input')


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


def refbox_rotation(input_dir_rad, zero_deg=0.0, sign=1.0, step=45):
    """Input-side direction in the map -> refbox rotation degrees, snapped to `step`.

    Refbox: counter-clockwise from field +x (zero_deg=0, sign=+1). zero_deg / sign
    are only for a map that is rotated or mirrored against the field."""
    deg = (zero_deg + sign * math.degrees(input_dir_rad)) % 360
    return int(round(deg / step) * step) % 360


def circular_mean(angles):
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


class MachineEstimator:
    """Collects sightings and decides when a machine is known well enough to report.

    A wrong report costs points and only the first report counts, so a machine is
    `ready` only with `min_sightings` frames that agree within `max_spread` m, seen
    from at least `min_views` different robot positions, with the box centre at
    least `zone_margin` m inside its zone (as Carologistics did: >= 10 frames,
    tag inside the zone with a margin)."""

    def __init__(self, field, markers=None, min_sightings=10, max_spread=0.15, max_range=2.0,
                 min_views=2, zone_margin=0.08):
        self.field = field
        self.markers = dict(markers or DEFAULT_MARKERS)
        self.min_sightings, self.max_spread, self.max_range = min_sightings, max_spread, max_range
        self.min_views, self.zone_margin = min_views, zone_margin
        self.sightings = defaultdict(list)       # machine -> [(cx, cy, input_dir)]
        self.views = defaultdict(set)            # machine -> robot positions rounded to 0.3 m

    def add(self, robot_pose, marker_id, pos_base, marker_yaw_deg):
        """Add one detection. Returns the machine name or None if ignored."""
        if marker_id not in self.markers or pos_base is None or marker_yaw_deg is None:
            return None
        if math.hypot(pos_base[0], pos_base[1]) > self.max_range:
            return None                          # far detections are too noisy
        name, side = self.markers[marker_id]
        m = marker_in_map(robot_pose, pos_base, marker_yaw_deg)
        self.sightings[name].append(machine_from_marker(m, side))
        self.views[name].add((round(robot_pose[0] / 0.3), round(robot_pose[1] / 0.3)))
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
        zone = self.field.zone_at(cx, cy)
        well_inside = zone is not None and self.field.inside(zone, cx, cy, margin=self.zone_margin)
        return {
            'machine': name, 'x': round(cx, 3), 'y': round(cy, 3), 'zone': zone,
            'rotation': refbox_rotation(direction), 'input_dir_deg': round(math.degrees(direction), 1),
            'sightings': len(pts), 'agreeing': len(close), 'views': len(self.views[name]),
            'ready': (len(close) >= self.min_sightings and len(self.views[name]) >= self.min_views
                      and well_inside),
        }

    def all(self):
        return [self.estimate(n) for n in sorted(self.sightings)]
