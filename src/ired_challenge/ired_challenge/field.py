"""Field geometry for the Industrial Logistics Challenge (no ROS).

The field is 5 x 5 zones of 1 m. Zone "C_Zxy" is column x (1..5) and row y
(1..5); in the map frame its centre is ((x - 0.5) * size + origin_x,
(y - 0.5) * size + origin_y). With origin (0, 0) the map origin is the outer
corner of C_Z11, the same convention as the refbox example code."""
import math
import re

ZONE_RE = re.compile(r'^([CM])_Z([1-9])([1-9])$')


class Field:
    def __init__(self, zone_size=1.0, origin=(0.0, 0.0), columns=5, rows=5):
        self.size = zone_size
        self.ox, self.oy = origin
        self.columns, self.rows = columns, rows

    def parse(self, zone):
        m = ZONE_RE.match(zone.strip())
        if not m:
            raise ValueError(f'not a zone name: {zone!r}')
        x, y = int(m.group(2)), int(m.group(3))
        if not (1 <= x <= self.columns and 1 <= y <= self.rows):
            raise ValueError(f'zone outside the field: {zone!r}')
        return m.group(1), x, y

    def center(self, zone):
        _, x, y = self.parse(zone)
        return ((x - 0.5) * self.size + self.ox, (y - 0.5) * self.size + self.oy)

    def zone_at(self, px, py, prefix='C'):
        x = math.floor((px - self.ox) / self.size) + 1
        y = math.floor((py - self.oy) / self.size) + 1
        if 1 <= x <= self.columns and 1 <= y <= self.rows:
            return f'{prefix}_Z{x}{y}'
        return None

    def inside(self, zone, px, py, margin=0.0):
        """True if (px, py) is inside the zone, at least `margin` from its border."""
        cx, cy = self.center(zone)
        half = self.size / 2 - margin
        return abs(px - cx) <= half and abs(py - cy) <= half

    def goal_in_zone(self, zone, from_xy, blocked_zones=(), robot_radius=0.22, mps_half_diag=0.15):
        """Point to drive to inside `zone`.

        Normally the zone centre. If an MPS station stands in the zone (its centre
        is taken as the zone centre), stop beside it instead: on the side facing
        `from_xy`, far enough from the box for the robot to fit but still inside
        the zone."""
        cx, cy = self.center(zone)
        _, zx, zy = self.parse(zone)
        if not any(self.parse(b)[1:] == (zx, zy) for b in blocked_zones):
            return cx, cy
        offset = min(robot_radius + mps_half_diag + 0.03, self.size / 2 - 0.08)
        fx, fy = from_xy
        candidates = [(cx + offset, cy), (cx - offset, cy), (cx, cy + offset), (cx, cy - offset)]
        return min(candidates, key=lambda p: math.hypot(p[0] - fx, p[1] - fy))


def visit_order(zones, start_xy, field, mode='given'):
    """Order in which to visit `zones`.

    'given'   keep the refbox order (safe default while the rule is unclear)
    'nearest' greedy nearest-next from the robot, shorter driving if any order is allowed"""
    zones = list(zones)
    if mode == 'given':
        return zones
    if mode != 'nearest':
        raise ValueError(f'unknown order mode {mode!r}')
    out, pos, left = [], start_xy, zones[:]
    while left:
        nxt = min(left, key=lambda z: math.hypot(field.center(z)[0] - pos[0], field.center(z)[1] - pos[1]))
        out.append(nxt)
        left.remove(nxt)
        pos = field.center(nxt)
    return out
