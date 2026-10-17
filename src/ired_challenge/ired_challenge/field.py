"""Field geometry for the Industrial Logistics Challenge (no ROS).

The field is 5 x 5 zones of 1 m. Zone "C_Zxy" is column x (1..5) and row y
(1..5); in the map frame its centre is ((x - 0.5) * size + origin_x,
(y - 0.5) * size + origin_y). With origin (0, 0) the map origin is the outer
corner of C_Z11. This is the rcll-refbox convention (utils.clp pose-to-zone):
"C_" zones are the cyan half (x >= 0), "M_" zones the magenta half, mirrored
to negative x. The prefix is the field half, not "machine"."""
import math
import re

ZONE_RE = re.compile(r'^([CM])_Z([1-9])([1-9])$')


class Field:
    def __init__(self, zone_size=1.0, origin=(0.0, 0.0), columns=5, rows=5, magenta=False):
        self.magenta = magenta          # the course field is the cyan half only
        self.size = zone_size
        self.ox, self.oy = origin
        self.columns, self.rows = columns, rows

    def parse(self, zone):
        m = ZONE_RE.match(zone.strip())
        if not m:
            raise ValueError(f'not a zone name: {zone!r}')
        x, y = int(m.group(2)), int(m.group(3))
        if m.group(1) == 'M' and not self.magenta:
            raise ValueError(f'magenta zone on a cyan-only field: {zone!r}')
        if not (1 <= x <= self.columns and 1 <= y <= self.rows):
            raise ValueError(f'zone outside the field: {zone!r}')
        return m.group(1), x, y

    def center(self, zone):
        half, x, y = self.parse(zone)
        sx = -1.0 if half == 'M' else 1.0
        return (sx * (x - 0.5) * self.size + self.ox, (y - 0.5) * self.size + self.oy)

    def zone_at(self, px, py):
        dx = px - self.ox
        if dx < 0 and not self.magenta:
            return None
        x = math.floor(abs(dx) / self.size) + 1
        y = math.floor((py - self.oy) / self.size) + 1
        if 1 <= x <= self.columns and 1 <= y <= self.rows:
            return f"{'M' if dx < 0 else 'C'}_Z{x}{y}"
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
        if zone not in blocked_zones:
            return cx, cy
        offset = min(robot_radius + mps_half_diag + 0.03, self.size / 2 - 0.08)
        fx, fy = from_xy
        candidates = [(cx + offset, cy), (cx - offset, cy), (cx, cy + offset), (cx, cy - offset)]
        return min(candidates, key=lambda p: math.hypot(p[0] - fx, p[1] - fy))


def mps_footprint(center, rotation_deg, size=0.20, margin=0.03, step=0.03):
    """Points filling an MPS box footprint (square of `size` + `margin` each side),
    rotated by `rotation_deg` (refbox: counter-clockwise from +x) about `center`."""
    half = size / 2 + margin
    n = max(2, int(round(2 * half / step)) + 1)
    c, s = math.cos(math.radians(rotation_deg)), math.sin(math.radians(rotation_deg))
    pts = []
    for i in range(n):
        for j in range(n):
            u, v = -half + 2 * half * i / (n - 1), -half + 2 * half * j / (n - 1)
            pts.append((center[0] + c * u - s * v, center[1] + s * u + c * v))
    return pts


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
