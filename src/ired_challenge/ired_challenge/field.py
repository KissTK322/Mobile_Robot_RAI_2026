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

    def goal_in_zone(self, zone, from_xy, blocked_zones=(), robot_radius=0.27, mps_half_diag=0.184,
                     wall_gap=0.08, step=0.01):
        """Point to drive to inside `zone`.

        Normally the zone centre. If an MPS station stands in the zone (its centre
        is taken as the zone centre), stop beside it instead: clear of the box in any
        rotation, and as deep inside the zone as that allows, because the refbox
        counts the pose the robot reports and a goal near the zone edge plus Nav2's
        goal tolerance plus AMCL error can end up outside (seen in the PC sim: the
        old side point was 10 cm from the edge, the robot stopped 2 cm inside and
        the refbox never counted the zone). A field wall on a side of the zone takes
        the robot radius off that side too. Of the best points, the one nearest
        `from_xy` wins."""
        cx, cy = self.center(zone)
        if zone not in blocked_zones:
            return cx, cy
        half = self.size / 2
        # mps_half_diag: half diagonal of the box plus the 3 cm footprint margin used in the
        # costmap (0.13 * sqrt 2), so the stop point clears the box in any rotation
        clear = robot_radius + mps_half_diag + 0.01
        _, zx, zy = self.parse(zone)
        sx = -1 if zone.startswith('M') else 1       # magenta zones are mirrored in x
        # wall on each side: robot centre must keep robot_radius + wall_gap from it
        wall = {'-x': zx == 1 if sx > 0 else zx == self.columns, '+x': zx == self.columns if sx > 0 else zx == 1,
                '-y': zy == 1, '+y': zy == self.rows}
        need = robot_radius + wall_gap
        fx, fy = from_xy
        best, best_key = (cx, cy), None
        n = int(round(half / step))
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                dx, dy = i * step, j * step
                if math.hypot(dx, dy) < clear:
                    continue
                margins = [half + dx - (need if wall['-x'] else 0), half - dx - (need if wall['+x'] else 0),
                           half + dy - (need if wall['-y'] else 0), half - dy - (need if wall['+y'] else 0)]
                m = min(margins)
                if m < 0:
                    continue
                key = (round(m, 3), -math.hypot(cx + dx - fx, cy + dy - fy))
                if best_key is None or key > best_key:
                    best, best_key = (cx + dx, cy + dy), key
        return best


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
