"""Zone-grid route planner for the navigation challenge (no ROS, testable on a PC).

Micromouse idea, adapted to an omni robot on the 5 x 5 m field:
- nodes are the zone centres; a zone with an MPS box uses the stop point beside the box
- two neighbouring nodes (straight or diagonal) are linked only if the straight segment
  keeps `clear` metres from every obstacle point (map walls and MPS boxes), so gaps that
  are only just wide enough for the robot are never used; a link into a box zone may
  pass that zone's own box at `own_clear` (the stop point is meant to be beside it)
- cost is time: length / speed, plus `turn_s` per 90 deg change of direction (braking);
  Dijkstra over (node, direction) states = a time-based flood fill
- the order of the zones is the exact fastest open tour (Held-Karp dynamic programming)
- a route is shortened by cutting corners where the straight line keeps `clear`

style='lanes' (route_mode lanes, the user's idea): routes run along the zone-centre
lanes with right-angle turns; a diagonal link or a cut corner is only used where it
keeps `diag_clear` (0.45 m) from walls and boxes, i.e. in open space. Narrow links
(between clear and diag_clear) cost extra seconds and entering a zone that holds a
box costs `box_risk_s`, so the order and the routes avoid the spots where drives
had to be retried in the sim, unless that saves more time than the risk costs.

Obstacles are given as points in the map frame: the occupied cells of the SLAM map
(walls, START fence) and the MPS footprints from the refbox. Nothing about the field
is hard-coded except the zone grid itself."""
import heapq
import math

import numpy as np

from ired_challenge.field import mps_footprint


def seg_point_dist(p, q, pts):
    """Smallest distance from the segment p-q to the points pts (N x 2 array)."""
    if len(pts) == 0:
        return math.inf
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p
    L = float(d @ d)
    if L == 0:
        return float(np.min(np.hypot(*(pts - p).T)))
    t = np.clip(((pts - p) @ d) / L, 0.0, 1.0)
    proj = p + t[:, None] * d
    return float(np.min(np.hypot(*(pts - proj).T)))


class GridPlanner:
    def __init__(self, field, clear=0.32, own_clear=0.27, speed=0.22, turn_s=0.6, box_margin=0.03,
                 style='grid', diag_clear=0.45, narrow_s=3.0, box_risk_s=4.0):
        self.field = field
        self.style = style
        self.diag_clear, self.narrow_s, self.box_risk_s = diag_clear, narrow_s, box_risk_s
        self.pen = {}                   # (a, b) -> extra seconds for a narrow link (lanes)
        self.clear, self.own_clear = clear, own_clear
        self.speed, self.turn_s = speed, turn_s
        self.box_margin = box_margin
        self.static = np.zeros((0, 2))
        self.boxes = {}                 # zone -> footprint points (N x 2)
        self.zones = [f'C_Z{a}{b}' for a in range(1, field.columns + 1) for b in range(1, field.rows + 1)]
        self.version = 0
        self._build()

    # ------------------------------------------------------------ inputs
    def set_static(self, points):
        """Occupied map cells (walls, fences) as N x 2 points in the map frame."""
        self.static = np.asarray(points, float).reshape(-1, 2)
        self._build()

    def set_boxes(self, boxes):
        """boxes: iterable of (zone, rotation_deg) from the refbox machine info."""
        new = {}
        for zone, rot in boxes:
            try:
                c = self.field.center(zone)
            except ValueError:
                continue
            new[zone] = np.array(mps_footprint(c, rot, margin=self.box_margin, step=0.03), float)
        if set(new) != set(self.boxes) or any(not np.array_equal(new[z], self.boxes[z]) for z in new):
            self.boxes = new
            self._build()

    # ------------------------------------------------------------ graph
    def _clear_seg(self, p, q, own=()):
        if seg_point_dist(p, q, self.static) < self.clear:
            return False
        for zone, pts in self.boxes.items():
            if seg_point_dist(p, q, pts) < (self.own_clear if zone in own else self.clear):
                return False
        return True

    def _margin(self, p, q, own=()):
        """Distance of segment p-q to the walls and to the boxes not in `own`."""
        d = seg_point_dist(p, q, self.static)
        for zone, pts in self.boxes.items():
            if zone not in own:
                d = min(d, seg_point_dist(p, q, pts))
        return d

    def _open(self, p, q, own=()):
        """lanes: may this straight line be used although it is not along a lane?"""
        axis = abs(p[0] - q[0]) < 1e-6 or abs(p[1] - q[1]) < 1e-6
        return self.style != 'lanes' or axis or self._margin(p, q, own) >= self.diag_clear

    def _build(self):
        # Built into locals and swapped in at the end: the ROS node updates the map and
        # boxes from its executor thread while the task thread may be planning.
        blocked = list(self.boxes)
        node = {z: (self.field.goal_in_zone(z, self.field.center(z), blocked) if z in self.boxes
                    else self.field.center(z)) for z in self.zones}
        links = {z: [] for z in self.zones}
        pen = {}
        for i, a in enumerate(self.zones):
            _, ax, ay = self.field.parse(a)
            for b in self.zones[i + 1:]:
                _, bx, by = self.field.parse(b)
                if max(abs(ax - bx), abs(ay - by)) == 1 and self._clear_seg(node[a], node[b], own=(a, b)):
                    diagonal = abs(ax - bx) == 1 and abs(ay - by) == 1
                    if self.style == 'lanes':
                        m = self._margin(node[a], node[b], own=(a, b))
                        if diagonal and m < self.diag_clear:
                            continue            # lanes: no diagonal close to a wall or a box
                        span = max(self.diag_clear - self.clear, 1e-6)
                        pen[(a, b)] = pen[(b, a)] = self.narrow_s * min(1.0, max(0.0, (self.diag_clear - m) / span))
                    links[a].append(b)
                    links[b].append(a)
        self.node, self.links, self.pen = node, links, pen
        self.version += 1

    def _start_links(self, xy):
        """Nodes reachable in a straight, clear line from an arbitrary point (the robot)."""
        own = tuple(z for z in self.boxes if self.field.zone_at(*xy) == z)
        out = []
        for z in self.zones:
            if math.dist(xy, self.node[z]) <= 1.5 and self._clear_seg(xy, self.node[z], own=own + (z,)) \
                    and self._open(xy, self.node[z], own=own + (z,)):
                out.append(z)
        return out

    # ------------------------------------------------------------ search
    def _edge_cost(self, p, q, prev_dir):
        d = math.atan2(q[1] - p[1], q[0] - p[0])
        cost = math.dist(p, q) / self.speed
        if prev_dir is not None:
            turn = abs((d - prev_dir + math.pi) % (2 * math.pi) - math.pi)
            if turn > 0.1:
                cost += self.turn_s * turn / (math.pi / 2)
        return cost, d

    def _dijkstra(self, start_xy, start_links, targets):
        """Fastest time and node path from start_xy to each target zone."""
        best = {}
        pq = []
        for z in start_links:
            c, d = self._edge_cost(start_xy, self.node[z], None)
            if self.style == 'lanes' and z in self.boxes and math.dist(start_xy, self.node[z]) > 0.05:
                c += self.box_risk_s
            heapq.heappush(pq, (c, z, round(d, 3), (z,)))
        seen = {}
        left = set(targets)
        while pq and left:
            t, z, d, path = heapq.heappop(pq)
            if seen.get((z, d), math.inf) <= t:
                continue
            seen[(z, d)] = t
            if z in left:
                left.discard(z)
                best[z] = (t, list(path))
            if z in self.boxes and math.dist(self.node[z], start_xy) > 0.05:
                continue                # a stop point beside a box is an end point (or where the robot
                                        # already is), never a way through: that passes the box at the
                                        # robot radius
            for n in self.links[z]:
                c, nd = self._edge_cost(self.node[z], self.node[n], d)
                c += self.pen.get((z, n), 0.0)
                if self.style == 'lanes' and n in self.boxes:
                    c += self.box_risk_s        # stopping beside a box: the riskiest drive
                heapq.heappush(pq, (t + c, n, round(nd, 3), path + (n,)))
        return best

    def path(self, from_xy, zone):
        """Waypoints (map frame) from from_xy to the stop point of `zone`, corners cut
        where clear; None if the zone cannot be reached on the grid."""
        if self.field.zone_at(*from_xy) == zone:
            return [tuple(from_xy), self.node[zone]]
        res = self._dijkstra(tuple(from_xy), self._start_links(tuple(from_xy)), [zone])
        if zone not in res:
            return None
        pts = [tuple(from_xy)] + [self.node[z] for z in res[zone][1]]
        names = [None] + res[zone][1]
        return self._shortcut(pts, names)

    def _shortcut(self, pts, names):
        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not (self._clear_seg(pts[i], pts[j], own=tuple(n for n in (names[i], names[j]) if n))
                                     and self._open(pts[i], pts[j], own=tuple(n for n in (names[i], names[j]) if n))):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def travel_time(self, from_xy, zone):
        res = self._dijkstra(tuple(from_xy), self._start_links(tuple(from_xy)), [zone])
        return res[zone][0] if zone in res else math.inf

    def order(self, from_xy, zones):
        """Fastest visiting order of `zones` starting at from_xy (Held-Karp, exact).
        Zones that cannot be reached on the grid are put last, in the given order."""
        zones = list(dict.fromkeys(zones))
        start = self._dijkstra(tuple(from_xy), self._start_links(tuple(from_xy)), zones)
        reach = [z for z in zones if z in start]
        rest = [z for z in zones if z not in start]
        n = len(reach)
        if n <= 1:
            return reach + rest
        T = [[0.0] * n for _ in range(n)]
        for i, a in enumerate(reach):
            r = self._dijkstra(self.node[a], self.links[a], reach)
            for j, b in enumerate(reach):
                T[i][j] = 0.0 if i == j else (r[b][0] if b in r else math.inf)
        full = 1 << n
        INF = math.inf
        dp = [[INF] * n for _ in range(full)]
        par = [[-1] * n for _ in range(full)]
        for i, z in enumerate(reach):
            dp[1 << i][i] = start[z][0]
        for mask in range(full):
            row = dp[mask]
            for last in range(n):
                cur = row[last]
                if cur == INF:
                    continue
                Tl = T[last]
                for nxt in range(n):
                    if mask & (1 << nxt):
                        continue
                    nm = mask | (1 << nxt)
                    v = cur + Tl[nxt]
                    if v < dp[nm][nxt]:
                        dp[nm][nxt] = v
                        par[nm][nxt] = last
        mask = full - 1
        last = min(range(n), key=lambda i: dp[mask][i])
        if dp[mask][last] == INF:
            return reach + rest                 # some pair not connected: keep it simple
        seq = []
        while last != -1:
            seq.append(reach[last])
            last, mask = par[mask][last], mask & ~(1 << last)
        return seq[::-1] + rest

    def tour_time(self, from_xy, seq):
        t, cur = 0.0, tuple(from_xy)
        for z in seq:
            t += self.travel_time(cur, z)
            cur = self.node[z]
        return t
