"""Find the MPS boxes with the LiDAR (pure Python/numpy, cheap enough for the Pi).

The seniors stack two boxes (~40 cm, 2026-10-28), above the 30.5 cm LiDAR plane, so the scan shows
each box as a short cluster of points that is not a field wall. `BoxFinder.add` takes one scan in the
map frame (x, y points) and counts, per zone, how many scans showed a box-sized cluster there; `zones`
returns the zones seen often enough. Only a hint for the exploration order: the camera still decides
which machine it is and how it is turned."""
import math

import numpy as np


class BoxFinder:
    def __init__(self, field, wall_dist=0.12, eps=0.10, min_points=3, max_extent=0.42, min_scans=5,
                 field_margin=0.12, centre_tol=0.25):
        self.field = field
        self.wall_dist, self.eps, self.min_points = wall_dist, eps, min_points
        self.max_extent, self.min_scans, self.margin = max_extent, min_scans, field_margin
        # a box stands at a zone centre. Field game 2026-10-28: the two boxes landed 2-5 cm from it
        # (sd 7-10 cm) while the false zones were bits of wall and START fence 0.4 m and more away
        self.centre_tol = centre_tol
        self.walls = np.zeros((0, 2))
        self.counts = {}

    def set_walls(self, walls_xy):
        self.walls = np.asarray(walls_xy, float).reshape(-1, 2)

    def _not_wall(self, pts):
        if not len(self.walls) or not len(pts):
            return np.ones(len(pts), bool)
        keep = np.ones(len(pts), bool)
        for i in range(0, len(pts), 256):               # chunks: bounded memory on the Pi
            d = np.min(np.linalg.norm(pts[i:i + 256, None, :] - self.walls[None, :, :], axis=2), axis=1)
            keep[i:i + 256] = d > self.wall_dist
        return keep

    def clusters(self, pts):
        """Group points closer than eps (scan order is enough: a box is a run of neighbours)."""
        out, cur = [], []
        for p in pts:
            if cur and math.dist(p, cur[-1]) > self.eps:
                out.append(cur)
                cur = []
            cur.append(p)
        if cur:
            out.append(cur)
        return out

    def add(self, pts_xy, robot_xy):
        """One scan (points in scan order, map frame). Returns the zones seen in this scan."""
        pts = np.asarray(pts_xy, float).reshape(-1, 2)
        f = self.field
        lo, hi = self.margin, f.columns * f.size - self.margin
        inside = (pts[:, 0] > f.ox + lo) & (pts[:, 0] < f.ox + hi) & (pts[:, 1] > f.oy + lo) & \
                 (pts[:, 1] < f.oy + f.rows * f.size - self.margin)
        pts = pts[inside]
        pts = pts[self._not_wall(pts)]
        seen = set()
        for c in self.clusters(pts):
            if len(c) < self.min_points:
                continue
            c = np.array(c)
            if np.max(np.linalg.norm(c - c.mean(0), axis=1)) * 2 > self.max_extent:
                continue                                  # longer than a box: a wall piece or a person
            m = c.mean(0)
            # the scan sees the faces towards the robot: move ~7 cm away from it to the box centre
            d = m - np.asarray(robot_xy, float)
            n = np.linalg.norm(d)
            centre = m + (d / n * 0.07 if n > 1e-6 else 0)
            zone = f.zone_at(centre[0], centre[1])
            if zone and math.dist(centre, f.center(zone)) <= self.centre_tol:
                seen.add(zone)
        for z in seen:
            self.counts[z] = self.counts.get(z, 0) + 1
        return seen

    def zones(self):
        return {z for z, n in self.counts.items() if n >= self.min_scans}
