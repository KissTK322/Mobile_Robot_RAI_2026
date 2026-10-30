"""Exploration challenge state machine (no ROS, so it can be tested on a PC).

The two machines are not sent by the refbox (course config exploration_easy.yaml:
send-mps-ground-truth: []), and the LiDAR scans over the 30 cm boxes. So the robot
only stands on and drives along the zone borders: a box stands at a zone centre, and
the crossings of the zone borders are 0.71 m from every zone centre, the border
lines 0.5 m, whatever is in the zones.

Flow: wait for the game -> leave START through the red zone (rules) -> visit the
border crossings in a back-and-forth sweep -> at each one, turn to face the
neighbouring zones that may hold a machine and collect ArUco sightings for `look_time`
-> report a machine as soon as MachineEstimator says it is solid (only the first
report counts) -> a machine seen well from one position only gets a second look right
away from a point a little along the border -> done when every expected machine is
reported.

`nav` must provide:
    go_via(points, yaw)  follow waypoints (map frame), keep the heading `yaw`
    turn_to(yaw)         turn in place to a heading
    state()              'active' | 'succeeded' | 'failed'
    cancel()
`report(machine, x, y, rotation_deg)` sends one machine report; returns True if sent."""
import math

WAIT_GAME, RED_ZONE, MOVE, LOOK, DONE = 'WAIT_GAME', 'RED_ZONE', 'MOVE', 'LOOK', 'DONE'


def crossings(field, free):
    """Inner crossings of the zone borders (x, y) that `free(p)` accepts (clear of walls)."""
    pts = [(float(i), float(j)) for j in range(1, field.rows) for i in range(1, field.columns)]
    return [p for p in pts if free(p)]


def sweep(points, start_xy):
    """Back-and-forth order (rows of equal y, alternating direction), starting with the
    row nearest to start_xy and running away from it."""
    rows = sorted({p[1] for p in points}, key=lambda y: abs(y - start_xy[1]))
    rows = sorted(rows) if rows and rows[0] <= sum(rows) / len(rows) else sorted(rows, reverse=True)
    out, left_to_right = [], start_xy[0] <= 2.5
    for y in rows:
        row = sorted((p for p in points if p[1] == y), key=lambda p: p[0], reverse=not left_to_right)
        out += row
        left_to_right = not left_to_right
    return out


def border_route(a, b):
    """Waypoints from crossing a to crossing b along the zone borders (x first, then y)."""
    if a[0] == b[0] or a[1] == b[1]:
        return [a, b]
    return [a, (b[0], a[1]), b]


class ExplorationTask:
    def __init__(self, nav, field, estimator, report, log=print, expected=('C-BS', 'C-CS1'),
                 red_zone='C_Z32', candidate=None, free=None, look_time=1.5, move_timeout=30.0,
                 look_spread_deg=12.0, final_single_view=True, box_zones=None, relook_sightings=15,
                 relook_step=0.32):
        self.nav, self.field, self.est, self.report, self.log = nav, field, estimator, report, log
        self.expected = list(expected)
        self.red_zone = red_zone
        self.candidate = candidate or (lambda zone: True)    # may this zone hold a machine?
        self.free = free or (lambda p: True)                 # is this point clear of walls?
        self.look_time, self.move_timeout = look_time, move_timeout
        self.look_spread = math.radians(look_spread_deg)
        self.final_single_view = final_single_view
        self.state, self.t_state = WAIT_GAME, 0.0
        self.reported = {}                                   # machine -> report dict
        self.queue, self.looks = [], []
        self.here = None
        self.route, self.resends = [], 0         # current border route, re-sends after aborts
        self.red_done = False
        self.heading = None                                  # last commanded heading
        self.t_still = None                                  # when the current turn finished
        # LiDAR search (optional): box_zones() -> zones where the LiDAR sees a box (stacked boxes,
        # 2026-10-28). Their crossings are visited first and only those zones are looked at; the
        # normal sweep follows as the fallback for anything still missing.
        self.box_zones = box_zones
        self.focus = None                                    # zones looked at while focused
        self.planned = set()                                 # box zones already queued
        # second look (field game 2026-10-28: CS1 seen 24x at 41 s from its only reachable crossing,
        # reported at 218 s after the robot left and came back). A position counts as a new view
        # from another 0.3 m cell on (MachineEstimator), so relook_step must be over 0.3 m.
        self.relook_sightings, self.relook_step = relook_sightings, relook_step
        self.relooked = set()                                # machines that had their second look
        self.back = None                                     # crossing to return to after a second look

    def _set(self, s, now):
        if s != self.state:
            self.log(f'{self.state} -> {s}')
        self.state, self.t_state = s, now

    def info(self):
        return {'state': self.state, 'reported': self.reported, 'left': len(self.queue),
                'estimates': [e for e in self.est.all() if e]}

    # ------------------------------------------------------------ helpers
    def _looks_at(self, p):
        """Headings from crossing p to the neighbouring zone centres worth looking at.

        Each zone gets the centre heading and, with `look_spread`, one heading on each side of
        it: the box is only ~0.45 m from the lens, its markers are 0.1 m off the box centre and
        the robot stops up to ~0.1 m short of the crossing and ~4 deg short of the heading, so a
        marker of a box on a border zone (conveyor along the wall, seen only obliquely) can be
        outside the 35.5 deg image of the real camera when it looks at the zone centre."""
        out = []
        for dx in (-0.5, 0.5):
            for dy in (-0.5, 0.5):
                zone = self.field.zone_at(p[0] + dx, p[1] + dy)
                if self.focus is not None and zone not in self.focus:
                    continue
                if zone and self.candidate(zone) and not self._zone_done(zone):
                    h = math.atan2(dy, dx)
                    out += [h - self.look_spread, h, h + self.look_spread] if self.look_spread else [h]
        return out

    def _zone_done(self, zone):
        return any(r['zone'] == zone for r in self.reported.values())

    def _report_ready(self, final=False):
        """Report every machine the estimator calls ready. `final` (sweep finished, nothing more
        to see): also a machine seen from one robot position only, if it has enough agreeing
        sightings well inside a zone. A box on a border zone shows its markers only obliquely,
        so the second crossing next to it often sees nothing (PC sim 2026-10-26: 3 of 4 games
        found one machine and stood still for 350 s); a solid one-view estimate is then better
        than no report."""
        for e in self.est.all():
            if not e or e['machine'] in self.reported or e['machine'] not in self.expected:
                continue
            if not (e['ready'] or (final and e['agreeing'] >= self.est.min_sightings
                                   and e['well_inside'] and e['plausible'])):
                continue
            cx, cy = self.field.center(e['zone'])            # zone centre: safest for the zone
            if self.report(e['machine'], cx, cy, e['rotation']):
                self.reported[e['machine']] = {'zone': e['zone'], 'rotation': e['rotation']}
                self.log(f"Reported {e['machine']}: {e['zone']} rotation {e['rotation']} "
                         f"({e['agreeing']} agreeing sightings, {e['views']} views)")

    def _relook(self):
        """(machine, point, heading) for a second look from next to self.here, or None.

        Only for a machine solid but for its second view. Steps along the border lines, so the
        point is as safe as the lanes; the one seeing the same marker most head-on wins."""
        for e in self.est.all():
            if (not e or e['machine'] in self.reported or e['machine'] not in self.expected
                    or e['machine'] in self.relooked or e['agreeing'] < self.relook_sightings
                    or not e['well_inside'] or not e['plausible'] or e['ready']):
                continue
            mx, my, face = self.est.last_marker[e['machine']]
            best = None
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                q = (self.here[0] + dx * self.relook_step, self.here[1] + dy * self.relook_step)
                d = math.dist(q, (mx, my))
                # d >= 0.55: the lens is 0.26 m ahead of the robot; field game: read at 0.31 m from the lens
                if not self.free(q) or d < 0.55 or d > 1.2 or math.dist(q, (e['x'], e['y'])) < 0.5:
                    continue
                score = ((q[0] - mx) * math.cos(face) + (q[1] - my) * math.sin(face)) / d
                if score > 0.2 and (best is None or score > best[0]):
                    best = (score, q)
            if best:
                return e['machine'], best[1], math.atan2(my - best[1][1], mx - best[1][0])
        return None

    def _box_crossings(self, zones, start):
        """Inner crossings at the corners of `zones`, nearest-first from start."""
        pts = []
        for z in sorted(zones):
            cx, cy = self.field.center(z)
            for dx in (-0.5, 0.5):
                for dy in (-0.5, 0.5):
                    p = (cx + dx, cy + dy)
                    if 0 < p[0] < self.field.columns and 0 < p[1] < self.field.rows and self.free(p) and p not in pts:
                        pts.append(p)
        out, cur = [], start
        while pts:
            p = min(pts, key=lambda q: math.dist(q, cur))
            pts.remove(p)
            out.append(p)
            cur = p
        return out

    def _plan_boxes(self, pose):
        """Put the crossings of newly found LiDAR boxes at the front of the queue."""
        if self.box_zones is None:
            return
        new = {z for z in self.box_zones() if self.candidate(z) and not self._zone_done(z)} - self.planned
        if not new:
            return
        self.planned |= new
        self.focus = (self.focus or set()) | new
        front = self._box_crossings(new, (pose[0], pose[1]))
        self.queue = front + ['END_FOCUS'] + [p for p in self.queue if p not in front and p != 'END_FOCUS']
        self.log(f'LiDAR sees boxes in {sorted(new)}: looking there first')

    def _next(self, pose, now):
        self._plan_boxes(pose)
        while self.queue:
            if self.queue[0] == 'END_FOCUS':
                self.queue.pop(0)
                if self.focus is not None:
                    self.log('LiDAR boxes looked at; sweeping the rest of the field')
                self.focus = None
                continue
            p = self.queue.pop(0)
            if not self._looks_at(p):
                continue                                     # nothing left to see from there
            start = self.back or self.here or (pose[0], pose[1])
            pts = border_route(start, p) if self.here is not None else [(pose[0], pose[1]), p]
            if self.back and self.here is not None:
                pts = [self.here] + pts                      # back onto the crossing first
            self.back = None
            self.target = p
            self.route, self.resends = list(pts), 0
            self.nav.go_via(pts, pose[2])
            self._set(MOVE, now)
            return
        if self.final_single_view:
            self._report_ready(final=True)
        self.log(f'Sweep finished, reported {sorted(self.reported)}')
        self._set(DONE, now)

    # ------------------------------------------------------------ main step
    def update(self, now, game_on, pose):
        if not game_on:
            if self.state in (RED_ZONE, MOVE, LOOK):
                self.nav.cancel()
                self.log('Game stopped, robot stopped')
                self._set(WAIT_GAME, now)
            return self.info()
        self._report_ready()
        if all(m in self.reported for m in self.expected):
            if self.state != DONE:
                self.nav.cancel()
                self.log(f'All machines reported: {self.reported}')
                self._set(DONE, now)
            return self.info()
        if self.state == DONE:
            return self.info()

        if self.state == WAIT_GAME:
            if not self.red_done:
                cx, cy = self.field.center(self.red_zone)
                start = (pose[0], pose[1])
                self.nav.go_via([start, (cx, start[1]), (cx, cy)], pose[2])   # out of START, into the red zone
                self._set(RED_ZONE, now)
            else:
                self._next(pose, now)
            return self.info()

        st = self.nav.state()
        if self.state == RED_ZONE:
            if self.field.inside(self.red_zone, pose[0], pose[1], margin=0.1) or st == 'succeeded':
                self.red_done = True
                self.log('Red zone passed')
                pts = [p for p in crossings(self.field, self.free)]
                # first hop: the nearest border crossing (a short move inside the red zone,
                # which never holds a machine), then the sweep from there along the borders
                first = min(pts, key=lambda p: math.dist(p, pose[:2]))
                self.queue = [first] + [p for p in sweep(pts, first) if p != first]
                self.here = None
                self._next(pose, now)
            elif st == 'failed' or now - self.t_state > self.move_timeout:
                self.log('Red zone drive failed, trying again')
                self._set(WAIT_GAME, now)
            return self.info()

        if self.state == MOVE:
            if st == 'succeeded' or (st != 'active' and math.dist(pose[:2], self.target) < 0.2):
                self.here = self.target
                if self.back is None:
                    self.looks = self._looks_at(self.target)
                self._look(now)
            elif st == 'failed' and self.resends < 3 and now - self.t_state > 1.0:
                # DWB aborts some drives on the way ("Resulting plan has 0 poses"): send the
                # rest of the same border route from here instead of skipping the point
                self.resends += 1
                rest = self.route[min(range(len(self.route)), key=lambda i: math.dist(self.route[i], pose[:2])) + 1:]
                self.log(f'Drive to {self.target} aborted at ({pose[0]:.2f}, {pose[1]:.2f}), sending the rest of the route')
                self.nav.go_via([(pose[0], pose[1])] + (rest or [self.target]), pose[2])
                self.t_state = now
            elif st == 'failed' or now - self.t_state > self.move_timeout:
                self.log(f'Could not reach {self.target}, skipping it')
                self.here = None
                self._next(pose, now)
            return self.info()

        if self.state == LOOK:
            if st == 'active' and now - self.t_state < self.move_timeout:
                return self.info()
            if self.t_still is None:
                self.t_still = now                           # turned (or gave up turning): now look
            if now - self.t_still >= self.look_time:
                self._look(now)
            return self.info()
        return self.info()

    def _look(self, now):
        self.t_still = None
        if self.looks:
            self.heading = self.looks.pop(0)
            self.nav.turn_to(self.heading)
            self.t_state = now
            self.state = LOOK
            return
        pose = (self.here[0], self.here[1], self.heading if self.heading is not None else 0.0)
        again = self._relook() if self.back is None else None
        if again:
            machine, q, h = again
            self.relooked.add(machine)
            self.log(f'{machine} seen from one position only: second look from ({q[0]:.2f}, {q[1]:.2f})')
            self.back, self.target = self.here, q
            self.looks = [h - self.look_spread, h, h + self.look_spread] if self.look_spread else [h]
            self.route, self.resends = [self.here, q], 0
            self.nav.go_via([self.here, q], pose[2])
            self._set(MOVE, now)
            return
        self._next(pose, now)
