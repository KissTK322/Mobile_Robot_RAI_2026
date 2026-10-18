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
report counts) -> done when every expected machine is reported.

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
                 red_zone='C_Z32', candidate=None, free=None, look_time=1.5, move_timeout=30.0):
        self.nav, self.field, self.est, self.report, self.log = nav, field, estimator, report, log
        self.expected = list(expected)
        self.red_zone = red_zone
        self.candidate = candidate or (lambda zone: True)    # may this zone hold a machine?
        self.free = free or (lambda p: True)                 # is this point clear of walls?
        self.look_time, self.move_timeout = look_time, move_timeout
        self.state, self.t_state = WAIT_GAME, 0.0
        self.reported = {}                                   # machine -> report dict
        self.queue, self.looks = [], []
        self.here = None
        self.red_done = False
        self.heading = None                                  # last commanded heading
        self.t_still = None                                  # when the current turn finished

    def _set(self, s, now):
        if s != self.state:
            self.log(f'{self.state} -> {s}')
        self.state, self.t_state = s, now

    def info(self):
        return {'state': self.state, 'reported': self.reported, 'left': len(self.queue),
                'estimates': [e for e in self.est.all() if e]}

    # ------------------------------------------------------------ helpers
    def _looks_at(self, p):
        """Headings from crossing p to the neighbouring zone centres worth looking at."""
        out = []
        for dx in (-0.5, 0.5):
            for dy in (-0.5, 0.5):
                zone = self.field.zone_at(p[0] + dx, p[1] + dy)
                if zone and self.candidate(zone) and not self._zone_done(zone):
                    out.append(math.atan2(dy, dx))
        return out

    def _zone_done(self, zone):
        return any(r['zone'] == zone for r in self.reported.values())

    def _report_ready(self):
        for e in self.est.all():
            if not e or e['machine'] in self.reported or e['machine'] not in self.expected or not e['ready']:
                continue
            cx, cy = self.field.center(e['zone'])            # zone centre: safest for the zone
            if self.report(e['machine'], cx, cy, e['rotation']):
                self.reported[e['machine']] = {'zone': e['zone'], 'rotation': e['rotation']}
                self.log(f"Reported {e['machine']}: {e['zone']} rotation {e['rotation']} "
                         f"({e['agreeing']} agreeing sightings, {e['views']} views)")

    def _next(self, pose, now):
        while self.queue:
            p = self.queue.pop(0)
            if not self._looks_at(p):
                continue                                     # nothing left to see from there
            start = self.here if self.here is not None else (pose[0], pose[1])
            pts = border_route(start, p) if self.here is not None else [(pose[0], pose[1]), p]
            self.target = p
            self.nav.go_via(pts, pose[2])
            self._set(MOVE, now)
            return
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
                self.looks = self._looks_at(self.target)
                self._look(now)
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
        else:
            self._next((self.here[0], self.here[1], self.heading if self.heading is not None else 0.0), now)
