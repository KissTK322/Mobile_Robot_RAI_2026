"""Navigation challenge state machine (no ROS, so it can be tested on a PC).

Flow: wait for the game -> drive through the red zone -> visit every zone the
refbox sends, staying DWELL seconds in each until the refbox drops it from the
`remaining` list -> done. Failed drives are retried, then the zone is moved to
the end of the list and tried again later. If the game stops, the robot stops
and continues from where it was when the game runs again.

The ROS node feeds update() with the game state, the zones still to visit and
the robot pose, and gives it a `nav` object with:
    go_to(x, y, yaw)   start driving to a pose in the map frame
    state()            'active' | 'succeeded' | 'failed'
    cancel()
    clear_costmaps()
    escape()           optional: slide away from nearby obstacles (stuck in inflation)
    go_via(points, yaw) optional: follow the given waypoints (grid route)

With a `router` (ired_challenge.grid_planner.GridPlanner, route_mode 'grid') the zones
are visited in the fastest order on the zone grid and each drive follows the grid
route; from the second try of a zone on, Nav2 plans the drive itself. Without a router
(route_mode 'nav2') the task behaves exactly as before: nearest-zone order, Nav2 plans."""
from ired_challenge.field import visit_order

WAIT_GAME, RED_ZONE, PICK, DRIVE, DWELL, DONE = 'WAIT_GAME', 'RED_ZONE', 'PICK', 'DRIVE', 'DWELL', 'DONE'


class NavigationTask:
    def __init__(self, nav, field, log=print, red_zone='C_Z32', dwell=5.5, confirm_timeout=4.0,
                 drive_timeout=40.0, retries=2, order='given', blocked_zones=(), router=None,
                 grid_tries=1, stuck_time=8.0, stuck_dist=0.10):
        self.nav, self.field, self.log = nav, field, log
        self.red_zone = red_zone
        self.dwell = dwell                      # refbox needs 5 s inside the zone
        self.confirm_timeout = confirm_timeout  # extra wait for the refbox to drop the zone
        self.drive_timeout = drive_timeout
        self.retries = retries
        self.order = order
        self.blocked = list(blocked_zones)      # zones with an MPS station
        self.state = WAIT_GAME
        self.red_done = False
        self.plan = []                          # zones still to visit, in order
        self.target = None
        self.tries = 0
        self.t_state = 0.0
        self.t_inside = None
        self.visited = []
        self.deferred = []
        self.fast_fails = 0                     # drives that failed at once, in a row
        self.router = router                    # GridPlanner, or None for plain Nav2 goals
        self.grid_tries = grid_tries            # tries per zone on the grid route before Nav2 plans
        self.stuck_time, self.stuck_dist = stuck_time, stuck_dist
        self.progress = None                    # (time, xy) of the last real movement

    # ---------------------------------------------------------------- helpers
    def _set(self, state, now):
        if state != self.state:
            self.log(f'{self.state} -> {state}' + (f' ({self.target})' if self.target else ''))
        self.state, self.t_state = state, now

    def _drive_to(self, zone, pose, now):
        self.target = zone
        pts = None
        if self.router is not None and self.tries < self.grid_tries and hasattr(self.nav, 'go_via'):
            pts = self.router.path((pose[0], pose[1]), zone)
        if pts:
            self.nav.go_via(pts, pose[2])       # omni base: keep the heading, no turning
        else:
            if self.router is not None:
                x, y = self.router.node[zone]
                self.log(f'{zone}: Nav2 plans this drive' + ('' if self.tries else ' (no grid route)'))
            else:
                x, y = self.field.goal_in_zone(zone, (pose[0], pose[1]), self.blocked)
            self.nav.go_to(x, y, pose[2])
        self.progress = (now, (pose[0], pose[1]))
        self._set(RED_ZONE if zone == self.red_zone and not self.red_done else DRIVE, now)

    def _stuck(self, now, pose):
        """True if the robot has not moved stuck_dist in the last stuck_time seconds."""
        if self.progress is None:
            self.progress = (now, (pose[0], pose[1]))
            return False
        t, xy = self.progress
        if abs(pose[0] - xy[0]) + abs(pose[1] - xy[1]) >= self.stuck_dist:
            self.progress = (now, (pose[0], pose[1]))
            return False
        return now - t > self.stuck_time

    def _order(self, pose):
        fresh = [z for z in self.plan if z not in self.deferred]
        late = [z for z in self.plan if z in self.deferred]
        if self.router is not None:
            return self.router.order((pose[0], pose[1]), fresh) + late
        return visit_order(fresh, (pose[0], pose[1]), self.field, self.order) + late

    def info(self):
        return {'state': self.state, 'target': self.target, 'plan': self.plan,
                'visited': self.visited, 'red_zone_done': self.red_done}

    # ---------------------------------------------------------------- main step
    def update(self, now, game_on, remaining, pose):
        """Call ~10 times a second. remaining: zone names the refbox still wants
        (None if no routes received yet). pose: (x, y, yaw) in the map frame."""
        if not game_on:
            if self.state in (RED_ZONE, DRIVE, DWELL):
                self.nav.cancel()
                self.log('Game stopped, robot stopped')
                self.target = None
                self._set(WAIT_GAME, now)
            return self.info()

        if self.state == DONE:
            return self.info()

        if self.state == WAIT_GAME:
            if not self.red_done:
                self.tries = 0
                self._drive_to(self.red_zone, pose, now)
            else:
                self._set(PICK, now)
            return self.info()

        # Read the drive state once per update: it can change between two calls
        # (Nav2 finishing in between made a reached zone look like a failed drive).
        nav_state = self.nav.state() if self.state in (RED_ZONE, DRIVE) else None

        if self.state == RED_ZONE:
            # Passing through is enough: done as soon as the robot is inside it
            if self.field.inside(self.red_zone, pose[0], pose[1], margin=0.1) or nav_state == 'succeeded':
                self.nav.cancel()
                self.red_done = True
                self.log('Red zone passed')
                self.target = None
                self._set(PICK, now)
            elif nav_state == 'failed' or now - self.t_state > self.drive_timeout or self._stuck(now, pose):
                if nav_state == 'active':
                    self.log(f'Red zone: no progress at ({pose[0]:.2f}, {pose[1]:.2f})')
                self.fast_fails = self.fast_fails + 1 if nav_state == 'failed' and now - self.t_state < 2.0 else 0
                self._retry(pose, now)
            return self.info()

        if remaining is None:
            return self.info()                  # no routes from the refbox yet
        remaining = list(remaining)
        # keep our order, but follow the refbox: drop visited zones, add new ones
        self.plan = [z for z in self.plan if z in remaining] + [z for z in remaining if z not in self.plan]

        if self.state == PICK:
            if not self.plan:
                self.target = None
                self._set(DONE, now)
                self.log(f'All zones visited: {self.visited}')
                return self.info()
            self.plan = self._order(pose)
            self.tries = 0
            self._drive_to(self.plan[0], pose, now)
            return self.info()

        if self.state == DRIVE:
            if self.target not in remaining:            # refbox already counted it
                self.nav.cancel()
                self._visited(now)
            elif nav_state == 'succeeded' and self.field.inside(self.target, pose[0], pose[1], margin=0.05):
                self.t_inside = now
                self._set(DWELL, now)
            elif nav_state in ('failed', 'succeeded') or now - self.t_state > self.drive_timeout \
                    or (nav_state == 'active' and self._stuck(now, pose)):
                why = (('timeout' if now - self.t_state > self.drive_timeout else 'no progress')
                       if nav_state == 'active' else f'drive {nav_state}')
                self.fast_fails = self.fast_fails + 1 if nav_state == 'failed' and now - self.t_state < 2.0 else 0
                self.log(f'{self.target}: {why} at ({pose[0]:.2f}, {pose[1]:.2f})')
                self._retry(pose, now)
            return self.info()

        if self.state == DWELL:
            if self.target not in remaining:
                self._visited(now)
            elif not self.field.inside(self.target, pose[0], pose[1], margin=0.02):
                self.log(f'Drifted out of {self.target}, driving back')
                self._drive_to(self.target, pose, now)
            elif now - self.t_inside > self.dwell + self.confirm_timeout:
                self.log(f'{self.target}: refbox did not confirm, driving to the zone centre again')
                self._retry(pose, now)
            return self.info()

        return self.info()

    def _visited(self, now):
        self.log(f'Visited {self.target}')
        self.visited.append(self.target)
        if self.target in self.plan:
            self.plan.remove(self.target)
        self.target = None
        self._set(PICK, now)

    def _retry(self, pose, now):
        self.nav.cancel()
        if self.fast_fails >= 2 and hasattr(self.nav, 'escape'):
            # Plans fail at once: the robot is inside an obstacle's inflation. Move it out first.
            self.log(f'Drives fail at once at ({pose[0]:.2f}, {pose[1]:.2f}): sliding away from obstacles')
            self.nav.escape()
            self.fast_fails = 0
        self.tries += 1
        zone = self.target
        if self.tries <= self.retries:
            self.log(f'{zone}: try {self.tries + 1} of {self.retries + 1}')
            self.nav.clear_costmaps()
            self._drive_to(zone, pose, now)
            return
        if zone == self.red_zone and not self.red_done:
            self.log('Red zone failed, trying again')     # cannot skip it, the rules require it
            self.tries = 0
            self.nav.clear_costmaps()
            self._drive_to(zone, pose, now)
            return
        self.log(f'{zone}: giving up for now, will come back to it last')
        self.deferred.append(zone)
        self.target = None
        self._set(PICK, now)


def visit_order_keep_deferred(plan, deferred, start_xy, field, mode):
    """Order the plan, then push zones that failed before to the end."""
    fresh = [z for z in plan if z not in deferred]
    late = [z for z in plan if z in deferred]
    return visit_order(fresh, start_xy, field, mode) + late

