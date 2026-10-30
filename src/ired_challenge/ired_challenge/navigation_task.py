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
import math

from ired_challenge.field import visit_order

WAIT_GAME, RED_ZONE, PICK, DRIVE, DWELL, BACKOFF, DONE = 'WAIT_GAME', 'RED_ZONE', 'PICK', 'DRIVE', 'DWELL', 'BACKOFF', 'DONE'


class NavigationTask:
    def __init__(self, nav, field, log=print, red_zone='C_Z32', dwell=5.5, confirm_timeout=4.0,
                 drive_timeout=40.0, retries=2, order='given', blocked_zones=(), router=None,
                 grid_tries=1, stuck_time=8.0, stuck_dist=0.10, fine=None, backoff=None,
                 leave_after=0.0, min_stop=5.0, assume_wait=5.0):
        self.nav, self.field, self.log = nav, field, log
        # Fine approach (optional): Nav2 calls a zone "reached" up to xy_goal_tolerance (0.10 m)
        # away. While the robot dwells in the zone anyway, a slow omni P-correction moves it onto
        # the stop point. dict(tol, gain, vmax, vmin, time) or None (off). Needs nav.nudge(vx, vy).
        self.fine = fine if fine and hasattr(nav, 'nudge') else None
        self.fine_t0 = None
        self.fine_on = False
        # Back off from a box (optional): the LiDAR looks over the 20 cm MPS boxes, and on the
        # robot (2026-10-26) a stop beside the C_Z33 box ended 0.38 m from its centre (goal 0.46
        # plus Nav2's tolerance); the next drive turned in place there and hit the box. After a
        # box zone is counted, slide straight away from the box first. dict(dist, speed, time).
        self.backoff = backoff if backoff and hasattr(nav, 'nudge') else None
        self.backoff_from = None
        self.red_zone = red_zone
        self.dwell = dwell                      # refbox needs 5 s inside the zone
        self.confirm_timeout = confirm_timeout  # extra wait for the refbox to drop the zone
        # Leave without waiting for the refbox message (optional, 0 = off). The refbox counts a zone
        # 5 s after the pose enters it, but broadcasts the routes only every 2 s and our node reads
        # them at 1 Hz: the field game of 2026-10-28 left about 9 s after entry. leave_after: s after
        # the robot entered the zone, min_stop: s after it reached the stop point. If the refbox
        # still lists the zone assume_wait s later, it goes back on the plan: costs time, no zone.
        self.leave_after, self.min_stop, self.assume_wait = leave_after, min_stop, assume_wait
        self.t_entry = None                     # when the robot entered the target zone
        self.assumed = {}                       # zone left before the refbox said so -> time
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
        self.goal_xy = None                     # stop point of the current drive
        self.drive_from = None                  # where the current drive started
        self.resends = 0                        # routes re-sent for this zone after a mid-way abort

    # ---------------------------------------------------------------- helpers
    def _set(self, state, now):
        if state != self.state:
            self.log(f'{self.state} -> {state}' + (f' ({self.target})' if self.target else ''))
            if self.state == DWELL:
                self._fine_stop()
            if state == DWELL:
                self.fine_t0 = None
        self.state, self.t_state = state, now

    def _may_leave(self, now):
        return (self.leave_after > 0 and self.t_entry is not None and now - self.t_entry >= self.leave_after
                and now - self.t_inside >= self.min_stop)

    def _drive_to(self, zone, pose, now):
        self.target = zone
        self.t_entry = None
        pts = None
        if self.router is not None and self.tries < self.grid_tries and hasattr(self.nav, 'go_via'):
            pts = self.router.path((pose[0], pose[1]), zone)
        if pts:
            self.goal_xy = tuple(pts[-1])
            self.nav.go_via(pts, pose[2])       # omni base: keep the heading, no turning
        else:
            if self.router is not None:
                x, y = self.router.node[zone]
                self.log(f'{zone}: Nav2 plans this drive' + ('' if self.tries else ' (no grid route)'))
            else:
                x, y = self.field.goal_in_zone(zone, (pose[0], pose[1]), self.blocked)
            self.goal_xy = (x, y)
            self.nav.go_to(x, y, pose[2])
        self.progress = (now, (pose[0], pose[1]))
        self.drive_from = (pose[0], pose[1])
        self._set(RED_ZONE if zone == self.red_zone and not self.red_done else DRIVE, now)

    def _fine_step(self, now, pose):
        """Slow P-correction onto goal_xy during the dwell (robot frame, omni, heading kept).
        Stops at fine['tol'] or after fine['time'] s; the zone keeps counting meanwhile."""
        f, goal = self.fine, self.goal_xy
        if goal is None:
            return
        if self.fine_t0 is None:
            self.fine_t0 = now
        ex, ey = goal[0] - pose[0], goal[1] - pose[1]
        err = math.hypot(ex, ey)
        if err <= f['tol'] or now - self.fine_t0 > f['time'] or err > 0.3:
            self._fine_stop()
            return
        c, s = math.cos(pose[2]), math.sin(pose[2])
        bx, by = c * ex + s * ey, -s * ex + c * ey          # error in the robot frame
        v = min(f['vmax'], max(f['vmin'], f['gain'] * err))  # vmin: wheels need a minimum to turn
        self.nav.nudge(v * bx / err, v * by / err)
        self.fine_on = True

    def _backoff_step(self, now, pose):
        """Slide away from backoff_from (robot frame, heading kept). True when done: far
        enough, out of time, or the next 0.3 m would come close to the field walls."""
        b = self.backoff
        ex, ey = pose[0] - self.backoff_from[0], pose[1] - self.backoff_from[1]
        d = math.hypot(ex, ey)
        ux, uy = (ex / d, ey / d) if d > 1e-3 else (math.cos(pose[2] + math.pi), math.sin(pose[2] + math.pi))
        f = self.field
        ahead = (abs(pose[0] + 0.3 * ux - f.ox), pose[1] + 0.3 * uy - f.oy)   # magenta half is mirrored in x
        size, cols, rows = f.size, f.columns, f.rows
        walls_ok = 0.25 <= ahead[0] <= cols * size - 0.25 and 0.25 <= ahead[1] <= rows * size - 0.25
        if d >= b['dist'] or now - self.t_state > b['time'] or not walls_ok:
            if d < b['dist']:
                self.log(f'Back-off stopped {d:.2f} m from the box' + ('' if walls_ok else ' (wall)'))
            self._backoff_stop()
            return True
        c, s = math.cos(pose[2]), math.sin(pose[2])
        v = b['speed']
        self.nav.nudge(v * (c * ux + s * uy), v * (-s * ux + c * uy))
        self.backoff_on = True
        return False

    def _backoff_stop(self):
        if getattr(self, 'backoff_on', False):
            self.nav.nudge(0.0, 0.0)
            self.backoff_on = False

    def _fine_stop(self):
        if self.fine_on:
            self.nav.nudge(0.0, 0.0)
            self.fine_on = False

    def _short_of_goal(self, pose, reach=0.3):
        """Within `reach` of the stop point and well inside the target zone."""
        goal = self.goal_xy
        return (goal is not None and math.dist(pose[:2], goal) <= reach
                and self.field.inside(self.target, pose[0], pose[1], margin=0.1))

    def _aborted_on_the_way(self, pose, now, moved=0.3, max_resends=3):
        """The drive ran for a while and the robot really moved before Nav2 gave up."""
        return (self.drive_from is not None and self.resends < max_resends and now - self.t_state >= 2.0
                and math.dist(pose[:2], self.drive_from) >= moved)

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
                'visited': self.visited, 'red_zone_done': self.red_done,
                'retries': getattr(self, 'retry_total', 0), 'deferred': list(getattr(self, 'deferred', []))}

    # ---------------------------------------------------------------- main step
    def update(self, now, game_on, remaining, pose):
        """Call ~10 times a second. remaining: zone names the refbox still wants
        (None if no routes received yet). pose: (x, y, yaw) in the map frame."""
        if not game_on:
            if self.state in (RED_ZONE, DRIVE, DWELL, BACKOFF):
                self.nav.cancel()
                self._fine_stop()
                self._backoff_stop()
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
        for z, t in list(self.assumed.items()):
            if z not in remaining:
                del self.assumed[z]
            elif now - t > self.assume_wait:
                del self.assumed[z]
                self.log(f'{z} not counted after leaving it: back on the plan')
                if z not in self.deferred:
                    self.deferred.append(z)
        wanted = [z for z in remaining if z not in self.assumed]
        self.plan = [z for z in self.plan if z in wanted] + [z for z in wanted if z not in self.plan]
        if self.state in (DRIVE, DWELL) and self.t_entry is None and self.target \
                and self.field.inside(self.target, pose[0], pose[1], margin=0.03):
            self.t_entry = now

        if self.state == PICK:
            if not self.plan:
                if self.assumed:
                    return self.info()          # the refbox has not confirmed the last zones yet
                self.target = None
                self._set(DONE, now)
                self.log(f'All zones visited: {self.visited}')
                return self.info()
            self.plan = self._order(pose)
            self.tries = self.resends = 0
            self._drive_to(self.plan[0], pose, now)
            return self.info()

        if self.state == DRIVE:
            if self.target not in remaining:            # refbox already counted it
                self.nav.cancel()
                self._visited(now)
            elif nav_state == 'succeeded' and self.field.inside(self.target, pose[0], pose[1], margin=0.05):
                self.t_inside = now
                self._set(DWELL, now)
            elif nav_state == 'failed' and self._short_of_goal(pose):
                # DWB with a short look-ahead aborts the last few cm of a path ("Resulting
                # plan has 0 poses"); already in the zone, so that is arrival, not a failure
                self.log(f'{self.target}: drive ended {math.dist(pose[:2], self.goal_xy):.2f} m short of the stop point, in the zone')
                self.t_inside = now
                self._set(DWELL, now)
            elif nav_state == 'failed' and self._aborted_on_the_way(pose, now):
                # DWB also aborts in the middle of a route ("Resulting plan has 0 poses")
                # although the robot is driving fine: send the route again from here,
                # which is not a failed try
                self.resends += 1
                self.log(f'{self.target}: drive aborted on the way at ({pose[0]:.2f}, {pose[1]:.2f}), sending the route again')
                self._drive_to(self.target, pose, now)
            elif nav_state in ('failed', 'succeeded') or now - self.t_state > self.drive_timeout \
                    or (nav_state == 'active' and self._stuck(now, pose)):
                why = (('timeout' if now - self.t_state > self.drive_timeout else 'no progress')
                       if nav_state == 'active' else f'drive {nav_state}')
                self.fast_fails = self.fast_fails + 1 if nav_state == 'failed' and now - self.t_state < 2.0 else 0
                self.log(f'{self.target}: {why} at ({pose[0]:.2f}, {pose[1]:.2f})')
                self._retry(pose, now)
            return self.info()

        if self.state == BACKOFF:
            if self._backoff_step(now, pose):
                self._set(PICK, now)
            return self.info()

        if self.state == DWELL:
            if self.fine:
                self._fine_step(now, pose)
            if self.target not in remaining or self._may_leave(now):
                if self.target in remaining:
                    self.assumed[self.target] = now
                self._fine_stop()
                box = self.target in self.blocked and self.field.center(self.target)
                self._visited(now)
                if self.backoff and box and math.dist(pose[:2], box) < self.backoff['dist']:
                    self.backoff_from = box
                    self.log(f'{math.dist(pose[:2], box):.2f} m from the box: backing off before the next drive')
                    self._set(BACKOFF, now)
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
        self.retry_total = getattr(self, 'retry_total', 0) + 1
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

