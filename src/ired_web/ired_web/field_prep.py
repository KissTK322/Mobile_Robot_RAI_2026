"""One button on the Game page: get the robot ready for a game with the field refbox.

Does, in the order that worked on the robot (2026-10-27): Robot -> sensors talking -> a Navigation
profile -> Nav2 really active (the lifecycle can hang half-configured: then Navigation is restarted
once) -> (Camera for exploration) -> the challenge program -> start pose C_Z51 sent to AMCL AFTER the
challenge runs (mqtt_refbox only forwards AMCL poses it receives, an earlier start pose never reaches
the refbox). The robot does not move: it waits for the refbox to start the game.

`hub` is the web manager (see Manager.field_prep_hub); tests pass a fake."""
import math
import threading
import time

STEPS = [('estop', 'STOP released'), ('robot', 'Robot mode + wheel board + LiDAR'),
         ('nav', 'Navigation active (Nav2)'), ('camera', 'Camera + ArUco'),
         ('challenge', 'Challenge program'), ('pose', 'Start pose C_Z51 (robot on C_Z51 facing C_Z41)')]
START = (4.5, 0.5, math.pi)


CHALLENGES = {'navigation': 'challenge_nav', 'exploration': 'challenge_exp', 'exploration_lidar': 'challenge_exp_lidar'}


class FieldPrep:
    def __init__(self, hub, clock=time.monotonic, sleep=time.sleep, nav_profile='navigation_quick',
                 robot_timeout=30.0, nav_timeout=60.0, challenge_timeout=40.0):
        self.hub, self.clock, self.sleep = hub, clock, sleep
        self.nav_profile = nav_profile
        self.robot_timeout, self.nav_timeout, self.challenge_timeout = robot_timeout, nav_timeout, challenge_timeout
        self.lock = threading.Lock()
        self.busy = False
        self.mode = None
        self.steps = []
        self.error = ''
        self.done = False

    def status(self):
        with self.lock:
            return {'busy': self.busy, 'mode': self.mode, 'done': self.done, 'error': self.error,
                    'steps': [dict(s) for s in self.steps]}

    def start(self, mode):
        if mode not in CHALLENGES:
            raise ValueError('Mode must be navigation, exploration or exploration_lidar')
        with self.lock:
            if self.busy:
                raise ValueError('Already preparing')
            self.busy, self.mode, self.done, self.error = True, mode, False, ''
            self.steps = [{'key': k, 'label': l, 'state': 'pending', 'msg': ''} for k, l in STEPS
                          if k != 'camera' or mode != 'navigation']
        threading.Thread(target=self.run, daemon=True).start()
        return self.status()

    # ------------------------------------------------------------------ steps
    def _set(self, key, state, msg=''):
        with self.lock:
            for s in self.steps:
                if s['key'] == key:
                    s['state'], s['msg'] = state, msg

    def _wait(self, test, timeout, every=0.5):
        end = self.clock() + timeout
        while True:
            if test():
                return True
            if self.clock() >= end:
                return False
            self.sleep(every)

    def run(self):
        h = self.hub
        try:
            if h.source() != 'field':
                raise RuntimeError('Game source is Practice: choose Field refbox and press "Use selected source" first')
            self._set('estop', 'active')
            if h.estop():
                raise RuntimeError('STOP is on: release it (top bar)')
            self._set('estop', 'ok')

            self._set('robot', 'active', 'starting' if not h.running('robot') else 'checking')
            if not h.running('robot'):
                h.start('robot')
            ok = self._wait(lambda: h.rates().get('/ired/motor/data', 0) >= 5 and h.rates().get('/scan_filtered', 0) >= 6,
                            self.robot_timeout)
            r = h.rates()
            if not ok:
                raise RuntimeError(f"Robot not talking: wheels {r.get('/ired/motor/data', 0):.0f} Hz, LiDAR "
                                   f"{r.get('/scan_filtered', 0):.1f} Hz (drive switch on? USB? Stop and Start Robot)")
            self._set('robot', 'ok', f"wheels {r.get('/ired/motor/data', 0):.0f} Hz, LiDAR {r.get('/scan_filtered', 0):.1f} Hz")

            nav = h.running_nav()
            self._set('nav', 'active', f'starting {self.nav_profile}' if not nav else f'checking {nav}')
            if not nav:
                nav = self.nav_profile
                h.start(nav)
            if not self._wait(h.nav2_active, self.nav_timeout, 1.0):
                # seen on the robot: controller_server stayed unconfigured; a restart fixed it
                self._set('nav', 'active', f'{nav} did not become active: restarting it once')
                h.stop(nav)
                self.sleep(3.0)
                h.start(nav)
                if not self._wait(h.nav2_active, self.nav_timeout, 1.0):
                    raise RuntimeError(f'{nav} did not become active twice: check its log on the Launch page')
            self._set('nav', 'ok', f'{nav} active')

            if self.mode != 'navigation':
                self._set('camera', 'active', 'starting')
                if not h.running('camera'):
                    h.start('camera')
                if not self._wait(lambda: h.running('camera'), 15.0):
                    raise RuntimeError('Camera + ArUco did not start: check its log on the Launch page')
                self._set('camera', 'ok', 'running')

            chal = CHALLENGES[self.mode]
            self._set('challenge', 'active', 'starting' if not h.running(chal) else 'already running')
            if not h.running(chal):
                h.start(chal)
            if not self._wait(lambda: h.challenge_ready(self.mode), self.challenge_timeout):
                raise RuntimeError('Challenge program gives no state: check its log on the Launch page')
            self._set('challenge', 'ok', 'waiting for the refbox to start the game')

            self._set('pose', 'active', 'sending')
            h.send_start_pose(*START)
            ok = self._wait(lambda: self._near_start(h.pose()), 8.0)
            p = h.pose()
            if not ok:
                raise RuntimeError('AMCL does not report C_Z51 after the start pose: is the robot on C_Z51 facing C_Z41?'
                                   + (f' (now {p[0]:.2f}, {p[1]:.2f})' if p else ''))
            self._set('pose', 'ok', f'({p[0]:.2f}, {p[1]:.2f})')
            with self.lock:
                self.done = True
        except Exception as e:                          # report, never leave the button stuck
            with self.lock:
                self.error = str(e)
                for s in self.steps:
                    if s['state'] == 'active':
                        s['state'], s['msg'] = 'bad', str(e)
        finally:
            with self.lock:
                self.busy = False

    @staticmethod
    def _near_start(p):
        if p is None:
            return False
        dyaw = abs(math.atan2(math.sin(p[2] - START[2]), math.cos(p[2] - START[2])))
        return math.hypot(p[0] - START[0], p[1] - START[1]) <= 0.15 and dyaw <= math.radians(15)
