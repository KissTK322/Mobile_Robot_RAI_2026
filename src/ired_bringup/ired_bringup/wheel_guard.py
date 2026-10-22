"""Wheel runaway guard for the modbus driver.

The wheel PID runs on the micro board. If a wheel's encoder stops counting (seen on
2026-10-20: rear-left speed_fb 0.0 while commanded), the board's PID winds up and
drives that wheel at full speed. The guard watches the setpoints and the feedback the
driver already has: a wheel commanded faster than `min_cmd` whose feedback stays below
`zero_fb` for longer than `hold` seconds trips it. Once tripped all four setpoints are
sent as zero until reset() (a service in the driver), so the robot stops instead of
spinning off. A blocked wheel (robot pushing a wall) trips it too, which is also a
reason to stop. A wheel whose feedback is far above its command (runaway with a counting
encoder) trips it the same way.

Feedback is only checked on cycles where the modbus read succeeded; a failed read sets
the feedback to zero in the driver and must not count as a dead encoder."""


class WheelGuard:
    NAMES = ('FL', 'FR', 'RL', 'RR')

    def __init__(self, min_cmd=10.0, zero_fb=0.5, hold=0.5, enabled=True, over_ratio=2.0, over_margin=20.0):
        self.min_cmd = min_cmd      # rpm: smaller commands are ignored (start-up, creeping)
        self.zero_fb = zero_fb      # rpm: feedback below this counts as "not turning"
        self.hold = hold            # s: how long a commanded wheel may read zero / run away
        self.enabled = enabled
        # overspeed: |feedback| > over_ratio * |command| + over_margin (2026-10-22: FL read
        # -44..-72 rpm for a -10 rpm command with a counting encoder); <= 0 turns it off
        self.over_ratio = over_ratio
        self.over_margin = over_margin
        self.since = [None] * 4     # time each wheel started reading zero while commanded
        self.over_since = [None] * 4
        self.tripped = None         # None or a text saying why

    def update(self, sp, fb, now):
        """sp, fb: 4 speeds (rpm) from the same cycle, now: seconds. Returns the trip
        text the first time it trips, else None."""
        if not self.enabled or self.tripped:
            return None
        for i in range(4):
            if abs(sp[i]) >= self.min_cmd and abs(fb[i]) < self.zero_fb:
                if self.since[i] is None:
                    self.since[i] = now
                elif now - self.since[i] > self.hold:
                    self.tripped = (f'{self.NAMES[i]} wheel commanded {sp[i]:.0f} rpm but its encoder read '
                                    f'{fb[i]:.1f} rpm for {now - self.since[i]:.1f} s: all wheels stopped '
                                    '(encoder cable? wheel blocked?). Call /ired/wheel_guard/reset to drive again.')
                    return self.tripped
            else:
                self.since[i] = None
            if self.over_ratio > 0 and abs(fb[i]) > self.over_ratio * abs(sp[i]) + self.over_margin:
                if self.over_since[i] is None:
                    self.over_since[i] = now
                elif now - self.over_since[i] > self.hold:
                    self.tripped = (f'{self.NAMES[i]} wheel runs at {fb[i]:.0f} rpm but was commanded '
                                    f'{sp[i]:.0f} rpm for {now - self.over_since[i]:.1f} s: all wheels stopped '
                                    '(encoder or motor fault?). Call /ired/wheel_guard/reset to drive again.')
                    return self.tripped
            else:
                self.over_since[i] = None
        return None

    def output(self, sp):
        """Setpoints to send: zeros while tripped."""
        return [0.0] * 4 if self.tripped else list(sp)

    def reset(self):
        self.tripped = None
        self.since = [None] * 4
        self.over_since = [None] * 4
