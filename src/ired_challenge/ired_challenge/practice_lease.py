"""Opt-in practice heartbeat lease; loss latches until the challenge is restarted."""
import re
import time


class PracticeLease:
    def __init__(self, enabled=False, timeout=2.0, clock=time.monotonic):
        self.enabled, self.timeout, self.clock = enabled, timeout, clock
        self.token = None
        self.last = None
        self.tripped = False

    def heartbeat(self, token):
        if not re.fullmatch(r'[0-9a-f]{32}', token) or self.tripped:
            return
        if self.token is not None and token != self.token:
            self.tripped = True
            return
        self.token, self.last = token, self.clock()

    def allowed(self, requested):
        if not self.enabled or not requested:
            return requested
        if self.last is None or self.clock()-self.last > self.timeout:
            self.tripped = True
        return not self.tripped
