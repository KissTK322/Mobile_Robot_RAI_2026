from ired_bringup.wheel_guard import WheelGuard


def run(g, steps, dt=0.1):
    """steps: list of (sp, fb); returns the first trip text and its step index."""
    for k, (sp, fb) in enumerate(steps):
        trip = g.update(sp, fb, k * dt)
        if trip:
            return trip, k
    return None, None


def test_dead_encoder_trips_after_hold():
    g = WheelGuard()
    steps = [([60, 60, 60, 60], [60, 60, 0.0, 60])] * 10
    trip, k = run(g, steps)
    assert trip.startswith('RL wheel') and k == 6          # 0.6 s > 0.5 s
    assert g.output([60, 60, 60, 60]) == [0.0] * 4


def test_start_up_lag_does_not_trip():
    # feedback zero for the first two cycles after a step, as seen on the robot
    g = WheelGuard()
    steps = [([60] * 4, [0.0] * 4)] * 2 + [([60] * 4, [55.0] * 4)] * 20
    assert run(g, steps) == (None, None)


def test_small_commands_and_stops_ignored():
    g = WheelGuard()
    steps = [([5, -5, 5, -5], [0.0] * 4)] * 20 + [([0] * 4, [0.0] * 4)] * 20
    assert run(g, steps) == (None, None)


def test_reverse_command_dead_encoder():
    g = WheelGuard()
    trip, _ = run(g, [([-40, 0, 0, 0], [0.2, 0, 0, 0])] * 10)
    assert trip.startswith('FL wheel')


def test_reset_and_disabled():
    g = WheelGuard()
    run(g, [([60] * 4, [0.0] * 4)] * 10)
    assert g.tripped
    g.reset()
    assert not g.tripped and g.output([1, 2, 3, 4]) == [1, 2, 3, 4]
    off = WheelGuard(enabled=False)
    assert run(off, [([60] * 4, [0.0] * 4)] * 20) == (None, None)
