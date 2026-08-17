from autosim.clock import SimClock


def test_tick_advances_and_stamp():
    clock = SimClock(start_sec=0.0)
    clock.tick(0.02)
    sec, nanosec = clock.stamp()
    assert sec == 0
    assert nanosec == 20_000_000
    assert abs(clock.now() - 0.02) < 1e-9


def test_stamp_rolls_seconds():
    clock = SimClock(start_sec=0.0)
    clock.tick(1.5)
    sec, nanosec = clock.stamp()
    assert sec == 1
    assert nanosec == 500_000_000
