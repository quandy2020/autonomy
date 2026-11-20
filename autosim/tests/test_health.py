"""Health counters, worker drop accounting, and ``/autosim/status`` rows."""

import threading

import pytest

from autosim.runtime.health import HealthMonitor
from autosim.runtime.runner import Runner
from autosim.runtime.worker import SensorWorker


def test_stream_counters_track_drops_latency_and_rate():
    monitor = HealthMonitor()
    entry = monitor.stream("camera")
    entry.record_submit()
    entry.record_drop()
    entry.record(4.0, now=1.0)
    entry.record(6.0, now=1.5)
    assert entry.submitted == 1
    assert entry.dropped == 1
    assert entry.completed == 2
    assert entry.mean_ms() == pytest.approx(5.0)
    assert entry.max_ms == pytest.approx(6.0)
    assert entry.rate_hz == pytest.approx(2.0)
    assert entry.level() == "WARN"


def test_stream_level_reports_errors_before_drops():
    monitor = HealthMonitor()
    entry = monitor.stream("scan")
    entry.record_drop()
    entry.record_error(RuntimeError("encode failed"))
    assert entry.level() == "ERROR"
    assert "encode failed" in entry.last_error


def test_health_rows_include_control_loop():
    monitor = HealthMonitor()
    monitor.stream("odom").record(1.0, now=0.0)
    monitor.record_cycle(30.0, 20.0)
    rows = monitor.rows(hardware_id="google_robot")
    names = [row["name"] for row in rows]
    assert names == ["autosim/odom", "autosim/control"]
    control = rows[-1]
    assert control["level"] == "WARN"
    assert control["values"]["overruns"] == 1
    assert control["values"]["max_cycle_ms"] == pytest.approx(30.0)
    assert rows[0]["hardware_id"] == "google_robot"


def test_worker_counts_a_replaced_frame():
    monitor = HealthMonitor()
    worker = SensorWorker(monitor)
    worker.submit(lambda: None, label="camera")
    worker.submit(lambda: None, label="camera")
    assert worker.submitted == 2
    assert worker.dropped == 1
    assert monitor.stream("camera").dropped == 1


def test_worker_records_completion_and_error():
    monitor = HealthMonitor()
    worker = SensorWorker(monitor)
    worker.start()
    done = threading.Event()
    try:
        worker.submit(done.set, label="scan")
        assert done.wait(2.0)
        worker.submit(lambda: (_ for _ in ()).throw(RuntimeError("boom")), label="rgb")
        for _ in range(200):
            if monitor.stream("rgb").errors:
                break
            threading.Event().wait(0.01)
    finally:
        worker.stop()
    assert monitor.stream("scan").completed == 1
    assert monitor.stream("rgb").errors == 1
    assert "boom" in monitor.stream("rgb").last_error


def test_emit_leaves_async_completion_to_the_worker():
    """Queued streams must not be counted once by emit and again by the worker."""
    runner = object.__new__(Runner)
    runner.health = HealthMonitor()
    runner.emit("points", lambda: None)
    assert runner.health.stream("points").completed == 0
    runner.emit("odom", lambda: None)
    assert runner.health.stream("odom").completed == 1
