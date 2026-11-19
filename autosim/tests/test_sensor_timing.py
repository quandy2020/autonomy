# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Sensor latency / jitter model: parsing, sampling and the delay queue."""

import random
from pathlib import Path

import pytest

from autosim.bridge.timing import DelayQueue, SensorTiming
from autosim.runtime import Config
from autosim.runtime import Runner
from tests.fake_simulator import FakeSimulator

ROOT = Path(__file__).resolve().parents[1]


def test_missing_keys_mean_no_delay():
    timing = SensorTiming.from_block({"enabled": True}, "habitat.sensors.camera")
    assert timing.latency_sec == 0.0
    assert timing.jitter_sec == 0.0
    assert not timing.enabled
    assert timing.delay() == 0.0
    assert SensorTiming.from_block(None, "habitat.sensors.camera") == SensorTiming()


@pytest.mark.parametrize(
    "block",
    [
        {"latency_sec": -0.1},
        {"latency_sec": 5.0},
        {"jitter_sec": -0.01},
        {"jitter_sec": 2.0},
        {"latency_sec": float("nan")},
    ],
)
def test_bad_timing_is_rejected(block):
    with pytest.raises(ValueError):
        SensorTiming.from_block(block, "habitat.sensors.imu")


def test_fixed_latency_is_exact():
    timing = SensorTiming(latency_sec=0.03)
    assert timing.enabled
    assert timing.delay(random.Random(1)) == pytest.approx(0.03)


def test_jitter_stays_inside_the_window_and_never_goes_negative():
    """Jitter alone must not deliver a frame before it was captured."""
    timing = SensorTiming(latency_sec=0.01, jitter_sec=0.02)
    rng = random.Random(7)
    delays = [timing.delay(rng) for _ in range(200)]
    assert all(0.0 <= value <= 0.03 for value in delays)
    assert min(delays) == 0.0  # the clamp is reachable
    assert max(delays) > 0.02  # and the window is actually used


def test_delay_queue_releases_in_time_then_arrival_order():
    queue = DelayQueue()
    queue.push(0.20, "late", "c")
    queue.push(0.10, "early", "a")
    queue.push(0.10, "early", "b")
    assert len(queue) == 3
    assert queue.pop_ready(0.05) == []
    assert queue.pop_ready(0.10) == [("early", "a"), ("early", "b")]
    assert len(queue) == 1
    assert queue.pop_ready(0.25) == [("late", "c")]
    assert len(queue) == 0


class _Writer:
    def __init__(self):
        self.msgs = []

    def write(self, msg):
        self.msgs.append(msg)


class _Reader:
    def has_msg(self):
        return False

    def get_msg(self):
        return None


class _Node:
    def __init__(self):
        self.writers = {}

    def create_writer(self, channel, dtype, qos_depth=1):
        writer = _Writer()
        self.writers[channel] = writer
        return writer

    def create_reader(self, channel, dtype, qos_depth=1):
        return _Reader()


class _Link:
    def __init__(self):
        self.node = None

    def init(self, name):
        return None

    def shutdown(self):
        return None

    def is_shutdown(self):
        return False

    def Node(self, name):
        self.node = _Node()
        return self.node


def _runner() -> Runner:
    settings = Config.load(ROOT / "config" / "default.yaml")
    settings.data["habitat"]["backend"] = "habitat"
    sensors = settings.data["habitat"]["sensors"]
    sensors["camera"]["enabled"] = True
    sensors["lidar_2d"]["enabled"] = False
    sensors["lidar_3d"]["enabled"] = False
    sensors["imu"]["enabled"] = False
    sensors["odom"]["enabled"] = True
    settings.data["habitat"]["mode"] = "nav"
    settings.data["habitat"]["map"]["enabled"] = False
    return Runner(settings, max_steps=1, link=_Link(), simulator=FakeSimulator())


def test_runner_holds_camera_frames_until_the_latency_elapses():
    runner = _runner()
    runner.timings["camera"] = SensorTiming(latency_sec=0.05)
    channel = runner.camera["rgb_channel"]

    runner.publish("rgb", "frame", sensor="camera")
    assert runner.node.writers[channel].msgs == []
    assert len(runner.delay_queue) == 1

    runner.clock.tick(0.02)
    runner.flush_delayed()
    assert runner.node.writers[channel].msgs == []

    runner.clock.tick(0.04)
    runner.flush_delayed()
    assert runner.node.writers[channel].msgs == ["frame"]
    assert len(runner.delay_queue) == 0


def test_runner_publishes_immediately_without_timing():
    runner = _runner()
    channel = runner.camera["rgb_channel"]
    runner.publish("rgb", "frame", sensor="camera")
    assert runner.node.writers[channel].msgs == ["frame"]
    assert len(runner.delay_queue) == 0
