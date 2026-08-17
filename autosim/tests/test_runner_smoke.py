from pathlib import Path

from autosim.config import Config
from autosim.runner import Runner
from autosim.simulator import Simulator


class FakeWriter:
    def __init__(self):
        self.msgs = []

    def write(self, msg):
        self.msgs.append(msg)


class FakeReader:
    def __init__(self):
        self.message = None
        self.has = False

    def set(self, msg):
        self.message = msg
        self.has = True

    def has_msg(self):
        return self.has

    def get_msg(self):
        self.has = False
        return self.message


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.writers = {}
        self.readers = {}

    def create_writer(self, channel, dtype, qos_depth=1):
        writer = FakeWriter()
        self.writers[channel] = writer
        return writer

    def create_reader(self, channel, dtype, qos_depth=1):
        reader = FakeReader()
        self.readers[channel] = reader
        return reader


class FakeLink:
    def __init__(self):
        self.node = None

    def init(self, name):
        return None

    def shutdown(self):
        return None

    def is_shutdown(self):
        return False

    def Node(self, name):
        self.node = FakeNode(name)
        return self.node


def test_runner_publishes_scan_and_points_when_enabled(monkeypatch):
    import autosim.runner as runner_module

    monkeypatch.setattr(runner_module.time, "sleep", lambda seconds: None)

    root = Path(__file__).resolve().parents[1]
    settings = Config.load(root / "config" / "default.yaml")
    sensors = settings.data["habitat"]["sensors"]
    sensors["lidar_2d"]["enabled"] = True
    sensors["lidar_2d"]["num_beams"] = 16
    sensors["lidar_3d"]["enabled"] = True
    sensors["lidar_3d"]["horizontal"]["num_beams"] = 8
    sensors["lidar_3d"]["vertical"]["num_rings"] = 2
    sensors["camera"]["enabled"] = False
    sensors["imu"]["enabled"] = False

    link = FakeLink()
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings=settings.data,
        use_mock=True,
    )
    runner = Runner(settings, max_steps=5, link=link, simulator=simulator)
    runner.run()
    assert "/scan" in runner.node.writers
    assert len(runner.node.writers["/scan"].msgs) >= 1
    assert "/points" in runner.node.writers
    assert len(runner.node.writers["/points"].msgs) >= 1
    assert len(runner.node.writers["/odom"].msgs) >= 1
    assert "/camera/rgb/image_raw" not in runner.node.writers
