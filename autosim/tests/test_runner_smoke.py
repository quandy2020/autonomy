from pathlib import Path

from autosim.config import load_config
from autosim.runner import BridgeRunner


class FakeWriter:
    def __init__(self):
        self.msgs = []

    def write(self, msg):
        self.msgs.append(msg)


class FakeReader:
    def __init__(self):
        self._msg = None
        self._has = False

    def set(self, msg):
        self._msg = msg
        self._has = True

    def has_msg(self):
        return self._has

    def get_msg(self):
        self._has = False
        return self._msg


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.writers = {}
        self.readers = {}

    def create_writer(self, channel, dtype, qos_depth=1):
        w = FakeWriter()
        self.writers[channel] = w
        return w

    def create_reader(self, channel, dtype, qos_depth=1):
        r = FakeReader()
        self.readers[channel] = r
        return r


def test_runner_publishes_with_mock_world(monkeypatch):
    import autosim.runner as runner_mod

    monkeypatch.setattr(runner_mod, "autolink", type("AL", (), {
        "init": staticmethod(lambda name: None),
        "shutdown": staticmethod(lambda: None),
        "is_shutdown": staticmethod(lambda: False),
        "Node": FakeNode,
    }))

    root = Path(__file__).resolve().parents[1]
    cfg = load_config(root / "config" / "default.yaml")
    cfg["scene"]["backend"] = "minimal"
    # Force mock path: monkeypatch create_world
    from autosim.world import MockWorld

    monkeypatch.setattr(runner_mod, "create_world", lambda c: MockWorld(64, 48))

    runner = BridgeRunner(cfg, max_steps=5)
    runner.run()
    assert "/scan" in runner.node.writers
    assert len(runner.node.writers["/scan"].msgs) >= 1
    assert len(runner.node.writers["/odom"].msgs) >= 1
