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

from pathlib import Path

import autosim.runtime.runner as runner_module


def test_main_acquires_instance_lock_before_runner_construction(
    monkeypatch, tmp_path: Path
) -> None:
    events = []
    config_path = tmp_path / "autosim.yaml"
    settings = object()

    class FakeLock:
        @staticmethod
        def from_environment(received_path):
            events.append(("lock_factory", received_path))
            return FakeLock()

        def __enter__(self):
            events.append(("lock_enter", None))
            return self

        def __exit__(self, _exc_type, _exc_value, _traceback):
            events.append(("lock_exit", None))
            return False

    def load(received_path):
        events.append(("config_load", received_path))
        return settings

    def construct(self, received_settings):
        events.append(("runner_construct", received_settings))

    def run(self):
        events.append(("runner_run", None))

    monkeypatch.setattr(runner_module, "InstanceLock", FakeLock)
    monkeypatch.setattr(runner_module.Config, "load", staticmethod(load))
    monkeypatch.setattr(runner_module.Runner, "__init__", construct)
    monkeypatch.setattr(runner_module.Runner, "run", run)

    runner_module.Runner.main(["--config", str(config_path)])

    assert events == [
        ("config_load", config_path),
        ("lock_factory", config_path),
        ("lock_enter", None),
        ("runner_construct", settings),
        ("runner_run", None),
        ("lock_exit", None),
    ]
