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

import json
import os
from pathlib import Path

import pytest

from autosim.runtime.instance_lock import InstanceLock, InstanceLockError


def test_rejects_second_holder_and_reports_owner(tmp_path: Path) -> None:
    path = tmp_path / "autosim.lock"
    first_config = tmp_path / "first.yaml"
    second_config = tmp_path / "second.yaml"

    with InstanceLock(path, first_config):
        with pytest.raises(InstanceLockError, match="already running") as caught:
            InstanceLock(path, second_config).acquire()

    message = str(caught.value)
    assert f"pid={os.getpid()}" in message
    assert f"config={first_config}" in message


def test_release_allows_next_holder_and_replaces_metadata(tmp_path: Path) -> None:
    path = tmp_path / "autosim.lock"
    first_config = tmp_path / "first.yaml"
    second_config = tmp_path / "second.yaml"

    with InstanceLock(path, first_config):
        first_metadata = json.loads(path.read_text(encoding="utf-8"))
        assert first_metadata["pid"] == os.getpid()
        assert first_metadata["config"] == str(first_config)
        assert first_metadata["started_at_unix_sec"] > 0.0

    with InstanceLock(path, second_config):
        second_metadata = json.loads(path.read_text(encoding="utf-8"))
        assert second_metadata["config"] == str(second_config)


def test_override_paths_allow_independent_domains(tmp_path: Path) -> None:
    first = InstanceLock(tmp_path / "first.lock", tmp_path / "first.yaml")
    second = InstanceLock(tmp_path / "second.lock", tmp_path / "second.yaml")

    with first, second:
        first_metadata = json.loads(first.path.read_text(encoding="utf-8"))
        second_metadata = json.loads(second.path.read_text(encoding="utf-8"))
        assert first_metadata["pid"] == os.getpid()
        assert second_metadata["pid"] == os.getpid()
