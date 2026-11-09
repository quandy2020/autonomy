# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json

from vntpa.engine import TrainConfig, build_jsonl_end_hook, build_jsonl_start_hook


def test_train_command():
    command = " ".join(TrainConfig(["--steps", "10"]).build_command())
    assert "thirdparty.pan.planner.train" in command


def test_jsonl_hooks(tmp_path):
    config = TrainConfig(["--steps", "10"])
    log = tmp_path / "run.jsonl"
    build_jsonl_start_hook(str(log))(config)
    build_jsonl_end_hook(str(log))(config, 0)
    events = [json.loads(line) for line in log.read_text().splitlines()]
    assert [event["event"] for event in events] == ["start", "end"]
