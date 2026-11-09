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

from vntpa.engine import TrainConfig, build_checkpoint_hook


def test_checkpoint_hook_noop_without_file(tmp_path):
    hook = build_checkpoint_hook(str(tmp_path / "missing.pt"), str(tmp_path / "out.pt"))
    hook(TrainConfig(), 0)
    assert not (tmp_path / "out.pt").exists()
