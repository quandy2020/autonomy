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

from autosim.clock import Clock


def test_tick_advances_and_stamp():
    clock = Clock(start_sec=0.0)
    clock.tick(0.02)
    sec, nanosec = clock.stamp()
    assert sec == 0
    assert nanosec == 20_000_000
    assert abs(clock.now() - 0.02) < 1e-9


def test_stamp_rolls_seconds():
    clock = Clock(start_sec=0.0)
    clock.tick(1.5)
    sec, nanosec = clock.stamp()
    assert sec == 1
    assert nanosec == 500_000_000
