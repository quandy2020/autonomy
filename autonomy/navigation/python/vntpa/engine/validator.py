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

"""Aggregate evaluation result."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class EvalResult:
    episodes: int = 0
    successes: int = 0
    spl_values: list[float] = field(default_factory=list)

    def rate(self) -> float:
        return self.successes / self.episodes if self.episodes else 0.0

    def spl(self) -> float:
        return sum(self.spl_values) / len(self.spl_values) if self.spl_values else 0.0
