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

"""Imitation-learning data collection (LeRobot v2.1).

Pipeline::

    Runner.cycle → DatasetCollector.tick → Episode → LeRobotWriter
                                                      ├─ data/**/*.parquet
                                                      ├─ videos/**/*.mp4
                                                      └─ meta/{info,episodes,tasks}
"""

from autosim.dataset.collector import DatasetCollector
from autosim.dataset.episode import Episode
from autosim.dataset.frame import Frame
from autosim.dataset.lerobot import LeRobotWriter
from autosim.dataset.schema import config_from_habitat, feature_dict

__all__ = [
    "DatasetCollector",
    "Episode",
    "Frame",
    "LeRobotWriter",
    "config_from_habitat",
    "feature_dict",
]
