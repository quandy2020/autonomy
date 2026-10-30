# Copyright 2026 The Openbot Authors (duyongquan)
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

"""Dataset readers for autonomy.tools.datasets."""

from autonomy.tools.datasets.readers.euroc import EurocReader, TumViReader
from autonomy.tools.datasets.readers.kitti import KittiReader
from autonomy.tools.datasets.readers.spires import ImuSample, SpiresFrame, SpiresReader
from autonomy.tools.datasets.readers.tum_rgbd import TumRgbdReader
from autonomy.tools.datasets.readers.visual import (
    ImageSample,
    ImuSample as VisualImuSample,
    VisualFrame,
)

__all__ = [
    "EurocReader",
    "ImageSample",
    "ImuSample",
    "KittiReader",
    "SpiresFrame",
    "SpiresReader",
    "TumRgbdReader",
    "TumViReader",
    "VisualFrame",
    "VisualImuSample",
]
