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

"""Dataset loaders that publish sensor streams over Autolink."""

from __future__ import annotations

__all__ = ["DatasetPublisher", "SpiresReader", "VisualDatasetPublisher"]


def __getattr__(name: str):
    if name == "DatasetPublisher":
        from autonomy.tools.datasets.publisher import DatasetPublisher

        return DatasetPublisher
    if name == "SpiresReader":
        from autonomy.tools.datasets.readers.spires import SpiresReader

        return SpiresReader
    if name == "VisualDatasetPublisher":
        from autonomy.tools.datasets.visual_publisher import VisualDatasetPublisher

        return VisualDatasetPublisher
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
