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

"""TensorRT export placeholder with an explicit capability error."""

from __future__ import annotations

from vntpa.utils.errors import UnsupportedFormatError


def tensorrt(*args, **kwargs) -> None:
    raise UnsupportedFormatError(
        "TensorRT export is not implemented; use ONNX and build the engine externally"
    )
