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

"""Product-level exception hierarchy."""

from __future__ import annotations


class VNTError(Exception):
    """Base class for all vntpa errors."""


class InvalidConfigError(VNTError, ValueError):
    """Configuration is missing or internally inconsistent."""


class ModelNotFoundError(VNTError, FileNotFoundError):
    """A local planner/DeFM model could not be found."""


class ModelValidationError(VNTError, ValueError):
    """ONNX model exists but does not match the expected contract."""


class UnsupportedFormatError(VNTError, ValueError):
    """Requested export format is not implemented."""
