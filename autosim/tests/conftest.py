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

"""Test bootstrap: tolerate protobuf gencode/runtime version skew."""
from __future__ import annotations

try:
    from google.protobuf import runtime_version as _runtime_version

    def _allow_gencode(*_args, **_kwargs):
        return None

    _runtime_version.ValidateProtobufRuntimeVersion = _allow_gencode  # type: ignore[method-assign]
except Exception:
    pass
