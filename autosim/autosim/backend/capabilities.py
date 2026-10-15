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

"""Optional WorldBackend capability flags (Runner must not branch on backend name).

Backends declare a ``capabilities`` mapping or individual ``supports_*`` attributes.
Unknown capabilities fall back to the defaults below (Habitat / Isaac-like).
"""

from __future__ import annotations

from typing import Any, Mapping

# Defaults match Habitat: surround cameras, navmesh map sample, URDF TF.
# serialize_sensors: one heavy sensor sample per control tick (Habitat GL).
CAPABILITY_DEFAULTS: dict[str, bool] = {
    "surround": True,
    "panorama": True,
    "navmesh_map_sample": True,
    "fallback_camera_tf": False,
    "serialize_sensors": True,
}

# Procedural fake world: no surround / panorama; map owned by FakeMap when enabled.
FAKE_CAPABILITIES: dict[str, bool] = {
    "surround": False,
    "panorama": False,
    "navmesh_map_sample": False,
    "fallback_camera_tf": True,
    "serialize_sensors": False,
}

# Isaac: /scan is cheap CPU cast; RGB needs every due tick (do not starve for lidar).
ISAAC_CAPABILITIES: dict[str, bool] = {
    "surround": True,
    "panorama": True,
    "navmesh_map_sample": True,
    "fallback_camera_tf": False,
    "serialize_sensors": False,
}


def capability(backend: Any, name: str, default: bool | None = None) -> bool:
    """Read a named capability from ``backend``.

    Lookup order:
    1. ``backend.capabilities[name]``
    2. ``backend.supports_<name>``
    3. ``default`` argument, else package default for ``name``, else ``False``
    """
    caps = getattr(backend, "capabilities", None)
    if isinstance(caps, Mapping) and name in caps:
        return bool(caps[name])
    attr = getattr(backend, f"supports_{name}", None)
    if attr is not None:
        return bool(attr)
    if default is not None:
        return bool(default)
    return bool(CAPABILITY_DEFAULTS.get(name, False))
