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

"""Named robot hardware profiles (URDF + USD pairing)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

from autosim.robot.urdf import UrdfModel


@dataclass(frozen=True)
class RobotProfile:
    """URDF / USD / kinematic defaults for one robot model."""

    name: str
    urdf: str
    usd_robot: str
    wheel_separation: float
    base_link_height: float
    sensor_frames: Mapping[str, str]
    usd_prim: str = "/World/Robot"
    articulated: bool = False
    root_height: float = 0.0
    max_linear: float = 0.5
    max_angular: float = 1.0


PROFILES: Dict[str, RobotProfile] = {
    "turtlebot3_burger": RobotProfile(
        name="turtlebot3_burger",
        urdf="robot/turtlebot3_burger/turtlebot3_burger.urdf",
        usd_robot="usd/robots/turtlebot3_burger/turtlebot3_burger.usd",
        wheel_separation=0.160,
        base_link_height=0.01,
        sensor_frames={
            "laser": "base_scan",
            "imu": "imu_link",
            "camera": "camera_link",
        },
        usd_prim="/World/Turtlebot3",
    ),
    "husky": RobotProfile(
        name="husky",
        urdf="robot/husky/husky.urdf",
        usd_robot="usd/robots/husky/husky.usd",
        wheel_separation=0.512,
        base_link_height=0.1651,
        sensor_frames={
            "laser": "laser_link",
            "imu": "imu_link",
            "camera": "camera_link",
        },
        usd_prim="/World/Husky",
    ),
}

# URDF-only Habitat models; these do not claim to provide Isaac USD assets.
# Heights place the lowest visual point on the floor for the URDF home pose:
# Google wheel radius, G1 ankle sole, and Go2 foot sole respectively.
_ARTICULATED = {
    "google_robot": 0.1975,
    "unitree_g1": 0.7162,
    "unitree_go2": 0.3848,
}

for _name, _height in _ARTICULATED.items():
    PROFILES[_name] = RobotProfile(
        name=_name, urdf=f"robot/{_name}/{_name}.urdf", usd_robot="",
        wheel_separation=0.5, base_link_height=_height,
        sensor_frames={}, articulated=True, root_height=_height,
    )


def get_profile(name: str) -> RobotProfile:
    """Return a registered profile.

    Raises:
        KeyError: Unknown profile name.
    """
    key = str(name or "").strip().lower()
    if key not in PROFILES:
        known = ", ".join(sorted(PROFILES))
        raise KeyError(f"unknown robot profile {name!r}; known: {known}")
    return PROFILES[key]


def apply_profile(robot_cfg: Mapping[str, Any]) -> Dict[str, Any]:
    """Merge ``robot.profile`` defaults into a robot config dict.

    Explicit keys in ``robot_cfg`` win over profile defaults.
    """
    out = dict(robot_cfg)
    name = str(out.get("profile") or "").strip()
    if not name:
        return out
    profile = get_profile(name)
    out.setdefault("urdf", profile.urdf)
    out.setdefault("usd_robot", profile.usd_robot)
    out.setdefault("usd_prim", profile.usd_prim)
    out.setdefault("wheel_separation", profile.wheel_separation)
    out.setdefault("base_link_height", profile.base_link_height)
    out.setdefault("sensor_frames", dict(profile.sensor_frames))
    if profile.articulated:
        out.setdefault("instance", True)
        out.setdefault("fixed_base", True)
        out.setdefault("root_height", profile.root_height)
        out.setdefault("max_linear", profile.max_linear)
        out.setdefault("max_angular", profile.max_angular)
    return out


def resolve_path(relative: str) -> Path:
    """Resolve a path relative to the autosim project root."""
    path = Path(str(relative or "").strip())
    if not path.is_absolute():
        path = UrdfModel.package_root() / path
    return path.resolve()


def resolve_usd_paths(
    habitat: Mapping[str, Any], robot_cfg: Mapping[str, Any]
) -> Tuple[Path, Optional[Path]]:
    """Resolve world + optional robot USD paths for Isaac.

    World path: ``habitat.path`` or ``habitat.usd.world``.
    Robot path: ``robot.usd_robot`` or profile default.
    """
    usd_block = habitat.get("usd") if isinstance(habitat.get("usd"), Mapping) else {}
    world = str(
        habitat.get("path") or (usd_block or {}).get("world") or ""
    ).strip()
    if not world:
        raise ValueError(
            "habitat.backend=isaac requires habitat.path or habitat.usd.world"
        )
    world_path = resolve_path(world)
    robot_rel = str(
        robot_cfg.get("usd_robot") or (usd_block or {}).get("robot") or ""
    ).strip()
    robot_path = resolve_path(robot_rel) if robot_rel else None
    return world_path, robot_path
