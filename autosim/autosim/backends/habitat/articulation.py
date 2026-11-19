"""Named kinematic control using Habitat's actual joint-position offsets."""
from __future__ import annotations

from typing import Mapping, Any
import numpy as np

from autosim.robot.joints import Joint, complete_positions


class Articulation:
    def __init__(self, robot: Any, joints: Mapping[str, Joint]) -> None:
        self.robot = robot
        self.joints = dict(joints)
        self.offsets = {}
        self.velocity_offsets = {}
        for link_id in robot.get_link_ids():
            if robot.get_link_num_joint_pos(link_id) == 0:
                continue
            name = robot.get_link_joint_name(link_id)
            if name not in self.joints or robot.get_link_num_joint_pos(link_id) != 1:
                raise ValueError(f"unsupported Habitat joint: {name}")
            self.offsets[name] = robot.get_link_joint_pos_offset(link_id)
            self.velocity_offsets[name] = robot.get_link_dof_offset(link_id)
        missing = set(self.joints) - set(self.offsets)
        if missing:
            raise ValueError(f"URDF joints missing from Habitat: {sorted(missing)}")
        self.set_positions({name: j.home() for name, j in self.joints.items() if not j.mimic})

    def positions(self) -> dict[str, float]:
        values = self.robot.joint_positions
        return {name: float(values[offset]) for name, offset in self.offsets.items()}

    def velocities(self) -> dict[str, float]:
        values = self.robot.joint_velocities
        return {name: float(values[offset]) for name, offset in self.velocity_offsets.items()}

    def set_positions(self, updates: Mapping[str, float]) -> None:
        for name in updates:
            if name not in self.joints:
                raise ValueError(f"unknown joint: {name}")
            if self.joints[name].mimic:
                raise ValueError(f"joint {name} mimics {self.joints[name].mimic}; command its source")
        positions = complete_positions(self.joints, {**self.positions(), **updates})
        values = np.array(self.robot.joint_positions, copy=True)
        for name, position in positions.items():
            values[self.offsets[name]] = position
        self.robot.joint_positions = values
        self.robot.joint_velocities = np.zeros_like(self.robot.joint_velocities)
