"""Named kinematic control using Habitat's actual joint-position offsets."""
from __future__ import annotations

import time
from typing import Mapping, Any
import numpy as np

from autosim.robot.joints import Joint, complete_positions


class Articulation:
    def __init__(self, robot: Any, joints: Mapping[str, Joint]) -> None:
        self.robot = robot
        self.joints = dict(joints)
        self.offsets = {}
        self.velocity_offsets = {}
        self.stamp: float | None = None
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

    def set_positions(
        self, updates: Mapping[str, float], dt: float | None = None
    ) -> None:
        """Apply a partial pose and derive joint velocities from the change.

        Habitat is kinematic, so ``joint_velocities`` stays zero unless this
        controller writes it. Finite differences over the wall-clock interval
        since the previous write give ``/joint_states`` a usable velocity for
        controllers that watch it; re-applying the same pose decays it to zero.

        Args:
            updates: Partial ``name → position`` commands (rad or m).
            dt: Interval override in seconds; wall clock when ``None``.
        """
        for name in updates:
            if name not in self.joints:
                raise ValueError(f"unknown joint: {name}")
            if self.joints[name].mimic:
                raise ValueError(f"joint {name} mimics {self.joints[name].mimic}; command its source")
        previous = self.positions()
        positions = complete_positions(self.joints, {**previous, **updates})
        values = np.array(self.robot.joint_positions, copy=True)
        for name, position in positions.items():
            values[self.offsets[name]] = position
        self.robot.joint_positions = values
        now = time.monotonic()
        span = float(dt) if dt is not None else (
            now - self.stamp if self.stamp is not None else 0.0
        )
        self.stamp = now
        velocities = np.zeros_like(self.robot.joint_velocities)
        if span > 1e-6:
            for name, offset in self.velocity_offsets.items():
                velocities[offset] = (positions[name] - previous[name]) / span
        self.robot.joint_velocities = velocities
