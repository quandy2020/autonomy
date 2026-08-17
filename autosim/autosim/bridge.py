from __future__ import annotations

from typing import Any, Dict


class Bridge:
    def __init__(self, node: Any, channels: Dict[str, str], types: Dict[str, Any]) -> None:
        self.node = node
        self.channels = channels
        self.writers = {
            key: node.create_writer(channels[key], types[key], qos_depth=10)
            for key in ("scan", "rgb", "depth", "camera_info", "imu", "odom", "gt_pose")
            if key in types
        }
        self.cmd_reader = node.create_reader(channels["cmd_vel"], types["cmd_vel"], qos_depth=10)

    def publish(self, key: str, msg: Any) -> None:
        if key in self.writers:
            self.writers[key].write(msg)
