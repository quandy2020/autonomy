"""autolink Node adapter: writers for sensors and a reader for ``cmd_vel``."""

from __future__ import annotations

from typing import Any, Dict


class Bridge:
    """Thin Writer/Reader wrapper around an autolink Node."""

    def __init__(self, node: Any, channels: Dict[str, str], types: Dict[str, Any]) -> None:
        """Create writers and the command reader from channel and type maps.

        Args:
            node: autolink ``Node`` with ``create_writer`` / ``create_reader``.
            channels: Logical key to channel name, e.g. ``{"scan": "/scan", ...}``.
            types: Logical key to protobuf type. Writers are created only for keys
                present in this map; ``cmd_vel`` always gets a Reader.
        """
        self.node = node
        self.channels = channels
        self.writers = {
            key: node.create_writer(channels[key], types[key], qos_depth=10)
            for key in (
                "scan",
                "points",
                "rgb",
                "depth",
                "camera_info",
                "imu",
                "odom",
                "gt_pose",
                "map_cloud",
                "map_grid",
                "tf",
                "tf_static",
                "clock",
            )
            if key in types and key in channels
        }
        self.command_reader = node.create_reader(
            channels["cmd_vel"], types["cmd_vel"], qos_depth=10
        )

    def publish(self, key: str, message: Any) -> None:
        """Publish on a registered logical channel; ignore unknown keys.

        Args:
            key: Logical key such as ``scan`` or ``odom``.
            message: Protobuf message instance.
        """
        if key in self.writers:
            self.writers[key].write(message)
