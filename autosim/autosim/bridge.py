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

"""autolink Node adapter: writers for sensors and a reader for ``cmd_vel``."""

from __future__ import annotations

from typing import Any, Dict


class _CommandReader:
    """Poll the latest cmd_vel message from an autolink callback reader."""

    def __init__(self, node: Any, channel: str, dtype: Any) -> None:
        self._pending: Any = None
        self._has = False

        def on_message(message: Any) -> None:
            self._pending = message
            self._has = True

        node.create_reader(channel, on_message, dtype)

    def has_msg(self) -> bool:
        return self._has

    def get_msg(self) -> Any:
        self._has = False
        return self._pending


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
                "depth_points",
                "semantic",
                "semantic_ids",
                "camera_info",
                "imu",
                "odom",
                "gt_pose",
                "footprint",
                "map_cloud",
                "map_grid",
                "tf",
                "tf_static",
                "clock",
            )
            if key in types and key in channels
        }
        self.command_reader = _CommandReader(
            node, channels["cmd_vel"], types["cmd_vel"]
        )

    def publish(self, key: str, message: Any) -> None:
        """Publish on a registered logical channel; ignore unknown keys.

        Args:
            key: Logical key such as ``scan`` or ``odom``.
            message: Protobuf message instance.
        """
        if key in self.writers:
            self.writers[key].write(message)
