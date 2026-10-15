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

"""Autolink Node adapter: sensor writers and a ``cmd_vel`` reader."""

from __future__ import annotations

from typing import Any, Dict


class CommandReader:
    """Poll the latest ``cmd_vel`` message from an autolink callback reader."""

    def __init__(self, node: Any, channel: str, dtype: Any) -> None:
        self.pending_message: Any = None
        self.has_message = False

        def on_message(message: Any) -> None:
            self.pending_message = message
            self.has_message = True

        node.create_reader(channel, on_message, dtype)

    def has_msg(self) -> bool:
        """Return True when a new command is waiting."""
        return self.has_message

    def get_msg(self) -> Any:
        """Consume and return the pending command message."""
        self.has_message = False
        return self.pending_message


class Bridge:
    """Thin writer/reader wrapper around an Autolink ``Node``."""

    def __init__(self, node: Any, channels: Dict[str, str], types: Dict[str, Any]) -> None:
        """Create writers and the command reader from channel and type maps.

        Args:
            node: Autolink ``Node`` with ``create_writer`` / ``create_reader``.
            channels: Logical key → channel name, e.g. ``{"scan": "/scan"}``.
            types: Logical key → protobuf type. Writers are created for every
                key except ``cmd_vel``, which always becomes a reader.
        """
        self.node = node
        self.channels = channels
        self.writers = {
            key: node.create_writer(channels[key], types[key], qos_depth=10)
            for key in types
            if key in channels and key != "cmd_vel"
        }
        self.command_reader = CommandReader(
            node, channels["cmd_vel"], types["cmd_vel"]
        )

    def publish(self, key: str, message: Any) -> None:
        """Publish on a registered logical channel; ignore unknown keys.

        Args:
            key: Logical stream name such as ``scan`` or ``odom``.
            message: Protobuf message instance.
        """
        writer = self.writers.get(key)
        if writer is not None:
            writer.write(message)
