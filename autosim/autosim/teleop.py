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

"""Keyboard teleoperation node that publishes ``cmd_vel`` via autolink.

Reads raw terminal keys and emits ``geometry_msgs.TwistStamped`` on the
configured command channel (default ``/cmd_vel``). Motion decays to zero when
no movement key is received within ``key_timeout``.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Any, Optional, Tuple

from autosim.clock import Clock
from autosim.config import Config
from autosim.messages import Messages

from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped


HELP = """
Keyboard teleop (publish TwistStamped on cmd_vel)

   w
a  s  d          move / turn (auto-stop if no key)
space            stop
q / e            raise / lower linear scale
z / c            raise / lower angular scale
Ctrl-C           quit
"""


class Teleop:
    """Terminal keyboard teleop publisher for planar ``cmd_vel``."""

    def __init__(
        self,
        channel: str = "/cmd_vel",
        max_linear: float = 0.5,
        max_angular: float = 1.0,
        rate_hz: float = 20.0,
        frame_id: str = "base_link",
        key_timeout: float = 0.3,
        link: Any = None,
    ) -> None:
        """Create a teleop publisher.

        Args:
            channel: autolink channel name for ``cmd_vel``.
            max_linear: Peak forward speed (m/s).
            max_angular: Peak yaw rate (rad/s).
            rate_hz: Publish rate (Hz).
            frame_id: Twist header frame.
            key_timeout: Seconds without a motion key before commanding stop.
            link: Injected autolink module; otherwise ``import autolink``.

        Raises:
            ImportError: autolink missing and ``link`` not provided.
        """
        self.link = self.resolve_link(link)
        self.channel = str(channel)
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.rate_hz = float(rate_hz)
        self.frame_id = str(frame_id)
        self.key_timeout = float(key_timeout)
        self.clock = Clock()
        self.linear = 0.0
        self.angular = 0.0
        self.linear_scale = 1.0
        self.angular_scale = 1.0
        self.last_motion_time = -1e9
        self.writer = None
        self.bind_writer()

    @staticmethod
    def resolve_link(link: Any) -> Any:
        """Return ``link`` or import autolink.

        Args:
            link: Optional pre-bound module.

        Returns:
            autolink module.

        Raises:
            ImportError: Package not installed.
        """
        if link is not None:
            return link
        try:
            import autolink as resolved
        except ImportError as exc:
            raise ImportError("autolink Python package is required") from exc
        return resolved

    def bind_writer(self) -> None:
        """Initialize autolink and create the ``cmd_vel`` writer."""
        self.link.init("autosim_teleop")
        self.node = self.link.Node("autosim_teleop")
        self.writer = self.node.create_writer(self.channel, TwistStamped, qos_depth=10)

    def command_speeds(self) -> Tuple[float, float]:
        """Scaled command velocities.

        Returns:
            ``(linear_x, angular_z)`` clamped by configured maxima.
        """
        linear = max(-self.max_linear, min(self.max_linear, self.linear * self.linear_scale))
        angular = max(
            -self.max_angular, min(self.max_angular, self.angular * self.angular_scale)
        )
        return linear, angular

    def apply_key(self, key: str) -> bool:
        """Update motion state from one key.

        Args:
            key: Single character or empty string.

        Returns:
            ``False`` when the operator requested quit; otherwise ``True``.
        """
        if key in ("\x03", "\x04"):
            return False
        mapping = {
            "w": (1.0, 0.0),
            "s": (-1.0, 0.0),
            "a": (0.0, 1.0),
            "d": (0.0, -1.0),
            " ": (0.0, 0.0),
        }
        if key in mapping:
            self.linear, self.angular = mapping[key]
            self.last_motion_time = self.clock.now()
        elif key == "q":
            self.linear_scale = min(1.0, self.linear_scale + 0.1)
        elif key == "e":
            self.linear_scale = max(0.1, self.linear_scale - 0.1)
        elif key == "z":
            self.angular_scale = min(1.0, self.angular_scale + 0.1)
        elif key == "c":
            self.angular_scale = max(0.1, self.angular_scale - 0.1)
        return True

    def decay_motion(self) -> None:
        """Zero velocities when no motion key arrives within ``key_timeout``."""
        if self.clock.now() - self.last_motion_time > self.key_timeout:
            self.linear = 0.0
            self.angular = 0.0

    def publish_twist(self) -> None:
        """Publish the current command as ``TwistStamped``."""
        linear, angular = self.command_speeds()
        stamp = self.clock.stamp()
        message = Messages.encode_twist_stamped(linear, angular, stamp, self.frame_id)
        self.writer.write(message)

    @staticmethod
    def read_key(timeout_sec: float) -> str:
        """Non-blocking raw stdin read.

        Args:
            timeout_sec: Select timeout in seconds.

        Returns:
            One character, or ``""`` on timeout / non-tty.
        """
        if not sys.stdin.isatty():
            return ""
        ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not ready:
            return ""
        return sys.stdin.read(1)

    def run(self) -> None:
        """Main teleop loop until quit.

        Restores terminal settings on exit and publishes a final zero twist.
        """
        print(HELP)
        print(
            f"channel={self.channel}  max_linear={self.max_linear}  "
            f"max_angular={self.max_angular}  key_timeout={self.key_timeout}"
        )
        dt = 1.0 / self.rate_hz
        settings = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
        try:
            if settings is not None:
                tty.setcbreak(sys.stdin.fileno())
            while not self.link.is_shutdown():
                key = self.read_key(dt)
                if key and not self.apply_key(key):
                    break
                self.decay_motion()
                self.clock.tick(dt)
                self.publish_twist()
        finally:
            self.linear = 0.0
            self.angular = 0.0
            self.publish_twist()
            if settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.link.shutdown()

    @staticmethod
    def main(argv: Optional[list[str]] = None) -> None:
        """CLI entry: load config limits and start keyboard teleop.

        Args:
            argv: Argument vector; defaults to ``sys.argv``.
        """
        parser = argparse.ArgumentParser(prog="autosim-teleop")
        parser.add_argument(
            "--config",
            type=Path,
            default=Path(__file__).resolve().parents[1] / "config" / "default.yaml",
        )
        args = parser.parse_args(argv)
        settings = Config.load(args.config)
        robot = settings["habitat"]["robot"]
        Teleop(
            channel=robot["cmd_vel"],
            max_linear=float(robot["max_linear"]),
            max_angular=float(robot["max_angular"]),
            rate_hz=min(20.0, float(robot["control_hz"])),
            frame_id="base_link",
            key_timeout=float(robot.get("watchdog_sec", 0.3)),
        ).run()
