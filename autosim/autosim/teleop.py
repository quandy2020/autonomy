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

Key bindings match ``autonomy_teleop``: ``w``/``x`` step linear speed, ``a``/``d``
step angular speed, ``space``/``s`` stop. Velocity is latched until the next
command (not hold-to-move).
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import tty
from pathlib import Path
from typing import Any, Optional, Tuple

from autosim.clock import Clock
from autosim.config import Config
from autosim.messages import Messages

from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped


HELP = """
Control Your autosim robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
"""


def constrain(value: float, low: float, high: float) -> float:
    """Clamp ``value`` to ``[low, high]``."""
    return max(low, min(high, value))


def make_simple_profile(output: float, target: float, slop: float) -> float:
    """Move ``output`` toward ``target`` by at most ``slop``."""
    if target > output:
        return min(target, output + slop)
    if target < output:
        return max(target, output - slop)
    return target


def get_key(settings: Any, timeout_sec: float = 0.1) -> str:
    """Read one key using the ``autonomy_teleop`` raw-mode pattern.

    Puts stdin in raw mode only for the poll, then restores ``settings`` so
    keystrokes are not left in cbreak for other readers.
    """
    if settings is None:
        return ""
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    key = sys.stdin.read(1) if ready else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Teleop:
    """Terminal keyboard teleop publisher for planar ``cmd_vel``."""

    def __init__(
        self,
        channel: str = "/cmd_vel",
        max_linear: float = 0.5,
        max_angular: float = 1.0,
        rate_hz: float = 20.0,
        frame_id: str = "base_link",
        linear_step: float = 0.05,
        angular_step: float = 0.1,
        link: Any = None,
    ) -> None:
        """Create a teleop publisher.

        Args:
            channel: autolink channel name for ``cmd_vel``.
            max_linear: Peak forward speed (m/s).
            max_angular: Peak yaw rate (rad/s).
            rate_hz: Publish rate (Hz); used when stdin is not a TTY.
            frame_id: Twist header frame.
            linear_step: Linear increment per ``w``/``x``.
            angular_step: Angular increment per ``a``/``d``.
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
        self.linear_step = float(linear_step)
        self.angular_step = float(angular_step)
        self.clock = Clock()
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.control_linear = 0.0
        self.control_angular = 0.0
        self.writer = None
        self.bind_writer()

    @property
    def linear(self) -> float:
        """Latched target linear velocity (m/s)."""
        return self.target_linear

    @property
    def angular(self) -> float:
        """Latched target angular velocity (rad/s)."""
        return self.target_angular

    @staticmethod
    def resolve_link(link: Any) -> Any:
        """Return ``link`` or import autolink."""
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

    def print_vels(self) -> None:
        """Print the latched target velocities (same wording as autonomy_teleop)."""
        print(
            "currently:\tlinear velocity {0:.3f}\t angular velocity {1:.3f}".format(
                self.target_linear, self.target_angular
            ),
            flush=True,
        )

    def apply_key(self, key: str) -> bool:
        """Update latched velocity from one key.

        Returns:
            ``False`` when the operator requested quit; otherwise ``True``.
        """
        if key in ("\x03", "\x04"):
            return False
        changed = False
        if key == "w":
            self.target_linear = constrain(
                self.target_linear + self.linear_step,
                -self.max_linear,
                self.max_linear,
            )
            changed = True
        elif key == "x":
            self.target_linear = constrain(
                self.target_linear - self.linear_step,
                -self.max_linear,
                self.max_linear,
            )
            changed = True
        elif key == "a":
            self.target_angular = constrain(
                self.target_angular + self.angular_step,
                -self.max_angular,
                self.max_angular,
            )
            changed = True
        elif key == "d":
            self.target_angular = constrain(
                self.target_angular - self.angular_step,
                -self.max_angular,
                self.max_angular,
            )
            changed = True
        elif key in (" ", "s"):
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.control_linear = 0.0
            self.control_angular = 0.0
            changed = True
        if changed:
            self.print_vels()
        return True

    def command_speeds(self) -> Tuple[float, float]:
        """Smoothed command velocities."""
        self.control_linear = make_simple_profile(
            self.control_linear, self.target_linear, self.linear_step / 2.0
        )
        self.control_angular = make_simple_profile(
            self.control_angular, self.target_angular, self.angular_step / 2.0
        )
        return self.control_linear, self.control_angular

    def publish_twist(self) -> None:
        """Publish the current command as ``TwistStamped``."""
        linear, angular = self.command_speeds()
        stamp = self.clock.stamp()
        message = Messages.encode_twist_stamped(linear, angular, stamp, self.frame_id)
        self.writer.write(message)

    def run(self) -> None:
        """Main teleop loop until quit."""
        settings = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
        if settings is None:
            print(
                "error: keyboard teleop needs an interactive terminal.\n"
                "  Use: docker exec -it SpaceHero /bin/bash\n"
                "  Then: src/autonomy/autosim/scripts/run.sh teleop",
                file=sys.stderr,
            )
            self.link.shutdown()
            raise SystemExit(1)

        print(HELP)
        print(
            f"channel={self.channel}  max_linear={self.max_linear}  "
            f"max_angular={self.max_angular}  "
            f"lin_step={self.linear_step}  ang_step={self.angular_step}",
            flush=True,
        )
        dt = 1.0 / self.rate_hz
        try:
            while not self.link.is_shutdown():
                key = get_key(settings, timeout_sec=dt)
                if key and not self.apply_key(key):
                    break
                self.clock.tick(dt)
                self.publish_twist()
        finally:
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.control_linear = 0.0
            self.control_angular = 0.0
            self.publish_twist()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.link.shutdown()

    @staticmethod
    def main(argv: Optional[list[str]] = None) -> None:
        """CLI entry: load config limits and start keyboard teleop."""
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
        ).run()
