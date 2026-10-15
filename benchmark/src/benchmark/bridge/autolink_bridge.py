# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Autolink publisher/subscriber bridge for benchmark ↔ autosim / autonomy."""

from __future__ import annotations

import logging
import math
import time
from typing import Any, Callable, Dict, Optional, Tuple

from benchmark.core.types import Pose2D, Twist2D

_LOG = logging.getLogger(__name__)


def _stamp_now() -> Tuple[int, int]:
    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1e9)
    return sec, nsec


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    # Planar yaw from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AutolinkBridge:
    """Publish ``cmd_vel`` and optionally listen to ``odom`` / ``scan``.

    Falls back to an in-memory null transport when autolink/automsgs are absent
    so unit tests and offline Arena Evaluation keep working.
    """

    def __init__(
        self,
        node_name: str = "benchmark",
        *,
        cmd_vel: str = "/cmd_vel",
        odom: str = "/odom",
        scan: str = "/scan",
        frame_id: str = "base_link",
    ) -> None:
        self.node_name = node_name
        self.cmd_vel_channel = cmd_vel
        self.odom_channel = odom
        self.scan_channel = scan
        self.frame_id = frame_id
        self._link: Any = None
        self._node: Any = None
        self._cmd_writer: Any = None
        self._connected = False
        self._latest_odom: Optional[Pose2D] = None
        self._latest_scan: Optional[Any] = None
        self._null_log: list[Twist2D] = []

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self, *, subscribe_odom: bool = True, subscribe_scan: bool = False) -> bool:
        try:
            import autolink as link
            from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
            from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
            from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan
        except ImportError as exc:
            _LOG.warning("autolink/automsgs unavailable (%s); null transport", exc)
            self._connected = False
            return False

        try:
            link.init(self.node_name)
            self._link = link
            self._node = link.Node(self.node_name)
            self._cmd_writer = self._node.create_writer(
                self.cmd_vel_channel, TwistStamped, qos_depth=10
            )
            if subscribe_odom:
                self._node.create_reader(self.odom_channel, self._on_odom, Odometry)
            if subscribe_scan:
                self._node.create_reader(self.scan_channel, self._on_scan, LaserScan)
            self._connected = True
            _LOG.info(
                "AutolinkBridge up node=%s cmd_vel=%s odom=%s",
                self.node_name,
                self.cmd_vel_channel,
                self.odom_channel if subscribe_odom else "-",
            )
            return True
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("AutolinkBridge connect failed: %s", exc)
            self._connected = False
            return False

    def _on_odom(self, message: Any) -> None:
        try:
            p = message.pose.pose.position
            q = message.pose.pose.orientation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            self._latest_odom = Pose2D(x=float(p.x), y=float(p.y), yaw=float(yaw))
        except Exception:  # noqa: BLE001
            pass

    def _on_scan(self, message: Any) -> None:
        self._latest_scan = message

    def publish_twist(self, twist: Twist2D) -> None:
        self._null_log.append(twist)
        if not self._connected or self._cmd_writer is None:
            return
        try:
            from autosim.io.messages import Messages

            msg = Messages.encode_twist_stamped(
                twist.linear, twist.angular, _stamp_now(), self.frame_id
            )
            self._cmd_writer.write(msg)
        except Exception:
            # Fallback without autosim helper
            try:
                from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped

                msg = TwistStamped()
                sec, nsec = _stamp_now()
                msg.header.stamp.sec = sec
                msg.header.stamp.nanosec = nsec
                msg.header.frame_id = self.frame_id
                msg.twist.linear.x = float(twist.linear)
                msg.twist.angular.z = float(twist.angular)
                self._cmd_writer.write(msg)
            except Exception as exc:  # noqa: BLE001
                _LOG.debug("publish_twist failed: %s", exc)

    def latest_pose(self) -> Optional[Pose2D]:
        return self._latest_odom

    def latest_scan(self) -> Any:
        return self._latest_scan

    def spin_once(self) -> None:
        """Yield so autolink callbacks can run (best-effort)."""
        if self._link is None:
            return
        for attr in ("spin_once", "spinOnce", "WaitForShutdown"):
            fn: Optional[Callable[..., Any]] = getattr(self._link, attr, None)
            if callable(fn) and attr.startswith("spin"):
                try:
                    fn()
                except TypeError:
                    try:
                        fn(0.001)
                    except Exception:  # noqa: BLE001
                        pass
                return

    def shutdown(self) -> None:
        self._cmd_writer = None
        self._node = None
        self._connected = False
        if self._link is not None:
            for attr in ("shutdown", "clear"):
                fn = getattr(self._link, attr, None)
                if callable(fn):
                    try:
                        fn()
                    except Exception:  # noqa: BLE001
                        pass
        self._link = None
