"""autolink topic configuration."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AutolinkTopics:
    depth: str = "/camera/depth/image_raw"
    odom: str = "/odom"
    goal: str = "/navigation/goal"
    cmd: str = "/cmd_vel"
