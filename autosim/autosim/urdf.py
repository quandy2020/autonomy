"""Lightweight URDF path resolve and sensor-mount extraction."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple


class UrdfModel:
    """Resolved URDF path plus link mounts in ROS base frame (z-up)."""

    SENSOR_LINKS = ("laser_link", "base_scan", "imu_link", "camera_link")

    def __init__(self, path: Path, mounts: Mapping[str, Tuple[float, float, float]]) -> None:
        """Store path and mounts.

        Args:
            path: Absolute URDF file path.
            mounts: Link name → ``(x, y, z)`` meters in root frame (ROS z-up).
        """
        self.path = path
        self.mounts = dict(mounts)

    @staticmethod
    def package_root() -> Path:
        """autosim project root (parent of the Python package)."""
        return Path(__file__).resolve().parents[1]

    @classmethod
    def resolve(cls, urdf: str) -> Optional[Path]:
        """Resolve config path; empty string → ``None``.

        Relative paths are under the autosim project root.

        Args:
            urdf: Absolute or project-relative path.

        Returns:
            Absolute path, or ``None`` if blank.
        """
        text = str(urdf or "").strip()
        if not text:
            return None
        path = Path(text)
        if not path.is_absolute():
            path = cls.package_root() / path
        return path.resolve()

    @classmethod
    def load(cls, urdf: str) -> Optional["UrdfModel"]:
        """Load URDF and accumulate mounts for known sensor links.

        Args:
            urdf: Config path (may be empty).

        Returns:
            Model, or ``None`` when ``urdf`` is empty.

        Raises:
            FileNotFoundError: Path set but missing.
            ValueError: Malformed XML or no joints.
        """
        path = cls.resolve(urdf)
        if path is None:
            return None
        if not path.is_file():
            raise FileNotFoundError(f"urdf not found: {path}")
        tree = ET.parse(path)
        parents = cls.joint_parents(tree.getroot())
        if not parents:
            raise ValueError(f"urdf has no joints: {path}")
        root = cls.root_link(parents)
        mounts: Dict[str, Tuple[float, float, float]] = {}
        for link in cls.SENSOR_LINKS:
            if link in parents or link == root:
                mounts[link] = cls.link_xyz(link, parents, root)
        return cls(path=path, mounts=mounts)

    @staticmethod
    def joint_parents(root: ET.Element) -> Dict[str, Tuple[str, Tuple[float, float, float]]]:
        """Map child link → ``(parent, xyz)`` from fixed/continuous joints.

        Args:
            root: ``<robot>`` element.

        Returns:
            Child → parent and origin translation.
        """
        parents: Dict[str, Tuple[str, Tuple[float, float, float]]] = {}
        for joint in root.findall("joint"):
            parent_el = joint.find("parent")
            child_el = joint.find("child")
            if parent_el is None or child_el is None:
                continue
            parent = parent_el.get("link")
            child = child_el.get("link")
            if not parent or not child:
                continue
            parents[child] = (parent, UrdfModel.origin_xyz(joint.find("origin")))
        return parents

    @staticmethod
    def origin_xyz(origin: Optional[ET.Element]) -> Tuple[float, float, float]:
        """Parse ``xyz`` from an ``<origin>`` element."""
        if origin is None:
            return (0.0, 0.0, 0.0)
        parts = str(origin.get("xyz", "0 0 0")).split()
        if len(parts) != 3:
            return (0.0, 0.0, 0.0)
        return (float(parts[0]), float(parts[1]), float(parts[2]))

    @staticmethod
    def root_link(parents: Mapping[str, Tuple[str, Tuple[float, float, float]]]) -> str:
        """Pick ``base_footprint``, else ``base_link``, else first parent without parent."""
        children = set(parents)
        candidates = {parent for parent, _ in parents.values()} - children
        if "base_footprint" in candidates:
            return "base_footprint"
        if "base_link" in candidates:
            return "base_link"
        if candidates:
            return sorted(candidates)[0]
        raise ValueError("urdf has no root link")

    @staticmethod
    def link_xyz(
        link: str,
        parents: Mapping[str, Tuple[str, Tuple[float, float, float]]],
        root: str,
    ) -> Tuple[float, float, float]:
        """Accumulate translations from ``root`` to ``link`` (ignore rpy)."""
        x = y = z = 0.0
        current = link
        for _ in range(len(parents) + 1):
            if current == root:
                return (x, y, z)
            if current not in parents:
                raise ValueError(f"link {link!r} not connected to root {root!r}")
            parent, (dx, dy, dz) = parents[current]
            x += dx
            y += dy
            z += dz
            current = parent
        raise ValueError(f"cycle while resolving link {link!r}")

    def laser_xyz(self) -> Tuple[float, float, float]:
        """Preferred lidar mount: ``laser_link`` then ``base_scan``."""
        if "laser_link" in self.mounts:
            return self.mounts["laser_link"]
        if "base_scan" in self.mounts:
            return self.mounts["base_scan"]
        return (0.0, 0.0, 0.0)

    def imu_xyz(self) -> Tuple[float, float, float]:
        """IMU mount or origin."""
        return self.mounts.get("imu_link", (0.0, 0.0, 0.0))

    def camera_xyz(self) -> Tuple[float, float, float]:
        """Camera mount or origin."""
        return self.mounts.get("camera_link", (0.0, 0.0, 0.0))
