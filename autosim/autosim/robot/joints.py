"""URDF scalar joint definitions, validation, mimic joints and transforms."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    child: str
    lower: float | None = None
    upper: float | None = None
    mimic: str | None = None
    multiplier: float = 1.0
    offset: float = 0.0

    def check(self, value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"joint {self.name}: position must be finite")
        if self.lower is not None and value < self.lower - 1e-7:
            raise ValueError(f"joint {self.name}: {value} below {self.lower}")
        if self.upper is not None and value > self.upper + 1e-7:
            raise ValueError(f"joint {self.name}: {value} above {self.upper}")
        return value

    def home(self) -> float:
        return min(self.upper if self.upper is not None else math.inf,
                   max(self.lower if self.lower is not None else -math.inf, 0.0))


def load_joints(path: Path) -> dict[str, Joint]:
    result = {}
    for node in ET.parse(path).getroot().findall("joint"):
        kind = node.get("type", "fixed")
        if kind == "fixed":
            continue
        if kind not in ("revolute", "continuous", "prismatic"):
            raise ValueError(f"unsupported joint type {kind!r}: {node.get('name')}")
        limit = node.find("limit")
        bounds = []
        for key in ("lower", "upper"):
            value = float(limit.get(key)) if kind != "continuous" and limit is not None and limit.get(key) is not None else None
            bounds.append(value if value is not None and math.isfinite(value) else None)
        if bounds[0] is not None and bounds[1] is not None and bounds[0] > bounds[1]:
            raise ValueError(f"invalid limits: {node.get('name')}")
        mimic = node.find("mimic")
        child = node.find("child")
        joint = Joint(node.attrib["name"], kind, child.attrib["link"], *bounds,
                      mimic.get("joint") if mimic is not None else None,
                      float(mimic.get("multiplier", "1")) if mimic is not None else 1.0,
                      float(mimic.get("offset", "0")) if mimic is not None else 0.0)
        result[joint.name] = joint
    return result


def complete_positions(joints: Mapping[str, Joint], positions: Mapping[str, float]) -> dict[str, float]:
    """Validate a complete pose; resolve mimic dependencies recursively."""
    unknown = set(positions) - set(joints)
    if unknown:
        raise ValueError(f"unknown joints: {sorted(unknown)}")
    result: dict[str, float] = {}
    visiting: set[str] = set()

    def resolve(name: str) -> float:
        if name in result:
            return result[name]
        if name in visiting:
            raise ValueError(f"mimic cycle: {name}")
        if name not in joints:
            raise ValueError(f"unknown mimic source: {name}")
        visiting.add(name)
        joint = joints[name]
        value = (resolve(joint.mimic) * joint.multiplier + joint.offset
                 if joint.mimic else positions.get(name, joint.home()))
        result[name] = joint.check(value)
        visiting.remove(name)
        return result[name]

    for name in joints:
        resolve(name)
    return result


def joint_transform(xyz, rpy, axis, kind: str, position: float):
    """Return origin * joint-motion as xyz and quaternion (xyzw)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])
    a = np.asarray(axis, dtype=float)
    length = np.linalg.norm(a)
    if length == 0:
        raise ValueError("joint axis has zero length")
    a /= length
    translation = np.asarray(xyz, dtype=float)
    if kind == "prismatic":
        translation = translation + rotation @ (a * position)
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    v = np.array([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])
    w = cr*cp*cy+sr*sp*sy
    if kind in ("continuous", "revolute"):
        b, c = a * math.sin(position/2), math.cos(position/2)
        v, w = w*b+c*v+np.cross(v,b), w*c-float(np.dot(v,b))
    return tuple(translation), (*map(float, v), float(w))
