# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Load XYZ (+ intensity) from PCD ascii / binary (Oxford Spires Hesai layout)."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO

import numpy as np


@dataclass
class _Field:
    name: str
    size: int
    type_char: str
    count: int
    offset: int


def _parse_header(
    fh: BinaryIO,
) -> tuple[list[_Field], int, str]:
    names: list[str] = []
    sizes: list[int] = []
    types: list[str] = []
    counts: list[int] = []
    npts = 0
    data_mode = "ascii"
    while True:
        raw = fh.readline()
        if not raw:
            break
        line = raw.decode("ascii", errors="ignore").strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        if key == "FIELDS":
            names = parts[1:]
        elif key == "SIZE":
            sizes = [int(x) for x in parts[1:]]
        elif key == "TYPE":
            types = [x[:1].upper() for x in parts[1:]]
        elif key == "COUNT":
            counts = [int(x) for x in parts[1:]]
        elif key in ("POINTS", "WIDTH"):
            npts = int(parts[1]) if len(parts) > 1 else npts
        elif key == "DATA":
            data_mode = parts[1].lower() if len(parts) > 1 else "ascii"
            break
    if not names or npts <= 0:
        raise ValueError("invalid PCD header")
    if len(sizes) < len(names):
        sizes = [4] * len(names)
    if len(types) < len(names):
        types = ["F"] * len(names)
    if len(counts) < len(names):
        counts = [1] * len(names)
    fields: list[_Field] = []
    offset = 0
    for i, name in enumerate(names):
        fields.append(
            _Field(
                name=name,
                size=sizes[i],
                type_char=types[i],
                count=counts[i],
                offset=offset,
            )
        )
        offset += sizes[i] * counts[i]
    return fields, npts, data_mode


def _read_float(buf: bytes, field: _Field) -> float:
    base = field.offset
    if field.type_char == "F" and field.size == 4:
        return struct.unpack_from("<f", buf, base)[0]
    if field.type_char == "F" and field.size == 8:
        return float(struct.unpack_from("<d", buf, base)[0])
    if field.type_char in ("U", "I") and field.size == 1:
        return float(buf[base])
    if field.type_char in ("U", "I") and field.size == 2:
        return float(struct.unpack_from("<H", buf, base)[0])
    if field.type_char in ("U", "I") and field.size == 4:
        return float(struct.unpack_from("<I", buf, base)[0])
    return 0.0


def _find(fields: list[_Field], *names: str) -> _Field | None:
    wanted = {n.lower() for n in names}
    for field in fields:
        if field.name.lower() in wanted:
            return field
    return None


def load_pcd_xyzi(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Load a PCD file as ``(Nx3 xyz, N intensity)`` float32 arrays."""
    path = Path(path)
    with path.open("rb") as fh:
        fields, npts, data_mode = _parse_header(fh)
        fx = _find(fields, "x")
        fy = _find(fields, "y")
        fz = _find(fields, "z")
        if fx is None or fy is None or fz is None:
            raise ValueError(f"PCD missing xyz: {path}")
        fi = _find(fields, "intensity", "reflectivity")
        point_step = sum(f.size * f.count for f in fields)
        if point_step <= 0:
            raise ValueError(f"invalid PCD point_step: {path}")

        xyz = np.zeros((npts, 3), dtype=np.float32)
        intensity = np.zeros((npts,), dtype=np.float32)

        if data_mode == "ascii":
            i = 0
            while i < npts:
                raw = fh.readline()
                if not raw:
                    break
                line = raw.decode("ascii", errors="ignore").strip()
                if not line or line.startswith("#"):
                    continue
                vals = [float(v) for v in line.split()]
                if len(vals) < 3:
                    continue
                xyz[i, 0] = vals[0]
                xyz[i, 1] = vals[1]
                xyz[i, 2] = vals[2]
                intensity[i] = vals[3] if len(vals) > 3 else 0.0
                i += 1
            return xyz[:i], intensity[:i]

        if data_mode == "binary_compressed":
            raise ValueError(f"binary_compressed PCD not supported: {path}")

        blob = fh.read(point_step * npts)
        if len(blob) < point_step * npts:
            raise ValueError(f"truncated PCD body: {path}")
        for i in range(npts):
            base = i * point_step
            chunk = blob[base : base + point_step]
            xyz[i, 0] = _read_float(chunk, fx)
            xyz[i, 1] = _read_float(chunk, fy)
            xyz[i, 2] = _read_float(chunk, fz)
            intensity[i] = _read_float(chunk, fi) if fi is not None else 0.0
        return xyz, intensity
