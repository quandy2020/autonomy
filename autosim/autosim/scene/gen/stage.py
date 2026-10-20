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

"""Habitat collision mesh for a boolean occupancy volume."""

from __future__ import annotations

from pathlib import Path

import numpy as np


class Stage:
    """Ground plane plus exposed voxel faces, written as a Y-up OBJ."""

    def write(
        self,
        occupied: np.ndarray,
        origin: np.ndarray,
        resolution: float,
        path: Path,
    ) -> Path:
        """Write the stage mesh.

        Args:
            occupied: Boolean volume ``(x, y, z)``.
            origin: Map-frame minimum corner of voxel ``(0,0,0)``.
            resolution: Voxel edge length in meters.
            path: Destination OBJ path.

        Returns:
            ``path``.
        """
        triangles = [self.ground(occupied.shape, origin, resolution)]
        triangles.extend(self.exposed_faces(occupied, origin, resolution))
        vertices = self.to_habitat(np.concatenate(triangles, axis=0)).reshape(-1, 3)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            handle.write("# autosim scenario stage, Habitat Y-up\n")
            for x, y, z in vertices:
                handle.write(f"v {x:.5f} {y:.5f} {z:.5f}\n")
            for index in range(0, vertices.shape[0], 3):
                handle.write(f"f {index + 1} {index + 2} {index + 3}\n")
        return path

    @staticmethod
    def ground(shape: tuple[int, int, int], origin: np.ndarray, resolution: float) -> np.ndarray:
        """Two triangles covering the map footprint at the volume floor."""
        x0, y0, z0 = origin
        x1 = x0 + shape[0] * resolution
        y1 = y0 + shape[1] * resolution
        return np.array(
            [
                [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0]],
                [[x0, y0, z0], [x1, y1, z0], [x0, y1, z0]],
            ],
            dtype=np.float64,
        )

    def exposed_faces(
        self, occupied: np.ndarray, origin: np.ndarray, resolution: float
    ) -> list[np.ndarray]:
        """Outward faces where an occupied voxel touches air or the volume border."""
        faces: list[np.ndarray] = []
        for axis, positive, negative in self.exposure_masks(occupied):
            for sign, mask in ((1, positive), (-1, negative)):
                quads = self.face_quads(mask, axis, sign, origin, resolution)
                if quads.shape[0]:
                    faces.append(self.quads_to_triangles(quads))
        return faces or [np.zeros((0, 3, 3), dtype=np.float64)]

    @staticmethod
    def exposure_masks(
        occupied: np.ndarray,
    ) -> list[tuple[int, np.ndarray, np.ndarray]]:
        """Per-axis masks of +side and -side exposed occupied cells."""
        masks = []
        for axis in range(3):
            positive = np.zeros(occupied.shape, dtype=bool)
            negative = np.zeros(occupied.shape, dtype=bool)
            high = [slice(None)] * 3
            low = [slice(None)] * 3
            high[axis] = slice(None, -1)
            low[axis] = slice(1, None)
            positive[tuple(high)] = occupied[tuple(high)] & ~occupied[tuple(low)]
            negative[tuple(low)] = occupied[tuple(low)] & ~occupied[tuple(high)]
            positive[tuple(Stage.edge(axis, occupied.shape, +1))] = occupied[
                tuple(Stage.edge(axis, occupied.shape, +1))
            ]
            negative[tuple(Stage.edge(axis, occupied.shape, -1))] = occupied[
                tuple(Stage.edge(axis, occupied.shape, -1))
            ]
            masks.append((axis, positive, negative))
        return masks

    @staticmethod
    def face_quads(
        mask: np.ndarray,
        axis: int,
        sign: int,
        origin: np.ndarray,
        resolution: float,
    ) -> np.ndarray:
        """Quad corners for every true cell in ``mask``."""
        index = np.argwhere(mask)
        if index.shape[0] == 0:
            return np.zeros((0, 4, 3), dtype=np.float64)
        base = origin + index.astype(np.float64) * resolution
        if sign > 0:
            base[:, axis] += resolution
        plane = [dim for dim in range(3) if dim != axis]
        offsets = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float64) * resolution
        if sign < 0:
            offsets = offsets[::-1]
        corners = np.repeat(base[:, None, :], 4, axis=1)
        corners[:, :, plane[0]] += offsets[:, 0]
        corners[:, :, plane[1]] += offsets[:, 1]
        return corners

    @staticmethod
    def edge(axis: int, shape: tuple[int, ...], sign: int) -> tuple[slice, ...]:
        """Last slab along ``axis`` when ``sign > 0``, otherwise the first slab."""
        index: list[slice] = [slice(None)] * len(shape)
        index[axis] = slice(-1, None) if sign > 0 else slice(0, 1)
        return tuple(index)

    @staticmethod
    def quads_to_triangles(corners: np.ndarray) -> np.ndarray:
        """Split ``(F, 4, 3)`` quads into ``(2F, 3, 3)`` triangles."""
        a, b, c, d = corners[:, 0], corners[:, 1], corners[:, 2], corners[:, 3]
        return np.stack((a, b, c, a, c, d), axis=1).reshape(-1, 3, 3)

    @staticmethod
    def to_habitat(points: np.ndarray) -> np.ndarray:
        """Map frame (z up) to Habitat (y up): ``(x, z, -y)``."""
        converted = np.empty_like(points)
        converted[..., 0] = points[..., 0]
        converted[..., 1] = points[..., 2]
        converted[..., 2] = -points[..., 1]
        return converted
