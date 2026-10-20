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

"""Improved Perlin noise used by the procedural scenario generator.

Port of the permutation-vector noise in
`HKUST-Aerial-Robotics/mockamap <https://github.com/HKUST-Aerial-Robotics/mockamap>`_
(itself a C++ translation of Ken Perlin's reference). The shuffle uses NumPy,
so a given seed does not reproduce the C++ ``std::default_random_engine`` map.
"""

from __future__ import annotations

import numpy as np


class Perlin:
    """3D improved Perlin noise on scalar or array coordinates."""

    def __init__(self, seed: int) -> None:
        """Build a permutation table from ``seed``.

        Args:
            seed: Non-negative integer seed.
        """
        table = np.random.default_rng(int(seed)).permutation(256).astype(np.int32)
        self.table = np.concatenate([table, table])

    def noise(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
        """Sample noise in ``[0, 1]``.

        Args:
            x: X coordinates.
            y: Y coordinates.
            z: Z coordinates. Broadcasts with ``x`` and ``y``.

        Returns:
            Noise with the broadcast shape of the inputs.
        """
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        z = np.asarray(z, dtype=np.float64)
        xi = np.floor(x).astype(np.int32) & 255
        yi = np.floor(y).astype(np.int32) & 255
        zi = np.floor(z).astype(np.int32) & 255
        xf = x - np.floor(x)
        yf = y - np.floor(y)
        zf = z - np.floor(z)
        u = self.fade(xf)
        v = self.fade(yf)
        w = self.fade(zf)
        hashed = self.corner_hashes(xi, yi, zi)
        return (self.blend(hashed, xf, yf, zf, u, v, w) + 1.0) / 2.0

    def corner_hashes(
        self, xi: np.ndarray, yi: np.ndarray, zi: np.ndarray
    ) -> tuple[np.ndarray, ...]:
        """Hash the eight corners of the unit cube containing each sample."""
        table = self.table
        aa = table[table[xi] + yi] + zi
        ab = table[table[xi] + yi + 1] + zi
        ba = table[table[xi + 1] + yi] + zi
        bb = table[table[xi + 1] + yi + 1] + zi
        return (
            table[aa],
            table[ba],
            table[ab],
            table[bb],
            table[aa + 1],
            table[ba + 1],
            table[ab + 1],
            table[bb + 1],
        )

    def blend(
        self,
        hashed: tuple[np.ndarray, ...],
        x: np.ndarray,
        y: np.ndarray,
        z: np.ndarray,
        u: np.ndarray,
        v: np.ndarray,
        w: np.ndarray,
    ) -> np.ndarray:
        """Trilinear blend of the eight corner gradients."""
        c00, c10, c01, c11, c00z, c10z, c01z, c11z = hashed
        x1 = self.lerp(
            u, self.grad(c00, x, y, z), self.grad(c10, x - 1.0, y, z)
        )
        x2 = self.lerp(
            u, self.grad(c01, x, y - 1.0, z), self.grad(c11, x - 1.0, y - 1.0, z)
        )
        y1 = self.lerp(v, x1, x2)
        x3 = self.lerp(
            u, self.grad(c00z, x, y, z - 1.0), self.grad(c10z, x - 1.0, y, z - 1.0)
        )
        x4 = self.lerp(
            u,
            self.grad(c01z, x, y - 1.0, z - 1.0),
            self.grad(c11z, x - 1.0, y - 1.0, z - 1.0),
        )
        return self.lerp(w, y1, self.lerp(v, x3, x4))

    @staticmethod
    def fade(t: np.ndarray) -> np.ndarray:
        """Perlin smoothstep ``6t^5 - 15t^4 + 10t^3``."""
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)

    @staticmethod
    def lerp(t: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Linear interpolate from ``a`` to ``b``."""
        return a + t * (b - a)

    @staticmethod
    def grad(h: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
        """Gradient contribution from the low 4 bits of ``h``."""
        bits = np.asarray(h) & 15
        u = np.where(bits < 8, x, y)
        v = np.where(bits < 4, y, np.where((bits == 12) | (bits == 14), x, z))
        return np.where((bits & 1) == 0, u, -u) + np.where((bits & 2) == 0, v, -v)

    def fractal(
        self,
        x: np.ndarray,
        y: np.ndarray,
        z: np.ndarray,
        complexity: float,
        attenuation: float,
        octaves: int,
    ) -> np.ndarray:
        """Sum ``octaves`` of noise. Octave ``k`` has frequency ``2^k``."""
        field = np.zeros(np.broadcast(x, y, z).shape, dtype=np.float64)
        for octave in range(1, int(octaves) + 1):
            frequency = float(2**octave)
            field += (attenuation / octave) * self.noise(
                frequency * x * complexity,
                frequency * y * complexity,
                frequency * z * complexity,
            )
        return field

    @staticmethod
    def occupy(field: np.ndarray, fill: float) -> np.ndarray:
        """True where ``field`` is above the ``(1 - fill)`` quantile.

        Raises:
            ValueError: ``fill`` is outside ``(0, 1)``.
        """
        fraction = float(fill)
        if not 0.0 < fraction < 1.0:
            raise ValueError("scenario.fill must be in (0, 1)")
        flat = np.sort(np.asarray(field, dtype=np.float64).ravel())
        index = min(max(int(flat.size * (1.0 - fraction)), 0), flat.size - 1)
        return field > flat[index]
