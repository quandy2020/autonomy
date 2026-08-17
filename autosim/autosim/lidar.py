from __future__ import annotations

import numpy as np

from autosim.world import World


class Lidar:
    def __init__(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_min: float,
        range_max: float,
    ) -> None:
        self.angle_min = float(angle_min)
        self.angle_max = float(angle_max)
        self.num_beams = int(num_beams)
        self.range_min = float(range_min)
        self.range_max = float(range_max)

    @property
    def angle_increment(self) -> float:
        if self.num_beams <= 1:
            return 0.0
        return (self.angle_max - self.angle_min) / float(self.num_beams - 1)

    def sample(self, world: World) -> np.ndarray:
        ranges = world.depth_ring(self.num_beams, self.range_max).astype(np.float32)
        ranges = np.clip(ranges, self.range_min, self.range_max)
        return ranges
