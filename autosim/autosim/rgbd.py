from __future__ import annotations

from typing import Tuple

import numpy as np

from autosim.world import World


class Rgbd:
    def sample(self, world: World) -> Tuple[np.ndarray, np.ndarray]:
        return world.rgb_depth()
