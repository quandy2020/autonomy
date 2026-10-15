# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

import numpy as np


def flatten_obs(obs: dict) -> np.ndarray:
    lidar = np.asarray(obs["lidar"], dtype=np.float32).ravel()
    goal = np.asarray(obs["goal"], dtype=np.float32).ravel()
    return np.concatenate([lidar, goal], axis=0)
