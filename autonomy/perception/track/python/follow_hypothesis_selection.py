# Copyright 2026 The Openbot Authors
"""Objectness-aware selection for follow continuity (mirrors C++ SelectFollowHypothesis)."""

from __future__ import annotations

import math

import numpy as np


def select_follow_hypothesis(
    terminal_xy_m: np.ndarray,  # [N,2]
    trajectory_cost: np.ndarray,  # [N]
    objectness_score: np.ndarray,  # [N]
    objectness_threshold: float = 0.35,
    suppression_angle_deg: float = 15.0,
    previous_target_yaw_rad: float | None = None,
    temporal_consistency_weight: float = 0.25,
) -> int:
    """Select the best lattice hypothesis for human following.

    Returns index into the arrays, or -1 when none are usable.
    """
    hypothesis_count = terminal_xy_m.shape[0]
    if hypothesis_count == 0:
        return -1

    retained_indices = [
        index
        for index in range(hypothesis_count)
        if objectness_score[index] >= objectness_threshold
    ]
    if not retained_indices:
        best_index = int(np.argmax(objectness_score))
        return best_index if objectness_score[best_index] > 0.05 else -1

    def yaw_rad(index: int) -> float:
        return math.atan2(
            float(terminal_xy_m[index, 1]), float(terminal_xy_m[index, 0])
        )

    def wrap_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    retained_indices.sort(
        key=lambda index: float(objectness_score[index]), reverse=True
    )
    survivor_indices: list[int] = []
    suppression_angle_rad = math.radians(suppression_angle_deg)
    for index in retained_indices:
        candidate_yaw = yaw_rad(index)
        if any(
            abs(wrap_angle(candidate_yaw - yaw_rad(survivor)))
            < suppression_angle_rad
            for survivor in survivor_indices
        ):
            continue
        survivor_indices.append(index)
    if not survivor_indices:
        return -1

    best_index = survivor_indices[0]
    best_metric = float("inf")
    for index in survivor_indices:
        metric = float(trajectory_cost[index]) - 0.5 * float(
            objectness_score[index]
        )
        if previous_target_yaw_rad is not None:
            metric += temporal_consistency_weight * abs(
                wrap_angle(yaw_rad(index) - previous_target_yaw_rad)
            )
        if metric < best_metric:
            best_metric = metric
            best_index = index
    return best_index
