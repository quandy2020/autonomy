"""Grid A* expert used to label training episodes."""

from __future__ import annotations

import argparse
import heapq
import math

import numpy as np
import yaml


def astar(grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    height, width = grid.shape
    open_set = [(0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start: 0.0}
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < width and 0 <= neighbor[1] < height):
                continue
            if grid[neighbor[1], neighbor[0]] != 0:
                continue
            step = math.hypot(dx, dy)
            candidate = cost[current] + step
            if candidate < cost.get(neighbor, float("inf")):
                cost[neighbor] = candidate
                came_from[neighbor] = current
                heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                heapq.heappush(open_set, (candidate + heuristic, neighbor))
    return []


def path_to_actions(path: list[tuple[int, int]], resolution: float = 0.1,
                    max_speed: float = 0.5) -> np.ndarray:
    actions = []
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        dx = (x1 - x0) * resolution
        dy = (y1 - y0) * resolution
        distance = math.hypot(dx, dy)
        actions.append([min(max_speed, distance / 0.1), 0.0, math.atan2(dy, dx)])
    return np.asarray(actions, dtype=np.float32)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/train_vntpa.yaml")
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    print("expert config:", config.get("data", {}).get("grid", "grid"))


if __name__ == "__main__":
    main()
