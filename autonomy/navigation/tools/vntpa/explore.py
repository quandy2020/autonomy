"""Random-policy rollout used to collect exploration episodes."""

from __future__ import annotations

import argparse

import numpy as np
import yaml


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/train_vntpa.yaml")
    parser.add_argument("--steps", type=int, default=100)
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    rng = np.random.default_rng(config["train"].get("seed", 0))
    pose = np.zeros(3, dtype=np.float32)
    for _ in range(args.steps):
        action = rng.uniform(-0.4, 0.4, size=3)
        pose[0] += action[0] * 0.1
        pose[1] += action[1] * 0.1
        pose[2] += action[2] * 0.1
    print("final pose", pose.tolist())


if __name__ == "__main__":
    main()
