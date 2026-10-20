#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Smoke: load YAML, enable HuNav, run Runner for N steps.
# Usage:
#   PYTHONPATH=... python3 scripts/smoke_hunav.py --config config/fake.yaml --steps 30
#   AUTOSIM_ISAAC_HEADLESS=1 ... python3 scripts/smoke_hunav.py --config config/isaac.yaml --manager --steps 20

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("smoke_hunav")


def main() -> int:
    parser = argparse.ArgumentParser(description="HuNav Runner smoke")
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config" / "fake.yaml",
    )
    parser.add_argument("--steps", type=int, default=30)
    parser.add_argument(
        "--agents",
        type=str,
        default="config/scenarios/hunav/warehouse_agents.yaml",
    )
    parser.add_argument(
        "--manager",
        action="store_true",
        help="Enable habitat.hunav.manager (Isaac attach_hunav)",
    )
    parser.add_argument("--own-motion", action="store_true", default=True)
    args = parser.parse_args()

    from autosim.runtime.runner import Runner
    from autosim.runtime.settings import Config

    cfg = Config.load(args.config)
    habitat = cfg.data["habitat"]
    hunav = dict(habitat.get("hunav") or {})
    hunav["service"] = True
    hunav["publish_agents"] = True
    hunav["agents"] = args.agents
    if args.manager:
        hunav["manager"] = True
        hunav["own_motion"] = bool(args.own_motion)
    habitat["hunav"] = hunav

    scenario = habitat.setdefault("scenario", {})
    ped = dict(scenario.get("pedestrian") or {})
    ped["enabled"] = True
    ped["social"] = True
    ped["agents"] = args.agents
    # Expand arena for warehouse spawns.
    ped.setdefault("x_length", 40.0)
    ped.setdefault("y_length", 40.0)
    scenario["pedestrian"] = ped
    # Re-validate after mutation.
    Config.validate(cfg.data)

    logger.info(
        "smoke config=%s backend=%s manager=%s steps=%d agents=%s",
        args.config,
        habitat.get("backend"),
        hunav.get("manager", False),
        args.steps,
        args.agents,
    )
    runner = Runner(cfg, max_steps=int(args.steps))
    host = getattr(runner, "hunav_host", None)
    codec = getattr(runner, "hunav_codec", None)
    mgr = getattr(runner.simulator, "hunav_manager", None)
    ped_n = int(runner.traffic.pedestrian.position.shape[0])
    logger.info(
        "bound: host=%s codec=%s manager=%s pedestrians=%d skip_ped=%s",
        host is not None and getattr(host, "available", False),
        codec is not None and getattr(codec, "available", False),
        mgr is not None,
        ped_n,
        getattr(runner.traffic, "skip_pedestrian_step", False),
    )
    if ped_n == 0:
        logger.error("no pedestrians spawned — check agents YAML")
        return 2
    before = runner.traffic.pedestrian.position.copy()
    runner.run()
    after = runner.traffic.pedestrian.position
    delta = float(((after - before) ** 2).sum() ** 0.5)
    logger.info("position L2 delta after %d steps: %.4f", args.steps, delta)
    if delta < 1e-4:
        logger.warning("agents barely moved (ok if own_motion=false and static goals)")
    logger.info("smoke_hunav OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
