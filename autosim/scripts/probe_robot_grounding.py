#!/usr/bin/env python3
"""Report each robot's lowest rendered point at its configured home pose.

The Habitat URDF importer decides how a mesh's authored axes map onto the
URDF frame, so the only reliable ground truth for "does the robot float or
sink" is the bounding box Habitat itself builds. This probe loads one robot
per process, reads the articulated object's cumulative bounding box, and
prints the height that would put the lowest vertex on the floor.

Use it after changing ``robot.joint_positions`` or ``robot.root_height``::

    python scripts/probe_robot_grounding.py --profiles google_robot
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autosim.backends.habitat.simulator import Simulator
from autosim.robot.profiles import PROFILES, shipped_robot_defaults
from autosim.robot.viewer import viewer_settings


def bbox_corners(box):
    """Return the eight corners of a Magnum ``Range3D``."""
    return np.array(
        [
            [x, y, z]
            for x in (box.min.x, box.max.x)
            for y in (box.min.y, box.max.y)
            for z in (box.min.z, box.max.z)
        ],
        dtype=float,
    )


def probe(profile: str) -> dict:
    """Load one robot and measure its world bounding box."""
    sim = Simulator.create(viewer_settings(profile, width=320, height=240))
    try:
        robot = sim.articulated
        if robot is None:
            raise RuntimeError("URDF was not instanced")
        # ``root_scene_node`` only covers the root link; every link owns its
        # own mesh, so union them all to get the whole robot.
        clouds = []
        for link_id in robot.get_link_ids():
            node = robot.get_link_scene_node(link_id)
            corners = bbox_corners(node.compute_cumulative_bb())
            matrix = np.array(node.absolute_transformation_matrix(), dtype=float)
            clouds.append(corners @ matrix[:3, :3].T + matrix[:3, 3])
        world = np.concatenate(clouds, axis=0)
        # Habitat is y-up: the floor plane is y = floor_y.
        lowest = float(world[:, 1].min())
        floor = float(sim.floor_y)
        root_height = float(sim.root_height())
        return {
            "profile": profile,
            "root_height": round(root_height, 4),
            "floor_y": round(floor, 4),
            "lowest_above_floor": round(lowest - floor, 4),
            "suggested_root_height": round(root_height + (floor - lowest), 4),
            "aabb_min": [round(float(v), 4) for v in world.min(axis=0)],
            "aabb_max": [round(float(v), 4) for v in world.max(axis=0)],
        }
    finally:
        sim.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profiles",
        nargs="+",
        default=[
            name
            for name, profile in PROFILES.items()
            if profile.articulated or shipped_robot_defaults(name).get("instance")
        ],
    )
    args = parser.parse_args()
    if len(args.profiles) > 1:
        # One Habitat session per process: GL contexts do not survive a close.
        for name in args.profiles:
            process = subprocess.run(
                [sys.executable, __file__, "--profiles", name],
                capture_output=True,
                text=True,
            )
            line = (process.stdout or "").strip().splitlines()
            if process.returncode == 0 and line:
                print(line[-1], flush=True)
            else:
                print(
                    json.dumps(
                        {
                            "profile": name,
                            "ok": False,
                            "exit_code": process.returncode,
                            "error": (process.stderr or "").strip()[-400:],
                        }
                    ),
                    flush=True,
                )
        return
    try:
        row = probe(args.profiles[0])
        row["ok"] = True
    except Exception as exc:
        row = {"profile": args.profiles[0], "ok": False, "error": repr(exc)}
    print(json.dumps(row), flush=True)
    if not row["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
