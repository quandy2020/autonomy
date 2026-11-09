#!/usr/bin/env python3
"""Controlled Habitat scene using the production autosim bridge and plant.

Only scene geometry, sensor pause, epoch reset and test telemetry are added.
Commands are atomic JSON files written by autosim_acceptance.py.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import yaml

from autosim.bridge.clock import Clock
from autosim.runtime.runner import Runner
from autosim.runtime.settings import Config
from autosim.scene.gen.stage import Stage


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value))
    temporary.replace(path)


class AcceptanceScene(Runner):
    def __init__(self, settings: Config, directory: Path) -> None:
        self.directory = directory
        self.command_id = -1
        self.depth_paused = False
        self.obstacle = None
        self.obstacle_bounds = None
        self.received_commands = 0
        self.collisions = 0
        self.min_clearance = math.inf
        self.last_state = 0.0
        self.map_mode = "none"
        self.explored_grid = np.full((90, 100), -1, dtype=np.int8)
        self.map_frames = 0
        self.map_last_publish = 0.0
        super().__init__(settings)
        self.clock = Clock(0)
        self.robot.last_command_time = 0
        self.trajectory = (directory / "trajectory.jsonl").open("w")

    def warmup_map(self) -> None:
        # This controlled fixture builds its partial map from actual camera
        # observations. Do not load the deployment's unrelated PLY scene.
        pass

    def encode_publish_camera(self, color, depth, stamp, **kwargs) -> None:
        super().encode_publish_camera(color, depth, stamp, **kwargs)
        if self.map_mode != "depth":
            return
        image = np.asarray(depth).squeeze()
        height, width = image.shape
        fx = width / (2 * math.tan(math.radians(float(self.camera.get("hfov_deg", 90))) / 2))
        x, y, yaw = self.robot.pose()
        mount = self.simulator.camera_local_offset()
        forward_offset, left_offset = -mount[2], -mount[0]
        x, y = (x + math.cos(yaw) * forward_offset - math.sin(yaw) * left_offset,
                y + math.sin(yaw) * forward_offset + math.cos(yaw) * left_offset)
        # Apply the actual URDF camera mount to horizontal rays.
        # Horizontal rays use Habitat's real rendered depth, capped at 3 m.
        for u in range(0, width, 2):
            measured = float(np.median(image[height // 2 - 1:height // 2 + 2, u]))
            if not math.isfinite(measured) or measured <= 0.05:
                continue
            lateral = -(u - width / 2) / fx
            distance = min(3.0, measured)
            for forward in np.arange(0, distance, 0.04):
                px = x + forward * (math.cos(yaw) - lateral * math.sin(yaw))
                py = y + forward * (math.sin(yaw) + lateral * math.cos(yaw))
                col, row = int(math.floor((px + 4) / .1)), int(math.floor((py + 4) / .1))
                if 0 <= row < 90 and 0 <= col < 100 and self.explored_grid[row, col] < 0:
                    self.explored_grid[row, col] = 0
            if measured <= 3.0:
                px = x + measured * (math.cos(yaw) - lateral * math.sin(yaw))
                py = y + measured * (math.sin(yaw) + lateral * math.cos(yaw))
                col, row = int(math.floor((px + 4) / .1)), int(math.floor((py + 4) / .1))
                if 0 <= row < 90 and 0 <= col < 100:
                    self.explored_grid[row, col] = 100

    def submit_map(self, stamp) -> None:
        if self.map_mode == "none" or time.monotonic() - self.map_last_publish < .2:
            return
        if self.map_mode == "full":
            grid = np.zeros((90, 100), dtype=np.int8)
            grid[[0, -1], :] = 100
            grid[:, [0, -1]] = 100
            if self.obstacle_bounds is not None:
                x0, y0, x1, y1 = self.obstacle_bounds
                grid[int((y0 + 4) / .1):int((y1 + 4) / .1),
                     int((x0 + 4) / .1):int((x1 + 4) / .1)] = 100
        else:
            grid = self.explored_grid
        self.encode_publish_map_grid(grid, .1, -4, -4, stamp)
        self.map_last_publish = time.monotonic()
        self.map_frames += 1
        if self.map_frames % 5 == 0:
            maps = self.directory / "maps"
            maps.mkdir(exist_ok=True)
            snapshot = maps / f"grid-{self.map_frames:06d}.npy"
            np.save(snapshot, grid)
            metadata = {"resolution": .1, "origin": [-4, -4],
                        "frame": self.map_cfg["frame"], "shape": [90, 100],
                        "source": "Habitat depth rays" if self.map_mode == "depth" else "controlled full room",
                        "map_mode": self.map_mode, "map_frames": self.map_frames,
                        "stamp": stamp, "time": time.monotonic()}
            atomic_json(snapshot.with_suffix(".json"), metadata)
            atomic_json(maps / "metadata.json", metadata)

    def poll_command(self, dt: float = 0.02) -> None:
        if self.bridge.command_reader.has_msg():
            self.received_commands += 1
        super().poll_command(dt)

    def emit(self, name, action, *args) -> None:
        if self.depth_paused and name in {"camera", "depth_points"}:
            return
        super().emit(name, action, *args)

    def set_obstacle(self, bounds) -> None:
        session = self.simulator.session
        manager = session.get_rigid_object_manager()
        if self.obstacle is not None:
            manager.remove_object_by_id(self.obstacle.object_id)
            self.obstacle = None
        self.obstacle_bounds = bounds
        if bounds is None:
            return
        import habitat_sim
        import magnum as mn

        templates = session.get_object_template_manager()
        handles = templates.get_template_handles("cubeSolid")
        if not handles:
            raise RuntimeError("Habitat cubeSolid object template unavailable")
        template = templates.get_template_by_handle(handles[0])
        x0, y0, x1, y1 = bounds
        template.scale = mn.Vector3((x1 - x0) / 2, 0.75, (y1 - y0) / 2)
        template_id = templates.register_template(template, "acceptance_obstacle")
        self.obstacle = manager.add_object_by_template_id(template_id)
        self.obstacle.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        self.obstacle.translation = mn.Vector3((x0 + x1) / 2, 0.75, -(y0 + y1) / 2)

    def process_command(self) -> None:
        path = self.directory / "scene_command.json"
        if not path.exists():
            return
        command = json.loads(path.read_text())
        if command["id"] == self.command_id:
            return
        if "map_mode" in command:
            self.map_mode = command["map_mode"]
            self.explored_grid.fill(-1)
            # Reserve only the currently occupied footprint as known free.
            x, y, _ = command.get("pose", self.robot.pose())
            col, row = int((x + 4) / .1), int((y + 4) / .1)
            self.explored_grid[max(0, row - 4):min(90, row + 5),
                               max(0, col - 4):min(100, col + 5)] = 0
        if "obstacle" in command:
            self.set_obstacle(command["obstacle"])
        if "depth_paused" in command:
            self.depth_paused = bool(command["depth_paused"])
        if "pose" in command:
            self.robot.teleport(*command["pose"])
            self.simulator.reset(*command["pose"])
            self.min_clearance = math.inf
            self.collisions = 0
        if "epoch" in command:
            self.clock = Clock(command["epoch"])
            self.robot.last_command_time = self.clock.now()
        self.command_id = command["id"]

    def cycle(self, dt: float) -> None:
        self.process_command()
        super().cycle(dt)
        x, y, yaw = self.robot.pose()
        # Independent geometry oracle at every plant step. Habitat moves
        # kinematically, so contact response is not used as the collision oracle.
        radius = 0.3  # Same conservative radius as differential_drive.pb.txt.
        clearance = min(x + 3.9, 5.9 - x, y + 3.9, 4.9 - y) - radius
        if self.obstacle_bounds is not None:
            x0, y0, x1, y1 = self.obstacle_bounds
            dx = max(x0 - x, 0, x - x1)
            dy = max(y0 - y, 0, y - y1)
            clearance = min(clearance, math.hypot(dx, dy) - radius)
        self.min_clearance = min(self.min_clearance, clearance)
        if clearance <= 0:
            self.collisions += 1
        now = time.monotonic()
        if now - self.last_state >= 0.04:
            state = {
                "command_id": self.command_id,
                "time": now,
                "stamp": self.clock.now(),
                "pose": [x, y, yaw],
                "velocity": list(self.robot.velocity()),
                "received_commands": self.received_commands,
                "collisions": self.collisions,
                "min_clearance": self.min_clearance,
                "depth_paused": self.depth_paused,
                "map_mode": self.map_mode,
                "map_frames": self.map_frames,
                "known_cells": int((self.explored_grid >= 0).sum()),
            }
            atomic_json(self.directory / "scene_state.json", state)
            self.trajectory.write(json.dumps(state) + "\n")
            self.trajectory.flush()
            self.last_state = now


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--autosim-root", type=Path, required=True)
    args = parser.parse_args()
    args.directory.mkdir(parents=True, exist_ok=True)
    volume = np.zeros((100, 90, 30), dtype=bool)
    volume[[0, -1], :, :] = True
    volume[:, [0, -1], :] = True
    scene = Stage().write(volume, np.array([-4., -4., 0.]), 0.1,
                          args.directory / "room.obj")
    data = yaml.safe_load((args.autosim_root / "config/atlas.yaml").read_text())
    habitat = data["habitat"]
    habitat.update(backend="habitat", mode="nav", path=str(scene), spawn=[0, 0, 0])
    habitat["map"].update(enabled=True, publish=True)
    habitat["map"]["grid"]["resolution"] = .1
    habitat["sensors"]["camera"]["sensor_height"] = .18
    habitat["sensors"]["odom"]["noise"] = 0
    habitat["sensors"]["chase"]["enabled"] = False
    habitat["sensors"]["camera"].update(width=160, height=120, rate_hz=20)
    habitat["sensors"]["imu"]["noise"] = {"gyro": 0., "accel": 0.}
    habitat["robot"]["tf"]["publish_map_odom"] = True
    atomic_json(args.directory / "config.json", data)
    AcceptanceScene(Config.load(data), args.directory).run()


if __name__ == "__main__":
    main()
