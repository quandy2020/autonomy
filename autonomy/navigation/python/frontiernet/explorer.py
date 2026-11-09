# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Closed-loop frontier explorer: depth/RGB -> FrontierNet -> ``/navigation/goal``.

The node is the runtime half of the "FrontierNet style exploration" plan: it
runs the torch model (official RGB-D checkpoint or the DeFM encoder variant),
decodes the frontier map, back-projects the frontier pixels with the depth
image and publishes the nearest reachable frontier as a ``TYPE_POSE`` goal. The
navigation process then seeks that goal, so the loop closes without any change
to the planner.

Run inside the SpaceHero container::

    PYTHONPATH=/workspace/autonomy/install/autonomy/python:\
/workspace/autonomy/install/autonomy/lib/python:\
/workspace/autonomy/build/autonomy/autonomy/navigation/generated/python \
    /opt/venv/bin/python -m frontiernet.explorer --variant official \
        --weights ../conf/models/rgbd_11cls.pth

``--dry-run`` prints the decisions without publishing a goal.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from frontiernet.postprocess import FrontierMap
from frontiernet.version import __version__

__all__ = ["FrontierExplorer", "main"]


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _decode_depth(message) -> Optional[np.ndarray]:
    """Return a float32 metres image, or None for unsupported encodings."""
    height, width = int(message.height), int(message.width)
    if height <= 0 or width <= 0:
        return None
    data = message.data
    if message.encoding == "32FC1":
        array = np.frombuffer(data, dtype=np.float32, count=height * width)
        return array.reshape(height, width).astype(np.float32)
    if message.encoding in ("16UC1", "mono16"):
        array = np.frombuffer(data, dtype=np.uint16, count=height * width)
        return array.reshape(height, width).astype(np.float32) * 0.001
    return None


def _decode_rgb(message) -> Optional[np.ndarray]:
    height, width = int(message.height), int(message.width)
    if height <= 0 or width <= 0:
        return None
    if message.encoding not in ("rgb8", "bgr8"):
        return None
    array = np.frombuffer(message.data, dtype=np.uint8, count=height * width * 3)
    array = array.reshape(height, width, 3)
    if message.encoding == "bgr8":
        array = array[:, :, ::-1]
    return np.ascontiguousarray(array)


class FrontierExplorer:
    """Runs the model on autosim frames and publishes frontier goals."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.depth: Optional[np.ndarray] = None
        self.rgb: Optional[np.ndarray] = None
        self.intrinsics: Optional[Tuple[float, float, float, float]] = None
        self.pose: Optional[Tuple[float, float, float]] = None
        self.depth_time = 0.0
        self.rgb_time = 0.0
        self.pose_time = 0.0
        self.last_goal: Optional[Tuple[float, float]] = None
        self.last_goal_time = 0.0
        self.goals = 0
        self.ticks = 0
        self.started = time.monotonic()
        self.model = self._load_model()
        self.predictor = self._build_predictor()
        import autolink
        from automsgs.msgs.nav_msgs import odometry_pb2
        from automsgs.msgs.sensor_msgs import camera_info_pb2, image_pb2
        from navigation.proto import navigation_pb2

        self._navigation_pb2 = navigation_pb2
        autolink.init("frontiernet_explorer")
        self.node = autolink.Node("frontiernet_explorer")
        self.node.create_reader(args.depth_topic, self._on_depth, data_type=image_pb2.Image)
        self.node.create_reader(args.rgb_topic, self._on_rgb, data_type=image_pb2.Image)
        self.node.create_reader(
            args.camera_info_topic, self._on_camera_info, data_type=camera_info_pb2.CameraInfo
        )
        self.node.create_reader(args.odom_topic, self._on_odom, data_type=odometry_pb2.Odometry)
        self.writer = self.node.create_writer(args.goal_topic, navigation_pb2.GoalSpec)
        self.timer = autolink.Timer(max(1, int(1000.0 / args.rate)), self._tick, False)

    # -- model ------------------------------------------------------------

    def _load_model(self):
        import torch

        from frontiernet.checkpoint import load_frontiernet_heads, read_state_dict

        torch.set_num_threads(self.args.threads)
        if self.args.variant == "official":
            from frontiernet.unet import TwoHeadUnet

            if not self.args.weights:
                raise ValueError("--variant official needs --weights")
            state = read_state_dict(self.args.weights)
            model = TwoHeadUnet(classes=self.args.classes, in_channels=4)
            model.load_state_dict(state, strict=True)
            return model.eval()
        from frontiernet.model import FrontierNetDeFM

        model = FrontierNetDeFM(
            classes=self.args.classes,
            defm_weights=self.args.defm_weights,
            input_size=self.args.input_size,
        )
        if self.args.weights:
            load_frontiernet_heads(model, self.args.weights).raise_for_errors()
        return model.eval()

    def _build_predictor(self):
        from frontiernet.infer import FrontierPredictor

        return FrontierPredictor(
            self.model,
            device=self.args.device,
            input_size=self.args.input_size,
            classes=self.args.classes,
            df_normalizer=self.args.df_normalizer,
            df_threshold=self.args.df_threshold,
            scale_factor=self.args.scale,
        )

    # -- callbacks --------------------------------------------------------

    def _on_depth(self, message) -> None:
        array = _decode_depth(message)
        if array is not None:
            self.depth = array
            self.depth_time = time.monotonic()

    def _on_rgb(self, message) -> None:
        array = _decode_rgb(message)
        if array is not None:
            self.rgb = array
            self.rgb_time = time.monotonic()

    def _on_camera_info(self, message) -> None:
        if len(message.k) < 9 or message.k[0] <= 0.0 or message.k[4] <= 0.0:
            return
        self.intrinsics = (message.k[0], message.k[4], message.k[2], message.k[5])

    def _on_odom(self, message) -> None:
        # PoseWithCovariance.pose is a PoseStamped, so the Pose is one level deeper.
        pose = message.pose.pose.pose
        yaw = _yaw_from_quaternion(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        self.pose = (pose.position.x, pose.position.y, yaw)
        self.pose_time = time.monotonic()

    # -- projection -------------------------------------------------------

    def _model_to_source(self, u: int, v: int) -> Optional[Tuple[int, int]]:
        """Map a model pixel back to the source image, or None inside padding."""
        if self.depth is None:
            return None
        height, width = self.depth.shape
        scale = self.args.scale
        size = self.args.input_size
        resized_w = max(1, int(width * scale))
        resized_h = max(1, int(height * scale))
        left = max(0, (resized_w - size) // 2)
        top = max(0, (resized_h - size) // 2)
        crop_w = min(size, resized_w)
        crop_h = min(size, resized_h)
        paste_x = max(0, (size - resized_w) // 2)
        paste_y = max(0, (size - resized_h) // 2)
        if u < paste_x or u >= paste_x + crop_w or v < paste_y or v >= paste_y + crop_h:
            return None
        source_x = int((u - paste_x + left) / scale)
        source_y = int((v - paste_y + top) / scale)
        return min(width - 1, max(0, source_x)), min(height - 1, max(0, source_y))

    def _project(self, u: int, v: int) -> Optional[Tuple[float, float]]:
        if self.depth is None or self.intrinsics is None or self.pose is None:
            return None
        source = self._model_to_source(u, v)
        if source is None:
            return None
        depth = float(self.depth[source[1], source[0]])
        if not math.isfinite(depth) or depth < self.args.min_depth or depth > self.args.max_depth:
            return None
        fx, fy, cx, cy = self.intrinsics
        x_optical = (source[0] + 0.5 - cx) * depth / fx
        x_base = depth
        y_base = -x_optical
        x, y, yaw = self.pose
        return (
            x + math.cos(yaw) * x_base - math.sin(yaw) * y_base,
            y + math.sin(yaw) * x_base + math.cos(yaw) * y_base,
        )

    # -- decision ---------------------------------------------------------

    def _clusters(self, frontier: FrontierMap) -> List[Dict[str, float]]:
        region = frontier.region
        gain = frontier.gain
        if region.ndim != 2 or gain.shape != region.shape:
            return []
        size_y, size_x = region.shape
        visited = np.zeros_like(region, dtype=bool)
        clusters: List[Dict[str, float]] = []
        for v in range(size_y):
            for u in range(size_x):
                if visited[v, u] or not region[v, u] or gain[v, u] < self.args.min_gain:
                    continue
                stack = [(u, v)]
                visited[v, u] = True
                sum_x = sum_y = weight = 0.0
                pixels = 0
                while stack:
                    cu, cv = stack.pop()
                    point = self._project(cu, cv)
                    if point is not None:
                        w = max(1.0, float(gain[cv, cu]))
                        sum_x += w * point[0]
                        sum_y += w * point[1]
                        weight += w
                        pixels += 1
                    for du, dv in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        nu, nv = cu + du, cv + dv
                        if nu < 0 or nv < 0 or nu >= size_x or nv >= size_y:
                            continue
                        if visited[nv, nu] or not region[nv, nu]:
                            continue
                        if gain[nv, nu] < self.args.min_gain:
                            continue
                        visited[nv, nu] = True
                        stack.append((nu, nv))
                if pixels >= self.args.min_cluster_pixels and weight > 0.0:
                    clusters.append(
                        {"x": sum_x / weight, "y": sum_y / weight, "pixels": float(pixels)}
                    )
        return clusters

    def _select_target(
        self, frontier: FrontierMap, pose: Optional[Tuple[float, float, float]] = None
    ) -> Optional[Tuple[float, float, float]]:
        pose = self.pose if pose is None else pose
        if pose is None:
            return None
        x, y, _ = pose
        candidates = []
        for cluster in self._clusters(frontier):
            distance = math.hypot(cluster["x"] - x, cluster["y"] - y)
            if distance < self.args.min_range or distance > self.args.max_range:
                continue
            if distance > self.args.target_distance:
                ratio = self.args.target_distance / distance
                cluster = {
                    "x": x + (cluster["x"] - x) * ratio,
                    "y": y + (cluster["y"] - y) * ratio,
                    "pixels": cluster["pixels"],
                }
                distance = self.args.target_distance
            candidates.append((distance, cluster))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        distance, best = candidates[0]
        yaw = math.atan2(best["y"] - y, best["x"] - x)
        return best["x"], best["y"], yaw

    def _fresh(self) -> bool:
        now = time.monotonic()
        if now - self.depth_time > self.args.stale_s:
            return False
        if now - self.pose_time > self.args.stale_s:
            return False
        if self.args.variant == "official" and now - self.rgb_time > self.args.stale_s:
            return False
        return True

    def _publish(self, target: Tuple[float, float, float]) -> None:
        navigation_pb2 = self._navigation_pb2
        goal = navigation_pb2.GoalSpec()
        goal.type = navigation_pb2.GoalSpec.TYPE_POSE
        goal.goal_id = f"frontiernet-{self.goals}"
        goal.sequence = self.goals
        goal.position_tolerance = self.args.position_tolerance
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp.sec = int(time.time())
        goal.pose.pose.position.x = float(target[0])
        goal.pose.pose.position.y = float(target[1])
        goal.pose.pose.orientation.z = math.sin(target[2] * 0.5)
        goal.pose.pose.orientation.w = math.cos(target[2] * 0.5)
        if not self.args.dry_run:
            self.writer.write(goal)
        self.goals += 1
        self.last_goal = (target[0], target[1])
        self.last_goal_time = time.monotonic()

    def _should_stop(self) -> bool:
        if self.args.max_goals > 0 and self.goals >= self.args.max_goals:
            return True
        return self.args.duration > 0 and time.monotonic() - self.started >= self.args.duration

    def _tick(self) -> None:
        if self._should_stop():
            import autolink

            self.timer.stop()
            autolink.shutdown()
            return
        if not self._fresh():
            return
        # Snapshot the pose: reader callbacks run on other threads.
        pose = self.pose
        self.ticks += 1
        started = time.monotonic()
        rgb = self.rgb if self.args.variant == "official" else None
        frontier = self.predictor.predict(rgb=rgb, depth=self.depth)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        target = self._select_target(frontier, pose)
        if target is None:
            if self.args.verbose:
                print(
                    f"[explorer] tick={self.ticks} pixels={int(frontier.region.sum())} "
                    f"no target inference={elapsed_ms:.1f}ms",
                    flush=True,
                )
            return
        x, y = (pose[0], pose[1]) if pose is not None else (0.0, 0.0)
        # Keep one goal until it is reached, then pick the next frontier.
        arrived = (
            self.last_goal is not None
            and math.hypot(x - self.last_goal[0], y - self.last_goal[1])
            <= self.args.goal_tolerance
        )
        # Commit to one frontier long enough to actually drive there: without
        # hysteresis the nearest cluster flips every few ticks while the robot
        # turns, so it rotates in place forever.
        current_goal_distance = (
            math.hypot(x - self.last_goal[0], y - self.last_goal[1])
            if self.last_goal is not None
            else 0.0
        )
        candidate_distance = math.hypot(target[0] - x, target[1] - y)
        better = candidate_distance < current_goal_distance - self.args.switch_margin
        elapsed = (
            time.monotonic() - self.last_goal_time
            if self.last_goal is not None
            else 0.0
        )
        stale = self.last_goal is not None and elapsed > self.args.republish_s and better
        hard_timeout = (
            self.last_goal is not None
            and elapsed > 3.0 * self.args.republish_s
        )
        if self.last_goal is None or arrived or stale or hard_timeout:
            self._publish(target)
        if self.args.verbose:
            print(
                f"[explorer] tick={self.ticks} pixels={int(frontier.region.sum())} "
                f"pose=({x:.2f}, {y:.2f}) target=({target[0]:.2f}, {target[1]:.2f}) "
                f"d={math.hypot(target[0] - x, target[1] - y):.2f} "
                f"inference={elapsed_ms:.1f}ms goals={self.goals}",
                flush=True,
            )

    def run(self) -> int:
        self.timer.start()
        print(
            f"[explorer] variant={self.args.variant} rate={self.args.rate}Hz "
            f"goal_topic={self.args.goal_topic} dry_run={self.args.dry_run}",
            flush=True,
        )
        self.node.spin()
        self.timer.stop()
        print(
            f"[explorer] done ticks={self.ticks} goals={self.goals} "
            f"elapsed={time.monotonic() - self.started:.1f}s",
            flush=True,
        )
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="frontiernet.explorer", description=__doc__)
    parser.add_argument("--version", action="version", version=__version__)
    parser.add_argument("--variant", choices=("official", "defm"), default="official")
    parser.add_argument("--weights", default=None, help="rgbd_11cls.pth")
    parser.add_argument("--defm-weights", default=None, help="defm_vit_s14.pth")
    parser.add_argument("--classes", type=int, default=11)
    parser.add_argument("--input-size", type=int, default=224)
    parser.add_argument("--scale", type=float, default=0.5)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--df-normalizer", type=float, default=10.0)
    parser.add_argument("--df-threshold", type=float, default=0.5)
    parser.add_argument("--min-gain", type=float, default=0.0)
    parser.add_argument("--min-cluster-pixels", type=int, default=12)
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=8.0)
    parser.add_argument("--min-range", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=6.0)
    parser.add_argument("--target-distance", type=float, default=2.0)
    parser.add_argument("--position-tolerance", type=float, default=0.25)
    parser.add_argument("--goal-tolerance", type=float, default=0.5)
    parser.add_argument(
        "--switch-margin",
        type=float,
        default=1.0,
        help="Only switch goals when the new frontier is this much closer",
    )
    parser.add_argument("--republish-s", type=float, default=15.0)
    parser.add_argument("--stale-s", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--max-goals", type=int, default=0)
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--rgb-topic", default="/camera/rgb/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--goal-topic", default="/navigation/goal")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    explorer = FrontierExplorer(args)
    try:
        return explorer.run()
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
