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

"""Capture depth/RGB frames from autolink for FrontierNet retraining.

Run inside the ``SpaceHero`` container, where ``autolink`` and ``automsgs``
are importable::

    PYTHONPATH=/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python \
      /opt/venv/bin/python -m frontiernet.collect --out data/frontiernet/frames --count 200

Each saved ``<index>.npz`` holds ``depth`` (metres, ``float32``), ``rgb``
(``uint8``), ``K`` (flattened 3x3), ``stamp`` and, when ``/odom`` is alive,
``pose`` (``x, y, z, qx, qy, qz, qw``).

The capture runs on an ``autolink.Timer`` so every callback executes on the
spin thread; ``autolink.shutdown()`` does not unblock ``Node.spin()`` in this
build, so the process exits through ``os._exit`` once the summary is written.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from typing import Dict, Optional

import numpy as np

__all__ = ["CollectSettings", "collect", "main"]


class CollectSettings:
    """Plain settings object so the module imports without autolink."""

    def __init__(
        self,
        out_dir: str,
        count: int = 200,
        hz: float = 5.0,
        min_valid_ratio: float = 0.2,
        timeout_s: float = 180.0,
        depth_topic: str = "/camera/depth/image_raw",
        rgb_topic: str = "/camera/rgb/image_raw",
        info_topic: str = "/camera/camera_info",
        odom_topic: str = "/odom",
        node_name: str = "frontiernet_collect",
        quiet: bool = False,
    ) -> None:
        self.out_dir = out_dir
        self.count = int(count)
        self.hz = float(hz)
        self.min_valid_ratio = float(min_valid_ratio)
        self.timeout_s = float(timeout_s)
        self.depth_topic = depth_topic
        self.rgb_topic = rgb_topic
        self.info_topic = info_topic
        self.odom_topic = odom_topic
        self.node_name = node_name
        self.quiet = bool(quiet)


def _decode_image(message, dtype) -> np.ndarray:
    channels = 3 if message.encoding in ("rgb8", "bgr8") else 1
    array = np.frombuffer(message.data, dtype=dtype)
    if channels == 3:
        array = array.reshape(message.height, message.width, channels)
        if message.encoding == "bgr8":
            array = array[:, :, ::-1]
    else:
        array = array.reshape(message.height, message.width)
    return np.ascontiguousarray(array)


def collect(settings: CollectSettings) -> Dict[str, object]:
    """Block until ``count`` frames are saved, then exit the process.

    ``autolink`` only delivers messages while ``Node.spin()`` runs, and this
    build's ``shutdown()`` does not return from ``spin()``, so the capture ends
    with ``os._exit`` after the summary is flushed to disk and stdout.
    """
    import autolink
    from automsgs.msgs.nav_msgs import odometry_pb2
    from automsgs.msgs.sensor_msgs import camera_info_pb2, image_pb2

    os.makedirs(settings.out_dir, exist_ok=True)
    state: Dict[str, object] = {
        "depth": None,
        "rgb": None,
        "K": None,
        "pose": None,
        "stamp": 0.0,
        "depth_seq": 0,
        "last_saved_seq": -1,
        "saved": 0,
        "skipped": 0,
        "depth_messages": 0,
        "rgb_messages": 0,
    }
    started = time.time()

    def on_depth(message) -> None:
        state["depth"] = _decode_image(message, np.float32)
        state["stamp"] = float(message.header.stamp.sec) + 1e-9 * float(
            message.header.stamp.nanosec
        )
        state["depth_seq"] = int(state["depth_seq"]) + 1
        state["depth_messages"] = int(state["depth_messages"]) + 1

    def on_rgb(message) -> None:
        state["rgb"] = _decode_image(message, np.uint8)
        state["rgb_messages"] = int(state["rgb_messages"]) + 1

    def on_info(message) -> None:
        state["K"] = np.asarray(list(message.k), dtype=np.float64)

    def on_odom(message) -> None:
        # automsgs nests geometry_msgs.Pose one level deeper than ROS does:
        # Odometry.pose -> PoseWithCovariance.pose -> Pose(header, pose) -> Pose
        pose = message.pose.pose
        if hasattr(pose, "pose"):
            pose = pose.pose
        state["pose"] = np.asarray(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=np.float64,
        )

    def summary() -> Dict[str, object]:
        return {
            "out_dir": os.path.abspath(settings.out_dir),
            "saved": int(state["saved"]),
            "skipped": int(state["skipped"]),
            "depth_messages": int(state["depth_messages"]),
            "rgb_messages": int(state["rgb_messages"]),
            "elapsed_s": round(time.time() - started, 2),
            "hz": settings.hz,
            "count_requested": settings.count,
        }

    def finish() -> None:
        payload = summary()
        with open(
            os.path.join(settings.out_dir, "collect.json"), "w", encoding="utf-8"
        ) as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
        print(json.dumps(payload, indent=2, sort_keys=True), flush=True)
        os._exit(0 if payload["saved"] > 0 else 1)

    def tick() -> None:
        if int(state["saved"]) >= settings.count:
            finish()
            return
        depth = state["depth"]
        intrinsics = state["K"]
        if depth is None or intrinsics is None:
            return
        if int(state["depth_seq"]) == int(state["last_saved_seq"]):
            return
        valid = np.isfinite(depth) & (depth > 0.0)
        if float(valid.mean()) < settings.min_valid_ratio:
            state["skipped"] = int(state["skipped"]) + 1
            return
        state["last_saved_seq"] = int(state["depth_seq"])
        index = int(state["saved"])
        payload = {
            "depth": np.asarray(depth, dtype=np.float32),
            "K": np.asarray(intrinsics, dtype=np.float64),
            "stamp": float(state["stamp"]),
        }
        rgb = state["rgb"]
        if rgb is not None and rgb.shape[:2] == depth.shape[:2]:
            payload["rgb"] = rgb
        if state["pose"] is not None:
            payload["pose"] = np.asarray(state["pose"], dtype=np.float64)
        np.savez_compressed(os.path.join(settings.out_dir, f"{index:05d}.npz"), **payload)
        state["saved"] = index + 1
        if not settings.quiet and int(state["saved"]) % 20 == 0:
            print(
                f"saved {state['saved']}/{settings.count} valid={float(valid.mean()):.2f}",
                flush=True,
            )

    def on_timeout() -> None:
        print(f"timeout after {settings.timeout_s:.0f}s", flush=True)
        finish()

    autolink.init(settings.node_name)
    node = autolink.Node(settings.node_name)
    node.create_reader(settings.depth_topic, on_depth, data_type=image_pb2.Image)
    node.create_reader(settings.rgb_topic, on_rgb, data_type=image_pb2.Image)
    node.create_reader(settings.info_topic, on_info, data_type=camera_info_pb2.CameraInfo)
    node.create_reader(settings.odom_topic, on_odom, data_type=odometry_pb2.Odometry)

    period_ms = max(1, int(round(1000.0 / max(settings.hz, 0.1))))
    tick_timer = autolink.Timer(period_ms, tick)
    tick_timer.start()
    timeout_timer = autolink.Timer(max(1, int(settings.timeout_s * 1000.0)), on_timeout, oneshot=True)
    timeout_timer.start()
    node.spin()
    finish()
    return summary()


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="frontiernet.collect", description="capture frames for retraining"
    )
    parser.add_argument("--out", required=True, help="output directory for frames")
    parser.add_argument("--count", type=int, default=200)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--min-valid-ratio", type=float, default=0.2)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--rgb-topic", default="/camera/rgb/image_raw")
    parser.add_argument("--info-topic", default="/camera/camera_info")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--node-name", default="frontiernet_collect")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)
    collect(
        CollectSettings(
            out_dir=args.out,
            count=args.count,
            hz=args.hz,
            min_valid_ratio=args.min_valid_ratio,
            timeout_s=args.timeout,
            depth_topic=args.depth_topic,
            rgb_topic=args.rgb_topic,
            info_topic=args.info_topic,
            odom_topic=args.odom_topic,
            node_name=args.node_name,
            quiet=args.quiet,
        )
    )
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
