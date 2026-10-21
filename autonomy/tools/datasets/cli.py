# Copyright 2026 The Openbot Authors (duyongquan)
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

"""CLI for autonomy.tools.datasets."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from autonomy.tools.datasets.publisher import (
    DatasetPublisher,
    PublishChannels,
    PublishFrames,
)
from autonomy.tools.datasets.readers.spires import SpiresReader
from autonomy.tools.datasets.tum import resolve_spires_gt


class DatasetsCli:
    """Parse arguments and run dataset info / publish."""

    def run_main(self, argv: list[str] | None = None) -> int:
        parser = argparse.ArgumentParser(
            prog="python -m autonomy.tools.datasets",
            description=(
                "Load datasets and publish /imu /points [/camera] [/gt/odometry] "
                "over autolink for Argus (localization_argus)."
            ),
        )
        sub = parser.add_subparsers(dest="command", required=True)

        info = sub.add_parser("info", help="Print dataset summary as JSON")
        self._add_common_reader_args(info)

        pub = sub.add_parser(
            "publish",
            help="Publish sensor streams on autolink (Argus subscribe)",
        )
        self._add_common_reader_args(pub)
        pub.add_argument(
            "--rate",
            type=float,
            default=1.0,
            help="Playback rate vs dataset time (0 = as fast as possible)",
        )
        pub.add_argument("--start", type=int, default=0, help="First lidar frame index")
        pub.add_argument(
            "--max-frames",
            type=int,
            default=None,
            help="Max lidar frames to publish",
        )
        pub.add_argument(
            "--loop",
            action="store_true",
            help="Repeat the sequence until interrupted",
        )
        pub.add_argument(
            "--no-image",
            action="store_true",
            help="Do not publish camera images (LIO-only)",
        )
        pub.add_argument("--imu-topic", default="/imu")
        pub.add_argument("--points-topic", default="/points")
        pub.add_argument(
            "--image-topic",
            default="/camera/rgb/image_raw",
            help="Must match --argus_image_topic (localization_argus.launch)",
        )
        pub.add_argument("--imu-frame", default="imu_link")
        pub.add_argument("--lidar-frame", default="lidar_link")
        pub.add_argument("--camera-frame", default="camera_link")
        pub.add_argument(
            "--node-name",
            default="dataset_publisher",
            help="autolink node name",
        )
        pub.add_argument(
            "--quiet",
            action="store_true",
            help="Suppress progress prints",
        )
        gt = pub.add_mutually_exclusive_group()
        gt.add_argument(
            "--gt",
            dest="publish_gt",
            action="store_true",
            default=True,
            help="Publish ground truth when a TUM file is found (default)",
        )
        gt.add_argument(
            "--no-gt",
            dest="publish_gt",
            action="store_false",
            help="Do not publish ground truth",
        )
        pub.add_argument(
            "--gt-file",
            type=Path,
            default=None,
            help="TUM GT path (default: processed/vilens-slam/gt.tum or trajectory/gt-tum.txt)",
        )
        pub.add_argument(
            "--gt-topic",
            default="/gt/odometry",
            help="GT Odometry topic (synced to lidar time)",
        )
        pub.add_argument(
            "--gt-path-topic",
            default="/gt/path",
            help="GT Path topic (grows with lidar time, synced to playback)",
        )
        pub.add_argument("--gt-frame", default="map", help="GT header.frame_id")
        pub.add_argument(
            "--gt-child-frame",
            default="gt_link",
            help="GT Odometry child_frame_id",
        )
        pub.add_argument(
            "--gt-max-dt",
            type=float,
            default=0.5,
            help="Max |t_gt - t_lidar| for association (sec)",
        )
        pub.add_argument(
            "--gt-path-stride",
            type=int,
            default=1,
            help="Stride when packing /gt/path (1 = all poses)",
        )
        pub.add_argument(
            "--gt-path-every",
            type=int,
            default=1,
            help="Also republish growing /gt/path every N frames if unchanged (late Autoviz)",
        )
        pub.add_argument(
            "--gt-zero-start",
            action="store_true",
            help="DEBUG only: rebase GT so first pose is identity (cheating for overlay; default off = absolute TUM)",
        )

        args = parser.parse_args(argv)
        if args.command == "info":
            return self._cmd_info(args)
        if args.command == "publish":
            return self._cmd_publish(args)
        parser.error(f"unknown command {args.command}")
        return 2

    @staticmethod
    def _add_common_reader_args(parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--dataset",
            choices=("spires",),
            default="spires",
            help="Dataset layout (default: oxford spires)",
        )
        parser.add_argument(
            "--path",
            required=True,
            type=Path,
            help="Sequence root or .../raw directory",
        )
        parser.add_argument(
            "--camera",
            default="cam0",
            help="Spires camera folder (cam0/cam1/cam2)",
        )
        parser.add_argument(
            "--max-image-dt",
            type=float,
            default=0.05,
            help="Max |t_img - t_lidar| for association (sec)",
        )
        parser.add_argument(
            "--prefer-undistorted",
            action="store_true",
            help="Prefer processed/vilens-slam/undist-clouds when present",
        )

    def _open_reader(self, args: argparse.Namespace, *, load_images: bool) -> SpiresReader:
        if args.dataset != "spires":
            raise SystemExit(f"unsupported dataset: {args.dataset}")
        return SpiresReader(
            args.path,
            camera=args.camera,
            max_image_dt_sec=args.max_image_dt,
            prefer_undistorted=args.prefer_undistorted,
            load_images=load_images,
        )

    def _cmd_info(self, args: argparse.Namespace) -> int:
        reader = self._open_reader(args, load_images=True)
        info = reader.info()
        gt = resolve_spires_gt(reader.seq_root, getattr(args, "gt_file", None))
        info["gt_file"] = str(gt) if gt is not None else ""
        print(json.dumps(info, indent=2, sort_keys=True))
        return 0

    def _cmd_publish(self, args: argparse.Namespace) -> int:
        publish_image = not args.no_image
        reader = self._open_reader(args, load_images=publish_image)
        info = reader.info()
        gt_path = resolve_spires_gt(reader.seq_root, args.gt_file)
        publish_gt = bool(args.publish_gt) and gt_path is not None
        print(
            f"[datasets] spires raw={info['raw_dir']} "
            f"lidar={info['lidar_frames']} imu={info['imu_samples']} "
            f"images={info['image_frames']} dur={info['duration_sec']:.1f}s",
            flush=True,
        )
        topics = f"{args.imu_topic} {args.points_topic}"
        if publish_image:
            topics += f" {args.image_topic}"
        else:
            topics += " (no image)"
        if publish_gt:
            topics += f" {args.gt_topic} {args.gt_path_topic}"
        elif args.publish_gt:
            topics += " (no GT file)"
        print(f"[datasets] publish → {topics} rate={args.rate}", flush=True)
        if publish_gt:
            print(f"[datasets] gt={gt_path}", flush=True)

        # Progress every N lidar frames (sensor cadence), not wall-clock — so
        # the printed step matches real lidar spacing (~0.1s @ 10 Hz).
        progress_every = 10

        def on_progress(published: int, total: int, t: float) -> None:
            if args.quiet:
                return
            if (
                published == 1
                or published == total
                or published % progress_every == 0
            ):
                print(
                    f"[datasets] frame {published}/{total} t={t:.3f}",
                    flush=True,
                )

        publisher = DatasetPublisher(
            reader,
            channels=PublishChannels(
                imu=args.imu_topic,
                points=args.points_topic,
                image=args.image_topic,
                gt_odom=args.gt_topic,
                gt_path=args.gt_path_topic,
            ),
            frames=PublishFrames(
                imu=args.imu_frame,
                lidar=args.lidar_frame,
                camera=args.camera_frame,
                gt=args.gt_frame,
                gt_child=args.gt_child_frame,
            ),
            rate=args.rate,
            publish_image=publish_image,
            publish_gt=publish_gt,
            gt_file=args.gt_file,
            gt_max_dt=args.gt_max_dt,
            gt_path_stride=args.gt_path_stride,
            gt_path_every=args.gt_path_every,
            gt_zero_start=args.gt_zero_start,
            node_name=args.node_name,
            on_progress=on_progress,
        )
        try:
            n = publisher.run(
                start=args.start,
                max_frames=args.max_frames,
                loop=args.loop,
            )
        except KeyboardInterrupt:
            print("\n[datasets] interrupted", flush=True)
            publisher.shutdown()
            return 130
        print(f"[datasets] done frames={n}", flush=True)
        return 0


def main(argv: list[str] | None = None) -> int:
    return DatasetsCli().run_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
