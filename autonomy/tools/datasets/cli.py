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
import time
from pathlib import Path

from autonomy.tools.datasets.publisher import (
    DatasetPublisher,
    PublishChannels,
    PublishFrames,
)
from autonomy.tools.datasets.readers.spires import SpiresReader


class DatasetsCli:
    """Parse arguments and run dataset info / publish."""

    def run_main(self, argv: list[str] | None = None) -> int:
        parser = argparse.ArgumentParser(
            prog="python -m autonomy.tools.datasets",
            description=(
                "Load datasets and publish /imu /points [/camera] over autolink "
                "for Argus (localization_argus)."
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
        print(json.dumps(reader.info(), indent=2, sort_keys=True))
        return 0

    def _cmd_publish(self, args: argparse.Namespace) -> int:
        publish_image = not args.no_image
        reader = self._open_reader(args, load_images=publish_image)
        info = reader.info()
        print(
            f"[datasets] spires raw={info['raw_dir']} "
            f"lidar={info['lidar_frames']} imu={info['imu_samples']} "
            f"images={info['image_frames']} dur={info['duration_sec']:.1f}s",
            flush=True,
        )
        print(
            f"[datasets] publish → {args.imu_topic} {args.points_topic}"
            + (f" {args.image_topic}" if publish_image else " (no image)")
            + f" rate={args.rate}",
            flush=True,
        )

        last_print = [0.0]

        def on_progress(published: int, total: int, t: float) -> None:
            if args.quiet:
                return
            now = time.time()
            if published == 1 or published == total or now - last_print[0] >= 1.0:
                last_print[0] = now
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
            ),
            frames=PublishFrames(
                imu=args.imu_frame,
                lidar=args.lidar_frame,
                camera=args.camera_frame,
            ),
            rate=args.rate,
            publish_image=publish_image,
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
