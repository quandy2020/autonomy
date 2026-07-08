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

"""Command-line interface for bag_convert."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from autonomy.tools.bag_convert.bag_converter import BagConverter
from autonomy.tools.bag_convert.bag_convert_config import BagConvertConfig
from autonomy.tools.bag_convert.dependency_checker import DependencyChecker
from autonomy.tools.bag_convert.preset_store import PresetStore
from autonomy.tools.bag_convert.proto_registry import ProtoRegistry


class BagConvertCli:
    """Parse arguments and run bag conversion."""

    PRESET_BACKPACK_2D = "backpack_2d"

    def __init__(self, config: BagConvertConfig | None = None) -> None:
        self._config = config or BagConvertConfig.create_default()
        self._converter = BagConverter(self._config)
        self._registry = ProtoRegistry(self._config)
        self._presets = PresetStore(self._config)

    def run_main(self, argv: list[str] | None = None) -> int:
        parser = argparse.ArgumentParser(
            description="Convert ROS1 bag files to autolink .record format."
        )
        parser.add_argument("bags", nargs="*", type=Path, help="Input ROS1 .bag file(s)")
        parser.add_argument(
            "-o",
            "--output",
            type=Path,
            default=None,
            help="Output .record path or directory (default: <bag>.record beside input)",
        )
        parser.add_argument(
            "--leading-slash",
            action="store_true",
            help="Use ROS-style leading '/' on channel names (default: strip for autolink)",
        )
        parser.add_argument("--topics", nargs="*", default=None, help="Optional topic whitelist")
        parser.add_argument(
            "--topic-remap",
            nargs="*",
            default=None,
            help="Optional topic remap rules, e.g. /horizontal_laser_2d:=echoes_1",
        )
        parser.add_argument(
            "--preset",
            default=None,
            help=f"Named preset from presets/<name>.json (e.g. {self.PRESET_BACKPACK_2D})",
        )
        parser.add_argument(
            "--backpack-2d",
            action="store_true",
            help=f"Shortcut for --preset {self.PRESET_BACKPACK_2D}",
        )
        parser.add_argument(
            "--skip-unsupported",
            action="store_true",
            help="Skip ROS types without commsgs proto instead of failing",
        )
        parser.add_argument(
            "--list-types",
            action="store_true",
            help="List ROS types mapped to autonomy.commsgs proto and exit",
        )
        args = parser.parse_args(argv)

        DependencyChecker(self._config).check_runtime_dependencies()
        if args.list_types:
            for ros_type in self._registry.list_supported_ros_types():
                print(ros_type)
            return 0

        if not args.bags:
            print("error: at least one bag file is required", file=sys.stderr)
            return 1

        use_leading_slash = args.leading_slash

        def normalize_topic(topic: str) -> str:
            topic = topic.strip()
            if topic.startswith("/"):
                topic = topic[1:]
            return f"/{topic}" if use_leading_slash else topic

        topic_remap: dict[str, str] = {}
        if args.topic_remap:
            sep = self._config.topic_remap_sep
            for entry in args.topic_remap:
                if sep not in entry:
                    print(f"error: invalid topic remap '{entry}', expected SRC{sep}DST", file=sys.stderr)
                    return 1
                src, dst = entry.split(sep, 1)
                src = normalize_topic(src)
                dst = normalize_topic(dst)
                if not src or not dst:
                    print(f"error: invalid topic remap '{entry}'", file=sys.stderr)
                    return 1
                topic_remap[src] = dst

        topics = list(args.topics) if args.topics else None
        preset_name = self.PRESET_BACKPACK_2D if args.backpack_2d else args.preset
        if preset_name:
            preset_topics, preset_remap = self._presets.load_preset(preset_name)
            topics = list(preset_topics)
            topic_remap = {**preset_remap, **topic_remap}

        suffix = self._config.record_suffix
        for bag_path in args.bags:
            bag_path = bag_path.expanduser().resolve()
            if not bag_path.exists():
                print(f"error: bag not found: {bag_path}", file=sys.stderr)
                return 1

            if args.output is None:
                out_path = bag_path.with_suffix(suffix)
            else:
                output = args.output.resolve()
                out_path = (
                    output
                    if output.suffix == suffix
                    else output / f"{bag_path.stem}{suffix}"
                )
                if output.suffix != suffix:
                    output.mkdir(parents=True, exist_ok=True)
            out_path.parent.mkdir(parents=True, exist_ok=True)

            print(f"Converting {bag_path.name} -> {out_path}")
            try:
                total, counts = self._converter.convert_bag(
                    bag_path,
                    out_path,
                    use_leading_slash,
                    topics,
                    topic_remap,
                    skip_unsupported=args.skip_unsupported,
                )
            except Exception as exc:  # noqa: BLE001
                print(f"error: failed to convert {bag_path}: {exc}", file=sys.stderr)
                return 1

            print(f"  messages: {total}")
            for topic, count in sorted(counts.items()):
                print(f"    {topic}: {count}")

        print("Done.")
        return 0
