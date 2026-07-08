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

"""Repository paths and format defaults for bag_convert."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class BagConvertConfig:
    """Runtime configuration derived from repository layout."""

    tool_dir: Path
    repo_root: Path
    proto_gen_dir: Path
    commsgs_proto_dir: Path
    autolink_include: Path
    autolink_proto_dir: Path
    requirements_file: Path
    presets_dir: Path
    record_suffix: str = ".record"
    topic_remap_sep: str = ":="
    ros_bag_store: str = "ROS1_NOETIC"
    autolink_record_proto_names: tuple[str, ...] = ("record.proto", "proto_desc.proto")

    @classmethod
    def create_default(cls) -> BagConvertConfig:
        tool_dir = Path(__file__).resolve().parent
        repo_root = tool_dir.parents[2]
        return cls(
            tool_dir=tool_dir,
            repo_root=repo_root,
            proto_gen_dir=tool_dir / ".proto_gen",
            commsgs_proto_dir=repo_root / "autonomy/commsgs/proto",
            autolink_include=repo_root / "autolink",
            autolink_proto_dir=repo_root / "autolink/autolink/proto",
            requirements_file=tool_dir / "requirements.txt",
            presets_dir=tool_dir / "presets",
        )

    @property
    def autolink_record_proto_paths(self) -> tuple[Path, ...]:
        missing = [
            name
            for name in self.autolink_record_proto_names
            if not (self.autolink_proto_dir / name).is_file()
        ]
        if missing:
            raise FileNotFoundError(
                f"missing autolink record protos under {self.autolink_proto_dir}: "
                f"{', '.join(missing)}"
            )
        return tuple(self.autolink_proto_dir / name for name in self.autolink_record_proto_names)

    @property
    def autolink_record_proto_relpaths(self) -> tuple[str, ...]:
        return tuple(f"autolink/proto/{name}" for name in self.autolink_record_proto_names)
