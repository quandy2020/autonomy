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

"""Protobuf generation and automsgs type registry."""

from __future__ import annotations

import importlib
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, Type

from google.protobuf.message import Message

from autonomy.tools.bag_convert.bag_convert_config import BagConvertConfig


class ProtoRegistry:
    """Generate automsgs/autolink protobuf modules and resolve ROS type names."""

    def __init__(self, config: BagConvertConfig | None = None) -> None:
        self._config = config or BagConvertConfig.create_default()
        self._stamp_file = self._config.proto_gen_dir / ".proto_stamp"
        self._registry: Dict[str, Type[Message]] | None = None
        self._by_name: Dict[str, Type[Message]] | None = None
        self._automsgs_roots: tuple[Path, ...] = ()
        self._import_ready = False

    def setup_proto_import_path(self) -> None:
        if self._import_ready:
            return
        self._ensure_autolink_pb2()
        self._ensure_automsgs_pb2()
        self._automsgs_roots = (self._config.proto_gen_dir,)

        proto_gen_str = str(self._config.proto_gen_dir)
        if proto_gen_str not in sys.path:
            sys.path.insert(0, proto_gen_str)
        self._import_ready = True

    def list_supported_ros_types(self) -> tuple[str, ...]:
        self.setup_proto_import_path()
        if self._registry is None:
            by_ros: Dict[str, Type[Message]] = {}
            by_name: Dict[str, Type[Message]] = {}
            for root in self._automsgs_roots:
                msgs_root = root / "automsgs" / "msgs"
                if not msgs_root.is_dir():
                    continue
                for pb2_path in sorted(msgs_root.glob("*/*_pb2.py")):
                    pkg = pb2_path.parent.name
                    module_name = f"automsgs.msgs.{pkg}.{pb2_path.stem}"
                    try:
                        module = importlib.import_module(module_name)
                    except Exception:
                        continue
                    for desc in module.DESCRIPTOR.message_types_by_name.values():
                        cls = getattr(module, desc.name)
                        by_name.setdefault(desc.name, cls)
                        for ros_type in (
                            f"{pkg}/{desc.name}",
                            f"{pkg}/msg/{desc.name}",
                        ):
                            by_ros.setdefault(ros_type, cls)
            self._registry, self._by_name = by_ros, by_name
        return tuple(sorted(self._registry))

    def resolve_proto_class(self, ros_type: str) -> Type[Message]:
        self.list_supported_ros_types()
        assert self._registry is not None and self._by_name is not None
        if ros_type in self._registry:
            return self._registry[ros_type]
        message_name = ros_type.rsplit("/", 1)[-1]
        if message_name in self._by_name:
            return self._by_name[message_name]
        raise KeyError(f"no automsgs proto for ROS type: {ros_type}")

    def _ensure_autolink_pb2(self) -> None:
        proto_gen = self._config.proto_gen_dir
        stamp = "\n".join(self._config.autolink_record_proto_relpaths)
        record_pb2 = proto_gen / "autolink/proto/record_pb2.py"
        stamp_file = proto_gen / ".autolink_stamp"
        stale = (
            not stamp_file.exists()
            or stamp_file.read_text(encoding="utf-8") != stamp
            or not record_pb2.exists()
        )
        if not stale:
            return
        proto_gen.mkdir(parents=True, exist_ok=True)
        print("Generating autolink record protobuf Python modules...")
        subprocess.check_call(
            [
                sys.executable,
                "-m",
                "grpc_tools.protoc",
                f"--python_out={proto_gen}",
                f"-I{self._config.autolink_include}",
                *[str(path) for path in self._config.autolink_record_proto_paths],
            ]
        )
        self._write_pkg_inits(proto_gen)
        stamp_file.write_text(stamp, encoding="utf-8")

    def _ensure_automsgs_pb2(self) -> None:
        proto_dir = self._config.automsgs_proto_dir
        msgs_dir = proto_dir / "msgs"
        if not msgs_dir.is_dir():
            raise FileNotFoundError(f"automsgs proto tree not found: {msgs_dir}")

        proto_gen = self._config.proto_gen_dir
        include_dir = proto_gen / "proto_include"
        packages = self._config.automsgs_compile_packages
        sources: list[Path] = []
        for pkg in packages:
            pkg_dir = msgs_dir / pkg
            if pkg_dir.is_dir():
                sources.extend(sorted(pkg_dir.glob("*.proto")))
        stamp = "\n".join(
            str(path.relative_to(self._config.repo_root)) for path in sources
        )
        imu_pb2 = proto_gen / "automsgs/msgs/sensor_msgs/imu_pb2.py"
        stale = (
            not self._stamp_file.exists()
            or self._stamp_file.read_text(encoding="utf-8") != stamp
            or not imu_pb2.exists()
        )
        if not stale:
            return

        if include_dir.exists():
            shutil.rmtree(include_dir)
        dest_msgs = include_dir / "automsgs" / "msgs"
        dest_msgs.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(msgs_dir, dest_msgs)

        print("Generating automsgs protobuf Python modules...")
        proto_gen.mkdir(parents=True, exist_ok=True)
        compile_files = [
            path
            for path in sorted((include_dir / "automsgs" / "msgs").glob("*/*.proto"))
            if path.parent.name in packages
        ]
        subprocess.check_call(
            [
                sys.executable,
                "-m",
                "grpc_tools.protoc",
                f"--python_out={proto_gen}",
                f"-I{include_dir}",
                *[str(path) for path in compile_files],
            ]
        )
        self._write_pkg_inits(proto_gen)
        self._stamp_file.write_text(stamp, encoding="utf-8")

    @staticmethod
    def _write_pkg_inits(root: Path) -> None:
        for path in sorted(root.rglob("*")):
            if path.is_dir() and not (path / "__init__.py").exists():
                (path / "__init__.py").write_text(
                    "# generated package marker\n", encoding="utf-8"
                )
