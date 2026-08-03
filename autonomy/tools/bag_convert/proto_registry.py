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

"""Protobuf generation and commsgs type registry."""

from __future__ import annotations

import importlib
import subprocess
import sys
from typing import Dict, Type

from google.protobuf.message import Message

from autonomy.tools.bag_convert.bag_convert_config import BagConvertConfig


class ProtoRegistry:
    """Generate commsgs/autolink protobuf modules and resolve ROS type names."""

    def __init__(self, config: BagConvertConfig | None = None) -> None:
        self._config = config or BagConvertConfig.create_default()
        self._stamp_file = self._config.proto_gen_dir / ".proto_stamp"
        self._registry: Dict[str, Type[Message]] | None = None
        self._by_name: Dict[str, Type[Message]] | None = None

    def setup_proto_import_path(self) -> None:
        proto_gen = self._config.proto_gen_dir
        commsgs_stamp = [
            str(path.relative_to(self._config.repo_root))
            for path in sorted(self._config.commsgs_proto_dir.glob("*.proto"))
        ]
        stamp = "\n".join(commsgs_stamp + list(self._config.autolink_record_proto_relpaths))
        record_pb2 = proto_gen / "autolink/proto/record_pb2.py"
        pb2_dir = proto_gen / "autonomy/commsgs/proto"
        stale = (
            not self._stamp_file.exists()
            or self._stamp_file.read_text(encoding="utf-8") != stamp
            or not pb2_dir.is_dir()
            or not any(pb2_dir.glob("*_pb2.py"))
            or not record_pb2.exists()
            or "runtime_version" in record_pb2.read_text(encoding="utf-8")
        )

        if stale:
            if proto_gen.exists():
                import shutil

                shutil.rmtree(proto_gen)
            proto_gen.mkdir(parents=True, exist_ok=True)
            print("Generating protobuf Python modules...")
            subprocess.check_call(
                [
                    sys.executable,
                    "-m",
                    "grpc_tools.protoc",
                    f"--python_out={proto_gen}",
                    f"-I{self._config.repo_root}",
                    *[str(path) for path in sorted(self._config.commsgs_proto_dir.glob("*.proto"))],
                ]
            )
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
            for path in sorted(proto_gen.rglob("*")):
                if path.is_dir() and not (path / "__init__.py").exists():
                    (path / "__init__.py").write_text(
                        "# generated package marker\n", encoding="utf-8"
                    )
            self._stamp_file.write_text(stamp, encoding="utf-8")
            self._registry = None
            self._by_name = None

        proto_gen_str = str(proto_gen)
        if proto_gen_str not in sys.path:
            sys.path.insert(0, proto_gen_str)

        import autonomy

        commsgs_root = str(proto_gen / "autonomy")
        if commsgs_root not in list(autonomy.__path__):
            autonomy.__path__.append(commsgs_root)

    def list_supported_ros_types(self) -> tuple[str, ...]:
        self.setup_proto_import_path()
        if self._registry is None:
            by_ros: Dict[str, Type[Message]] = {}
            by_name: Dict[str, Type[Message]] = {}
            for pb2_path in sorted(
                (self._config.proto_gen_dir / "autonomy/commsgs/proto").glob("*_pb2.py")
            ):
                module = importlib.import_module(f"automsgs.msgs.{pb2_path.stem}")
                package = pb2_path.stem.removesuffix("_pb2")
                for desc in module.DESCRIPTOR.message_types_by_name.values():
                    cls = getattr(module, desc.name)
                    by_name.setdefault(desc.name, cls)
                    for ros_type in (f"{package}/{desc.name}", f"{package}/msg/{desc.name}"):
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
        raise KeyError(f"no commsgs proto for ROS type: {ros_type}")
