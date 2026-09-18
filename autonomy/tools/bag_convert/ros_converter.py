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

"""ROS1 message to automsgs protobuf conversion."""

from __future__ import annotations

from typing import Any, Tuple

from google.protobuf.descriptor import FieldDescriptor
from google.protobuf.message import Message

from autonomy.tools.bag_convert.proto_registry import ProtoRegistry

_MISSING = object()
_FIELD_ALIASES = {
    "sec": ("sec", "secs"),
    "nanosec": ("nanosec", "nsecs", "nsec"),
}


class RosConverter:
    """Reflectively map ROS messages to automsgs protobuf."""

    _INT_TYPES = frozenset(
        {
            FieldDescriptor.TYPE_INT32,
            FieldDescriptor.TYPE_INT64,
            FieldDescriptor.TYPE_SINT32,
            FieldDescriptor.TYPE_SINT64,
            FieldDescriptor.TYPE_SFIXED32,
            FieldDescriptor.TYPE_SFIXED64,
            FieldDescriptor.TYPE_UINT32,
            FieldDescriptor.TYPE_UINT64,
            FieldDescriptor.TYPE_FIXED32,
            FieldDescriptor.TYPE_FIXED64,
            FieldDescriptor.TYPE_ENUM,
        }
    )

    def __init__(self, registry: ProtoRegistry | None = None) -> None:
        self._registry = registry or ProtoRegistry()

    def convert_ros_message(self, ros_type: str, msg: Any) -> Tuple[Message, str]:
        parts = ros_type.split("/")
        if ros_type.startswith("msg/"):
            normalized = ros_type
        elif len(parts) == 3 and parts[1] == "msg":
            normalized = f"{parts[0]}/{parts[2]}"
        else:
            normalized = ros_type

        if "/" in normalized:
            pkg, name = normalized.split("/", 1)
            aliases = (f"{pkg}/{name}", f"{pkg}/msg/{name}")
        else:
            aliases = (normalized,)

        proto_cls = None
        for candidate in aliases:
            try:
                proto_cls = self._registry.resolve_proto_class(candidate)
                break
            except KeyError:
                continue
        if proto_cls is None:
            raise KeyError(f"no automsgs proto for ROS type: {ros_type}")

        proto = proto_cls()
        self._copy_fields(proto, msg)
        return proto, proto.DESCRIPTOR.full_name

    def _copy_fields(self, dst: Message, src: Any) -> None:
        for field in dst.DESCRIPTOR.fields:
            value = self._src_value(src, field.name)
            if value is _MISSING:
                continue
            if field.label == FieldDescriptor.LABEL_REPEATED:
                if value is None:
                    continue
                if field.type == FieldDescriptor.TYPE_MESSAGE:
                    for item in value:
                        self._copy_fields(getattr(dst, field.name).add(), item)
                    continue
                dst_field = getattr(dst, field.name)
                for item in self._iter_values(value):
                    dst_field.append(self._coerce(item, field))
                continue
            if field.type == FieldDescriptor.TYPE_MESSAGE:
                if value is not None:
                    self._copy_fields(getattr(dst, field.name), value)
                continue
            setattr(dst, field.name, self._coerce(value, field))

    def _src_value(self, src: Any, field_name: str) -> Any:
        names = _FIELD_ALIASES.get(field_name, (field_name,))
        for name in names:
            if hasattr(src, name):
                return getattr(src, name)
            if isinstance(src, dict) and name in src:
                return src[name]
        return _MISSING

    def _coerce(self, value: Any, field: FieldDescriptor) -> Any:
        if field.type == FieldDescriptor.TYPE_BYTES:
            return self._to_bytes(value)
        if field.type in (FieldDescriptor.TYPE_FLOAT, FieldDescriptor.TYPE_DOUBLE):
            return float(value)
        if field.type in self._INT_TYPES:
            return int(value)
        if field.type == FieldDescriptor.TYPE_BOOL:
            return bool(value)
        if field.type == FieldDescriptor.TYPE_STRING:
            return str(value)
        return value

    @staticmethod
    def _iter_values(value: Any) -> Any:
        if hasattr(value, "tolist") and not isinstance(value, (bytes, bytearray, str)):
            try:
                return value.tolist()
            except TypeError:
                pass
        return value

    @staticmethod
    def _to_bytes(value: Any) -> bytes:
        if isinstance(value, (bytes, bytearray, memoryview)):
            return bytes(value)
        if hasattr(value, "tobytes"):
            return value.tobytes()
        if isinstance(value, str):
            return value.encode("utf-8")
        return bytes(bytearray(value))
