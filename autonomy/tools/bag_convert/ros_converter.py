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

"""ROS1 message to autonomy.commsgs protobuf conversion."""

from __future__ import annotations

from typing import Any, Tuple

from google.protobuf.descriptor import FieldDescriptor
from google.protobuf.message import Message

from autonomy.tools.bag_convert.proto_registry import ProtoRegistry


class RosConverter:
    """Reflectively map ROS messages to commsgs protobuf."""

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
            raise KeyError(f"no commsgs proto for ROS type: {ros_type}")

        proto = proto_cls()

        def copy_fields(dst: Message, src: Any) -> None:
            for field in dst.DESCRIPTOR.fields:
                if not hasattr(src, field.name):
                    continue
                value = getattr(src, field.name)
                if field.label == FieldDescriptor.LABEL_REPEATED:
                    if value is None:
                        continue
                    if field.type == FieldDescriptor.TYPE_MESSAGE:
                        for item in value:
                            copy_fields(getattr(dst, field.name).add(), item)
                        continue
                    dst_field = getattr(dst, field.name)
                    for item in value:
                        dst_field.append(self._coerce(item, field))
                    continue
                if field.type == FieldDescriptor.TYPE_MESSAGE:
                    if value is not None:
                        copy_fields(getattr(dst, field.name), value)
                    continue
                setattr(dst, field.name, self._coerce(value, field))

        copy_fields(proto, msg)
        return proto, proto.DESCRIPTOR.full_name

    def _coerce(self, value: Any, field: FieldDescriptor) -> Any:
        if field.type in (FieldDescriptor.TYPE_FLOAT, FieldDescriptor.TYPE_DOUBLE):
            return float(value)
        if field.type in self._INT_TYPES:
            return int(value)
        if field.type == FieldDescriptor.TYPE_BOOL:
            return bool(value)
        if field.type == FieldDescriptor.TYPE_STRING:
            return str(value)
        if field.type == FieldDescriptor.TYPE_BYTES:
            return bytes(value) if isinstance(value, (bytes, bytearray)) else bytes(bytearray(value))
        return value
