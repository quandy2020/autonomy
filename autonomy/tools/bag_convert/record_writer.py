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

"""Write autolink .record files (Cyber RT format)."""

from __future__ import annotations

import os
import struct
from typing import Dict, List

from google.protobuf.descriptor_pb2 import FileDescriptorProto
from google.protobuf.message import Message

_SECTION = struct.Struct("<i4xQ")
_HEADER_LENGTH = 2048
_HEADER_DEFAULTS = {
    "major_version": 1,
    "minor_version": 0,
    "chunk_interval": 20 * 1_000_000_000,
    "segment_interval": 60 * 1_000_000_000,
    "chunk_raw_size": 16 * 1024 * 1024,
    "segment_raw_size": 2 * 1024 * 1024 * 1024,
}


class RecordWriter:
    """Write autolink record files consumable by autolink_recorder play."""

    def __init__(self) -> None:
        from autolink.proto import record_pb2

        self._record_proto = record_pb2
        self._fd: int | None = None
        self._header = record_pb2.Header(
            compress=record_pb2.COMPRESS_NONE,
            **_HEADER_DEFAULTS,
        )
        self._index = record_pb2.Index()
        self._channel_counts: Dict[str, int] = {}
        self._messages: List = []

    def open_record(self, path: str) -> None:
        if os.path.exists(path):
            os.remove(path)
        self._fd = os.open(path, os.O_CREAT | os.O_WRONLY | os.O_TRUNC, 0o644)
        self._write_section(self._record_proto.SECTION_HEADER, self._header)

    def write_channel(self, channel_name: str, message_type: str, proto_desc: bytes) -> None:
        channel = self._record_proto.Channel(
            name=channel_name,
            message_type=message_type,
            proto_desc=proto_desc,
        )
        pos = os.lseek(self._fd, 0, os.SEEK_CUR)
        self._write_section(self._record_proto.SECTION_CHANNEL, channel)
        self._header.channel_number += 1
        idx = self._index.indexes.add()
        idx.type = self._record_proto.SECTION_CHANNEL
        idx.position = pos
        idx.channel_cache.name = channel_name
        idx.channel_cache.message_type = message_type
        idx.channel_cache.proto_desc = proto_desc
        self._channel_counts.setdefault(channel_name, 0)

    def write_message(self, channel_name: str, content: bytes, timestamp_ns: int) -> None:
        self._messages.append(
            self._record_proto.SingleMessage(
                channel_name=channel_name,
                time=timestamp_ns,
                content=content,
            )
        )
        self._channel_counts[channel_name] = self._channel_counts.get(channel_name, 0) + 1

    def close_record(self) -> None:
        if self._fd is None:
            return

        if self._messages:
            self._messages.sort(key=lambda m: m.time)
            msgs = self._messages
            chunk_header = self._record_proto.ChunkHeader(
                begin_time=msgs[0].time,
                end_time=msgs[-1].time,
                message_number=len(msgs),
                raw_size=sum(len(m.content) for m in msgs),
            )
            chunk_body = self._record_proto.ChunkBody(messages=msgs)
            pos = os.lseek(self._fd, 0, os.SEEK_CUR)
            self._write_section(self._record_proto.SECTION_CHUNK_HEADER, chunk_header)
            header_idx = self._index.indexes.add()
            header_idx.type = self._record_proto.SECTION_CHUNK_HEADER
            header_idx.position = pos
            header_idx.chunk_header_cache.begin_time = chunk_header.begin_time
            header_idx.chunk_header_cache.end_time = chunk_header.end_time
            header_idx.chunk_header_cache.message_number = chunk_header.message_number
            header_idx.chunk_header_cache.raw_size = chunk_header.raw_size

            pos = os.lseek(self._fd, 0, os.SEEK_CUR)
            self._write_section(self._record_proto.SECTION_CHUNK_BODY, chunk_body)
            body_idx = self._index.indexes.add()
            body_idx.type = self._record_proto.SECTION_CHUNK_BODY
            body_idx.position = pos
            body_idx.chunk_body_cache.message_number = len(msgs)

            self._header.chunk_number += 1
            self._header.message_number += len(msgs)
            self._header.begin_time = chunk_header.begin_time
            self._header.end_time = chunk_header.end_time

        for idx in self._index.indexes:
            if idx.type == self._record_proto.SECTION_CHANNEL:
                name = idx.channel_cache.name
                idx.channel_cache.message_number = self._channel_counts.get(name, 0)

        self._header.index_position = os.lseek(self._fd, 0, os.SEEK_CUR)
        self._write_section(self._record_proto.SECTION_INDEX, self._index)
        self._header.is_complete = True

        os.lseek(self._fd, 0, os.SEEK_SET)
        self._write_section(self._record_proto.SECTION_HEADER, self._header)
        os.close(self._fd)
        self._fd = None

    @staticmethod
    def encode_proto_desc(message: Message) -> bytes:
        from autolink.proto import proto_desc_pb2 as proto_desc_proto

        def build(file_desc) -> proto_desc_proto.ProtoDesc:
            proto = FileDescriptorProto()
            file_desc.CopyToProto(proto)
            proto.name = file_desc.name
            desc = proto_desc_proto.ProtoDesc()
            desc.desc = proto.SerializeToString()
            for dep in file_desc.dependencies:
                desc.dependencies.append(build(dep))
            return desc

        return build(message.DESCRIPTOR.file).SerializeToString()

    def _write_section(self, section_type: int, message: Message) -> None:
        assert self._fd is not None
        payload = message.SerializeToString()
        os.write(self._fd, _SECTION.pack(section_type, len(payload)))
        os.write(self._fd, payload)
        if section_type == self._record_proto.SECTION_HEADER:
            padding = _HEADER_LENGTH - len(payload)
            if padding > 0:
                os.write(self._fd, b"\0" * padding)
        self._header.size = os.lseek(self._fd, 0, os.SEEK_CUR)
