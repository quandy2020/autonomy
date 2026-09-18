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

"""ROS1 bag to autolink .record conversion."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

from autonomy.tools.bag_convert.bag_convert_config import BagConvertConfig
from autonomy.tools.bag_convert.proto_registry import ProtoRegistry
from autonomy.tools.bag_convert.record_writer import RecordWriter
from autonomy.tools.bag_convert.ros_converter import RosConverter


@dataclass(frozen=True)
class BagTopicInfo:
    topic: str
    msgtype: str
    msgcount: int


@dataclass(frozen=True)
class BagInfo:
    path: Path
    start_time_ns: int
    end_time_ns: int
    duration_sec: float
    message_count: int
    topics: Tuple[BagTopicInfo, ...]


class BagConverter:
    """Read a ROS1 bag and write an autolink .record file."""

    def __init__(
        self,
        config: BagConvertConfig | None = None,
        registry: ProtoRegistry | None = None,
        ros_converter: RosConverter | None = None,
    ) -> None:
        self._config = config or BagConvertConfig.create_default()
        self._registry = registry or ProtoRegistry(self._config)
        self._ros = ros_converter or RosConverter(self._registry)

    def inspect_bag(self, input_bag: Path) -> BagInfo:
        from rosbags.rosbag1 import Reader

        with Reader(str(input_bag)) as reader:
            topics = tuple(
                BagTopicInfo(
                    topic=topic,
                    msgtype=info.msgtype or "",
                    msgcount=int(info.msgcount),
                )
                for topic, info in sorted(reader.topics.items())
            )
            start = int(reader.start_time or 0)
            end = int(reader.end_time or 0)
            duration = (end - start) / 1e9 if end > start else 0.0
            return BagInfo(
                path=input_bag,
                start_time_ns=start,
                end_time_ns=end,
                duration_sec=duration,
                message_count=int(reader.message_count or 0),
                topics=topics,
            )

    def convert_bag(
        self,
        input_bag: Path,
        output_record: Path,
        use_leading_slash: bool,
        topic_filter: Iterable[str] | None,
        topic_remap: Dict[str, str] | None,
        skip_unsupported: bool = False,
        max_messages: int | None = None,
        start_offset_sec: float = 0.0,
        duration_sec: float | None = None,
    ) -> Tuple[int, Dict[str, int]]:
        self._registry.setup_proto_import_path()

        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore

        def normalize_topic(topic: str) -> str:
            topic = topic.strip()
            if topic.startswith("/"):
                topic = topic[1:]
            return f"/{topic}" if use_leading_slash else topic

        store = get_typestore(getattr(Stores, self._config.ros_bag_store))
        allowed = (
            {normalize_topic(t) for t in topic_filter} if topic_filter else None
        )

        writer = RecordWriter()
        writer.open_record(str(output_record))

        registered_topics: set[str] = set()
        counts: Dict[str, int] = {}
        skipped: Dict[str, int] = {}
        total = 0

        try:
            with Reader(str(input_bag)) as reader:
                connections = list(reader.connections)
                if allowed is not None:
                    connections = [
                        conn
                        for conn in connections
                        if normalize_topic(conn.topic) in allowed
                    ]
                    missing = allowed - {normalize_topic(c.topic) for c in connections}
                    if missing:
                        print(
                            "warning: topics not in bag: "
                            + ", ".join(sorted(missing))
                        )

                start_ns: Optional[int] = None
                stop_ns: Optional[int] = None
                if reader.start_time:
                    if start_offset_sec > 0:
                        start_ns = int(reader.start_time + start_offset_sec * 1e9)
                    if duration_sec is not None:
                        base = start_ns if start_ns is not None else int(reader.start_time)
                        stop_ns = int(base + duration_sec * 1e9)

                for conn, timestamp, raw in reader.messages(
                    connections=connections,
                    start=start_ns,
                    stop=stop_ns,
                ):
                    ros_msg = store.deserialize_ros1(raw, conn.msgtype)
                    try:
                        proto_msg, type_name = self._ros.convert_ros_message(
                            conn.msgtype, ros_msg
                        )
                    except KeyError:
                        if skip_unsupported:
                            skipped[conn.msgtype] = skipped.get(conn.msgtype, 0) + 1
                            continue
                        raise RuntimeError(
                            f"Unsupported message type {conn.msgtype} on {conn.topic}. "
                            "Add the message to automsgs proto or use --skip-unsupported"
                        ) from None

                    source_topic = normalize_topic(conn.topic)
                    if allowed is not None and source_topic not in allowed:
                        continue
                    topic = (
                        topic_remap.get(source_topic, source_topic)
                        if topic_remap
                        else source_topic
                    )

                    if topic not in registered_topics:
                        writer.write_channel(
                            topic,
                            type_name,
                            RecordWriter.encode_proto_desc(proto_msg),
                        )
                        registered_topics.add(topic)
                        counts[topic] = 0

                    writer.write_message(
                        topic, proto_msg.SerializeToString(), int(timestamp)
                    )
                    counts[topic] += 1
                    total += 1
                    if total % 200 == 0:
                        print(f"  converted {total} messages...", flush=True)
                    if max_messages is not None and total >= max_messages:
                        break
        finally:
            writer.close_record()

        if skipped:
            print("Skipped unsupported message types:")
            for msgtype, count in sorted(skipped.items()):
                print(f"  {msgtype}: {count}")
        return total, counts
