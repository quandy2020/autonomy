/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoviz/core/bridge/channel_sync.hpp"

#include <algorithm>
#include <optional>

#include "autoviz/core/bridge/msgs_converter.hpp"
#include "autoviz/core/bridge/schema_builder.hpp"
#include "autoviz/core/common/base64.hpp"
#include "autoviz/core/common/regex_utils.hpp"

namespace autoviz {
namespace bridge {
namespace {

constexpr char kAutolinkChannelEncoding[] = "protobuf";

bool IsAdvertisedTopic(const std::map<foxglove_ws::ChannelId, foxglove_ws::Channel>& advertised_channels,
                       const std::string& topic) {
  for (const auto& p : advertised_channels) {
    if (p.second.topic == topic) {
      return true;
    }
  }
  return false;
}

}  // namespace

ChannelInfos CollectChannelInfosFromWriters(const Writers& writers,
                                            const std::vector<std::regex>& whitelist) {
  ChannelInfos infos;
  infos.reserve(writers.size());
  for (const auto& w : writers) {
    const std::string channel = w.channel_name();
    if (channel.empty() || !foxglove_ws::isWhitelisted(channel, whitelist)) {
      continue;
    }

    auto it = infos.find(channel);
    if (it == infos.end()) {
      infos.emplace(channel, WriterChannelInfo{w.message_type(), w.proto_desc()});
      continue;
    }
    if (it->second.msg_type.empty() && !w.message_type().empty()) {
      it->second.msg_type = w.message_type();
    }
    if (it->second.proto_desc.empty() && !w.proto_desc().empty()) {
      it->second.proto_desc = w.proto_desc();
    }
  }
  return infos;
}

std::unordered_set<std::string> BuildTopicSet(const ChannelInfos& channel_infos) {
  std::unordered_set<std::string> topic_set;
  topic_set.reserve(channel_infos.size());
  for (const auto& kv : channel_infos) {
    topic_set.insert(kv.first);
  }
  return topic_set;
}

std::vector<foxglove_ws::ChannelId> CollectChannelsToRemove(
    const std::map<foxglove_ws::ChannelId, foxglove_ws::Channel>& advertised_channels,
    const std::unordered_set<std::string>& current_topics) {
  std::vector<foxglove_ws::ChannelId> to_remove;
  for (auto it = advertised_channels.begin(); it != advertised_channels.end(); ++it) {
    if (current_topics.count(it->second.topic) == 0) {
      to_remove.push_back(it->first);
    }
  }
  return to_remove;
}

std::vector<foxglove_ws::ChannelWithoutId> BuildChannelsToAdd(
    const ChannelInfos& channel_infos,
    const std::map<foxglove_ws::ChannelId, foxglove_ws::Channel>& advertised_channels) {
  std::vector<foxglove_ws::ChannelWithoutId> to_add;
  for (const auto& kv : channel_infos) {
    const std::string& topic = kv.first;
    if (IsAdvertisedTopic(advertised_channels, topic)) {
      continue;
    }

    const std::string& msg_type = kv.second.msg_type;
    const std::string& proto_desc = kv.second.proto_desc;

    foxglove_ws::ChannelWithoutId ch;
    ch.topic = topic;
    ch.schemaName = msg_type.empty() ? "unknown" : MapSchemaNameToRos2Style(msg_type);
    ch.encoding = kAutolinkChannelEncoding;
    const std::string schema_bin = BuildFoxgloveSchemaFromProtoDesc(proto_desc);
    ch.schema = schema_bin.empty() ? "" : foxglove_ws::base64Encode(schema_bin);
    ch.schemaEncoding = std::optional<std::string>("protobuf");
    to_add.push_back(std::move(ch));
  }
  return to_add;
}

foxglove_ws::MapOfSets BuildPublishedGraph(const Writers& writers,
                                           const std::vector<std::regex>& whitelist) {
  foxglove_ws::MapOfSets published;
  for (const auto& w : writers) {
    const std::string chan = w.channel_name();
    if (foxglove_ws::isWhitelisted(chan, whitelist)) {
      published[chan].insert(w.node_name());
    }
  }
  return published;
}

double ComputeUpdatePeriodMs(size_t update_count, double min_period_ms, double max_period_ms) {
  return std::max(min_period_ms, static_cast<double>(std::min(size_t(1) << update_count, static_cast<size_t>(max_period_ms))));
}

}  // namespace bridge
}  // namespace autoviz

