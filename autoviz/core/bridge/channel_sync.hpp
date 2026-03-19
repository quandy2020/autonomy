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

#pragma once

#include <map>
#include <regex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "autolink/proto/role_attributes.pb.h"
#include "autoviz/core/common/common.hpp"
#include "autoviz/core/common/server_interface.hpp"

namespace autoviz {
namespace bridge {

struct WriterChannelInfo {
  std::string msg_type;
  std::string proto_desc;
};

using Writers = std::vector<autolink::proto::RoleAttributes>;
using ChannelInfos = std::unordered_map<std::string, WriterChannelInfo>;

ChannelInfos CollectChannelInfosFromWriters(const Writers& writers,
                                            const std::vector<std::regex>& whitelist);

std::unordered_set<std::string> BuildTopicSet(const ChannelInfos& channel_infos);

std::vector<foxglove_ws::ChannelId> CollectChannelsToRemove(
    const std::map<foxglove_ws::ChannelId, foxglove_ws::Channel>& advertised_channels,
    const std::unordered_set<std::string>& current_topics);

std::vector<foxglove_ws::ChannelWithoutId> BuildChannelsToAdd(
    const ChannelInfos& channel_infos,
    const std::map<foxglove_ws::ChannelId, foxglove_ws::Channel>& advertised_channels);

foxglove_ws::MapOfSets BuildPublishedGraph(const Writers& writers,
                                           const std::vector<std::regex>& whitelist);

double ComputeUpdatePeriodMs(size_t update_count, double min_period_ms, double max_period_ms);

}  // namespace bridge
}  // namespace autoviz

