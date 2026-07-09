/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/visualization/transport/autolink_discovery.hpp"

#include <utility>

#include "autonomy/visualization/schema/schema_compatibility.hpp"

namespace autonomy {
namespace visualization {

AutolinkDiscovery::AutolinkDiscovery(
    autolink::service_discovery::ChannelManagerPtr channel_manager)
    : channel_manager_(std::move(channel_manager)) {}

std::vector<ChannelSnapshot> AutolinkDiscovery::ScanChannels() const {
  std::vector<ChannelSnapshot> channels;
  if (channel_manager_ == nullptr) {
    return channels;
  }

  std::vector<std::string> channel_names;
  channel_manager_->GetChannelNames(&channel_names);
  for (const auto& channel_name : channel_names) {
    if (!channel_manager_->HasWriter(channel_name)) {
      continue;
    }

    std::string message_type;
    std::string proto_desc;
    channel_manager_->GetMsgType(channel_name, &message_type);
    channel_manager_->GetProtoDesc(channel_name, &proto_desc);
    if (message_type.empty() || proto_desc.empty()) {
      continue;
    }

    channels.push_back(ChannelSnapshot{
        channel_name,
        message_type,
        SchemaCompatibility::NormalizeSchemaName(message_type),
        proto_desc,
        "",
    });
  }

  return channels;
}

}  // namespace visualization
}  // namespace autonomy
