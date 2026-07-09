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

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "autonomy/visualization/common/channel_snapshot.hpp"
#include "autonomy/visualization/schema/schema_compatibility.hpp"

namespace autonomy {
namespace visualization {

struct BridgeFilterOptions {
  std::vector<std::string> channel_allowlist;
  std::vector<std::string> message_type_allowlist;
  bool require_proto_desc = true;
};

class BridgeFilter {
 public:
  explicit BridgeFilter(BridgeFilterOptions options)
      : options_(std::move(options)) {}

  bool ShouldBridge(const ChannelSnapshot& channel) const {
    if (channel.channel_name.empty() || channel.source_message_type.empty()) {
      return false;
    }
    if (options_.require_proto_desc && channel.proto_desc.empty()) {
      return false;
    }
    if (!MatchesAllowlist(channel.channel_name, options_.channel_allowlist)) {
      return false;
    }
    if (!MatchesAllowlist(channel.source_message_type,
                          options_.message_type_allowlist)) {
      return false;
    }
    if (!SchemaCompatibility::IsBridgeable(channel.source_message_type)) {
      return false;
    }
    return true;
  }

 private:
  static bool MatchesAllowlist(const std::string& value,
                               const std::vector<std::string>& allowlist) {
    if (allowlist.empty()) {
      return true;
    }
    for (const auto& entry : allowlist) {
      if (entry.empty()) {
        continue;
      }
      if (value == entry || value.rfind(entry, 0) == 0) {
        return true;
      }
    }
    return false;
  }

  BridgeFilterOptions options_;
};

}  // namespace visualization
}  // namespace autonomy
