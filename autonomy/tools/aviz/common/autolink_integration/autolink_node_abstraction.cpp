/******************************************************************************
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
 *****************************************************************************/

#include "autonomy/tools/aviz/common/autolink_integration/autolink_node_abstraction.hpp"

#include "autolink/autolink.hpp"
#include "autolink/service_discovery/topology_manager.hpp"

namespace aviz {
namespace common {
namespace autolink_integration {

AutolinkNodeAbstraction::AutolinkNodeAbstraction(const std::string& node_name)
    : node_name_(node_name) {
    // autolink::CreateNode returns a shared_ptr<Node> and will internally
    // deal with reusing participants and discovery state as needed.
    raw_node_ = ::autolink::CreateNode(node_name_);
}

std::string AutolinkNodeAbstraction::get_node_name() const {
    return node_name_;
}

std::map<std::string, std::vector<std::string>>
AutolinkNodeAbstraction::get_channel_names_and_types() const {
    std::map<std::string, std::vector<std::string>> result;

    auto topology = ::autolink::service_discovery::TopologyManager::Instance();
    if (!topology) {
        return result;
    }
    auto& channel_mgr = topology->channel_manager();
    if (!channel_mgr) {
        return result;
    }

    std::vector<std::string> channels;
    channel_mgr->GetChannelNames(&channels);

    for (const auto& ch : channels) {
        std::string msg_type;
        channel_mgr->GetMsgType(ch, &msg_type);

        if (!msg_type.empty()) {
            result[ch].push_back(msg_type);
        } else {
            // Keep the channel with an empty type list so UI can still show it.
            result[ch];
        }
    }

    return result;
}

std::shared_ptr<::autolink::Node> AutolinkNodeAbstraction::get_raw_node() {
    return raw_node_;
}

}  // namespace autolink_integration
}  // namespace common
}  // namespace aviz
