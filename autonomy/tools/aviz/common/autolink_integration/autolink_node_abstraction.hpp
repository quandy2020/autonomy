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

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/tools/aviz/common/autolink_integration/autolink_node_abstraction_iface.hpp"

namespace aviz {
namespace common {
namespace autolink_integration {

/**
 * @brief Concrete implementation of AutolinkNodeAbstractionIface.
 *
 * This class owns or references a single autolink::Node instance and uses
 * Autolink's service discovery (TopologyManager) to provide information about
 * available channels and their message types
 * topic names and types from rclcpp.
 */
class AutolinkNodeAbstraction : public AutolinkNodeAbstractionIface
{
public:
    AutolinkNodeAbstraction() = delete;

    /// @brief Create a node abstraction for the given node name.
    explicit AutolinkNodeAbstraction(const std::string& node_name);

    std::string get_node_name() const override;

    std::map<std::string, std::vector<std::string>> get_channel_names_and_types() const override;

    std::shared_ptr<::autolink::Node> get_raw_node() override;

private:
    std::string node_name_;
    std::shared_ptr<::autolink::Node> raw_node_;
};

}  // namespace autolink_integration
}  // namespace common
}  // namespace aviz
