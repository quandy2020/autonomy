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

namespace autolink {
class Node;
}  // namespace autolink

namespace aviz {
namespace common {
namespace autolink_integration {

/**
 * @brief Abstract interface for an Autolink node used by aviz.
 *
 * This is similar to rviz_common::ros_integration::RosNodeAbstractionIface
 * but wraps an autolink::Node and exposes Autolink specific information.
 */
class AutolinkNodeAbstractionIface
{
public:
    using WeakPtr = std::weak_ptr<AutolinkNodeAbstractionIface>;

    virtual ~AutolinkNodeAbstractionIface() = default;

    /// @brief Get the logical node name.
    virtual std::string get_node_name() const = 0;

    /// @brief Get a map of channel names to a list of message type names.
    ///
    /// The list of types usually has size 1 for Autolink channels.
    virtual std::map<std::string, std::vector<std::string>> get_channel_names_and_types() const = 0;

    /// @brief Get access to the underlying autolink::Node.
    virtual std::shared_ptr<::autolink::Node> get_raw_node() = 0;
};

}  // namespace autolink_integration
}  // namespace common
}  // namespace aviz
