/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"

#include "autoview/common/interaction/node_abstraction_iface.hpp"

namespace autoview {
namespace common {
namespace interaction {

class NodeAbstraction : public NodeAbstractionIface
{
public:
  NodeAbstraction() = delete;

  /// Creates a ros node with the given name
  /**
   * Internally a rclcpp::Node is created.
   * If a rclcpp::Node with the given name already exists a node abstraction for this already
   * existing node is created.
   *
   * \param node_name name of the node to create
   */
  explicit NodeAbstraction(const std::string & node_name);

  /// Returns the name of the ros node
  /**
   * The returned node name is what was given as constructor argument.
   *
   * \return the name of the node
   */
  std::string get_node_name() const override;

  /// Return a map with topic names mapped to a list of types for that topic.
  /**
   * The node name is what was given when initializing this API.
   *
   * \return map of topic names and their types
   */
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types() const override;

  // TODO(wjwwood): think about a suitable way to extend the abstraction to also cover subscriptions
  std::shared_ptr<autolink::Node> get_raw_node() override
  {
    return raw_node_;
  }

private:
  std::shared_ptr<autolink::Node> raw_node_;
};  

}  // namespace interaction
}  // namespace common
}  // namespace autoview