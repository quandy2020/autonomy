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

#include <string>
#include <vector>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A BT::ActionNodeBase to shorten path by some distance
 */
class ConcatenatePaths : public BT::ActionNodeBase {
 public:
  /**
   * @brief A nav2_behavior_tree::ConcatenatePaths constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ConcatenatePaths(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific
   * ports
   */
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<commsgs::planning_msgs::Path>("input_path1", "Input Path 1 to cancatenate"),
        BT::InputPort<commsgs::planning_msgs::Path>("input_path2", "Input Path 2 to cancatenate"),
        BT::OutputPort<commsgs::planning_msgs::Path>("output_path", "Paths concatenated"),
    };
  }

 private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
