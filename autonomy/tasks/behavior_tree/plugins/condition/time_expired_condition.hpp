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

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

/**
 * @brief A BT::ConditionNode that returns SUCCESS every time a specified
 * time period passes and FAILURE otherwise
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class TimeExpiredCondition : public BT::ConditionNode {
 public:
  /**
   * @brief A constructor for nav2_behavior_tree::TimeExpiredCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TimeExpiredCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  TimeExpiredCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() { return {BT::InputPort<double>("seconds", 1.0, "Seconds")}; }

 private:
  std::shared_ptr<::autolink::Node> node_;
  commsgs::builtin_interfaces::Time start_;
  double period_;
};

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy