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

#include <atomic>
#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/transform/buffer.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

/**
 * @brief A BT::ConditionNode that returns SUCCESS if there is a valid transform
 * between two specified frames and FAILURE otherwise
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class TransformAvailableCondition : public BT::ConditionNode {
 public:
  /**
   * @brief A constructor for nav2_behavior_tree::TransformAvailableCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TransformAvailableCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  TransformAvailableCondition() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::TransformAvailableCondition
   */
  ~TransformAvailableCondition();

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
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("child", std::string(), "Child frame for transform"),
            BT::InputPort<std::string>("parent", std::string(), "parent frame for transform")};
  }

 private:
  std::shared_ptr<::autolink::Node> node_;
  std::shared_ptr<autonomy::transform::Buffer> tf_;

  std::atomic<bool> was_found_;

  std::string child_frame_;
  std::string parent_frame_;
};

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy