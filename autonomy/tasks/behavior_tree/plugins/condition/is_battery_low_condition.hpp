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

#include <memory>
#include <mutex>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

/**
 * @brief A BT::ConditionNode that listens to a battery topic and
 * returns SUCCESS when battery is low and FAILURE otherwise
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class IsBatteryLowCondition : public BT::ConditionNode {
 public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsBatteryLowCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  IsBatteryLowCondition() = delete;

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
   * @brief Function to create ROS interfaces
   */
  void createROSInterfaces();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("min_battery", "Minimum battery percentage/voltage"),
        BT::InputPort<std::string>("battery_topic", std::string("/battery_status"), "Battery topic"),
        BT::InputPort<bool>("is_voltage", false, "If true voltage will be used to check for low battery"),
    };
  }

 private:
  /**
   * @brief Callback function for battery topic
   * @param msg Shared pointer to commsgs::sensor_msgs::BatteryState message
   */
  void batteryCallback(const std::shared_ptr<commsgs::sensor_msgs::BatteryState>& msg);

  std::shared_ptr<::autolink::Node> node_;
  std::shared_ptr<::autolink::Reader<commsgs::sensor_msgs::BatteryState>> battery_sub_;
  std::string battery_topic_;
  double min_battery_;
  bool is_voltage_;
  bool is_battery_low_;
};

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy