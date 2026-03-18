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

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_service_node.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"
#include "autonomy/tasks/navigator/proto/srv.pb.h"
#include "behaviortree_cpp/json_export.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A autonomy::tasks::behavior_tree::BtServiceNode class that removes
 * goals that are in collision in on the global costmap wraps proto::GetCosts
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class RemoveInCollisionGoals : public BtServiceNode<proto::GetCosts> {
 public:
  /**
   * @brief A constructor for nav2_behavior_tree::RemoveInCollisionGoals
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  RemoveInCollisionGoals(const std::string& service_node_name, const BT::NodeConfiguration& conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  BT::NodeStatus on_completion(std::shared_ptr<proto::GetCosts::Response> response) override;

  static BT::PortsList providedPorts() {
    // Register JSON definitions for the types used in the ports
    return providedBasicPorts(
        {BT::InputPort<commsgs::planning_msgs::Goals>("input_goals", "Original goals to remove from"),
         BT::InputPort<double>("cost_threshold", 254.0, "Cost threshold for considering a goal in collision"),
         BT::InputPort<bool>("use_footprint", true, "Whether to use footprint cost"),
         BT::InputPort<bool>("consider_unknown_as_obstacle", false, "Whether to consider unknown cost as obstacle"),
         BT::OutputPort<commsgs::planning_msgs::Goals>("output_goals", "Goals with in-collision goals removed"),
         BT::InputPort<std::vector<proto::WaypointStatus>>("input_waypoint_statuses",
                                                           "Original waypoint_statuses to mark waypoint status from"),
         BT::OutputPort<std::vector<proto::WaypointStatus>>("output_waypoint_statuses",
                                                            "Waypoint_statuses with in-collision waypoints marked")});
  }

 private:
  bool use_footprint_;
  bool consider_unknown_as_obstacle_;
  double cost_threshold_;
  commsgs::planning_msgs::Goals input_goals_;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
