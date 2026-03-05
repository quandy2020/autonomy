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
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "autonomy/tasks/navigator/proto/msg.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A BT::ActionNodeBase that removes goals that the robot passed near to
 * @note This is an Asynchronous node. It will re-initialize when halted.
 */
class RemovePassedGoals : public BT::ActionNodeBase {
 public:
  RemovePassedGoals(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  static BT::PortsList providedPorts() {
    return {BT::InputPort<commsgs::planning_msgs::Goals>("input_goals", "Original goals to remove viapoints from"),
            BT::OutputPort<commsgs::planning_msgs::Goals>("output_goals", "Goals with passed viapoints removed"),
            BT::InputPort<double>("radius", 0.5, "radius to goal for it to be considered for removal"),
            BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
            BT::InputPort<std::vector<proto::WaypointStatus>>(
                "input_waypoint_statuses", "Original waypoint_statuses to mark waypoint status from"),
            BT::OutputPort<std::vector<proto::WaypointStatus>>("output_waypoint_statuses",
                                                               "Waypoint_statuses with passed waypoints marked")};
  }

 private:
  void halt() override {}
  BT::NodeStatus tick() override;

  double viapoint_achieved_radius_;
  double transform_tolerance_;
  std::shared_ptr<::autolink::Node> node_;
  std::shared_ptr<autonomy::transform::Buffer> tf_;
  std::string robot_base_frame_;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
