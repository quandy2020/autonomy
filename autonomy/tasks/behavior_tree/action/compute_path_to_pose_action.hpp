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

#include "autonomy/tasks/behavior_tree/action/bt_action.hpp"

// #include "autonomy/tasks/navigation/proto/compute_path_to_pose.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace action {

// /**
//  * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::ComputePathToPose
//  */
// class ComputePathToPoseAction : public BtActionNode<navigation::proto::ComputePathToPose>
// {
//     // using Action = nav2_msgs::action::ComputePathToPose;
//     // using ActionResult = Action::Result;

// public:
//     /**
//      * @brief A constructor for nav2_behavior_tree::ComputePathToPoseAction
//      * @param xml_tag_name Name for the XML tag for this node
//      * @param action_name Action name this node creates a client for
//      * @param conf BT node configuration
//      */
//     ComputePathToPoseAction(
//         const std::string& xml_tag_name, 
//         const std::string& action_name, 
//         const BT::NodeConfiguration& conf);

//     /**
//      * @brief Function to perform some user-defined operation on tick
//      */
//     void on_tick() override;

//     /**
//      * @brief Function to perform some user-defined operation upon successful completion of the action
//      */
//     BT::NodeStatus on_success() override;

//     /**
//      * @brief Function to perform some user-defined operation upon abortion of the action
//      */
//     BT::NodeStatus on_aborted() override;

//     /**
//      * @brief Function to perform some user-defined operation upon cancellation of the action
//      */
//     BT::NodeStatus on_cancelled() override;

//     /**
//      * \brief Override required by the a BT action. Cancel the action and set the path output
//      */
//     void halt() override;

//     /**
//      * @brief Creates list of BT ports
//      * @return BT::PortsList Containing basic ports along with node-specific ports
//      */
//     static BT::PortsList providedPorts()
//     {
//         return providedBasicPorts(
//         {
//             BT::InputPort<commsgs::geometry_msgs::PoseStamped>("goal", "Destination to plan to"),
//             BT::InputPort<commsgs::geometry_msgs::PoseStamped>("start", "Start pose of the path if overriding current robot pose"),
//             BT::InputPort<std::string>("planner_id", "", "Mapped name to the planner plugin type to use"),
//             BT::OutputPort<commsgs::planning_msgs::Path>("path", "Path created by ComputePathToPose node"),
//             // BT::OutputPort<ActionResult::_error_code_type>("error_code_id", "The compute path to pose error code"),
//         });
//     }
// };

}  // namespace action
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy