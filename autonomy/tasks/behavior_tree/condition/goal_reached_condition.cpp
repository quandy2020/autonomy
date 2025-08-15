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

#include "autonomy/tasks/behavior_tree/condition/goal_reached_condition.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace decorator {

GoalReachedCondition::GoalReachedCondition(
  const std::string& condition_name,
  const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf),
    initialized_(false)
{
    // auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    //     node, "robot_base_frame", this);
}

GoalReachedCondition::~GoalReachedCondition()
{
    cleanup();
}

void GoalReachedCondition::initialize()
{
    // node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    // nav2_util::declare_parameter_if_not_declared(
    //     node_, "goal_reached_tol",
    //     rclcpp::ParameterValue(0.25));
    // node_->get_parameter_or<double>("goal_reached_tol", goal_reached_tol_, 0.25);
    // tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    // node_->get_parameter("transform_tolerance", transform_tolerance_);

    // initialized_ = true;
}

BT::NodeStatus GoalReachedCondition::tick()
{
    if (!initialized_) {
        initialize();
    }

    if (isGoalReached()) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

bool GoalReachedCondition::isGoalReached()
{
    commsgs::geometry_msgs::PoseStamped goal;
    getInput("goal", goal);

    commsgs::geometry_msgs::PoseStamped current_pose;
    // if (!nav2_util::getCurrentPose(
    //     current_pose, *tf_, goal.header.frame_id, robot_base_frame_, transform_tolerance_))
    // {
    //     RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    //     return false;
    // }

    double dx = goal.pose.position.x - current_pose.pose.position.x;
    double dy = goal.pose.position.y - current_pose.pose.position.y;

    return (dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_);
}


}  // namespace decorator
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy