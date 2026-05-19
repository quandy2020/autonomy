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

#include "autonomy/tasks/behavior_tree/plugins/condition/goal_reached_condition.hpp"

#include "autonomy/common/log.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

GoalReachedCondition::GoalReachedCondition(const std::string& condition_name,
                                           const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf),
      goal_reached_tol_(0.25),
      transform_tolerance_(0.1) {
    auto node =
        config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");

    robot_base_frame_ = DeconflictPortAndParamFrame<std::string>(
        node, "robot_base_frame", this);
}

GoalReachedCondition::~GoalReachedCondition() {
    cleanup();
}

void GoalReachedCondition::initialize() {
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");

    getInput("goal_reached_tol", goal_reached_tol_);
    tf_ =
        config().blackboard->get<std::shared_ptr<autonomy::transform::Buffer>>(
            "tf_buffer");

    getInput("transform_tolerance", transform_tolerance_);
}

BT::NodeStatus GoalReachedCondition::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    if (isGoalReached()) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

bool GoalReachedCondition::isGoalReached() {
    commsgs::geometry_msgs::PoseStamped goal;
    getInput("goal", goal);

    commsgs::geometry_msgs::PoseStamped current_pose;
    if (!autonomy::tasks::utils::getCurrentPose(
            current_pose, tf_, goal.header.frame_id, robot_base_frame_,
            static_cast<float>(transform_tolerance_))) {
        ADEBUG << "Current robot pose is not available.";
        return false;
    }

    double dx = goal.pose.position.x - current_pose.pose.position.x;
    double dy = goal.pose.position.y - current_pose.pose.position.y;

    return (dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_);
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::
                                 condition::GoalReachedCondition>(
        "GoalReached");
}
