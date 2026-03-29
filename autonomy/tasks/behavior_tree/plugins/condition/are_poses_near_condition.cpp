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

#include "autonomy/tasks/behavior_tree/plugins/condition/are_poses_near_condition.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

ArePosesNearCondition::ArePosesNearCondition(const std::string& condition_name,
                                             const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf) {
    auto node =
        config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    global_frame_ =
        DeconflictPortAndParamFrame<std::string>(node, "global_frame", this);
}

void ArePosesNearCondition::initialize() {
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    tf_ =
        config().blackboard->get<std::shared_ptr<autonomy::transform::Buffer>>(
            "tf_buffer");
    getInput("transform_tolerance", transform_tolerance_);
}

BT::NodeStatus ArePosesNearCondition::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    if (arePosesNearby()) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

bool ArePosesNearCondition::arePosesNearby() {
    commsgs::geometry_msgs::PoseStamped pose1, pose2;
    double tol;
    getInput("ref_pose", pose1);
    getInput("target_pose", pose2);
    getInput("tolerance", tol);

    if (pose1.header.frame_id != pose2.header.frame_id) {
        if (!autonomy::tasks::utils::transformPoseInTargetFrame(
                pose1, pose1, tf_, global_frame_,
                static_cast<float>(transform_tolerance_)) ||
            !autonomy::tasks::utils::transformPoseInTargetFrame(
                pose2, pose2, tf_, global_frame_,
                static_cast<float>(transform_tolerance_))) {
            AERROR << "Failed to transform poses to the same frame";
            return false;
        }
    }

    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    return (dx * dx + dy * dy) <= (tol * tol);
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::
                                 condition::ArePosesNearCondition>(
        "ArePosesNear");
}
