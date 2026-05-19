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

#include "autonomy/tasks/behavior_tree/plugins/condition/distance_traveled_condition.hpp"

#include "autonomy/common/log.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

DistanceTraveledCondition::DistanceTraveledCondition(
    const std::string& condition_name, const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf),
      distance_(1.0),
      transform_tolerance_(0.1) {}

void DistanceTraveledCondition::initialize() {
    getInput("distance", distance_);
    tf_ =
        config().blackboard->get<std::shared_ptr<autonomy::transform::Buffer>>(
            "tf_buffer");
    getInput("transform_tolerance", transform_tolerance_);

    GetInputOrBlackboard("global_frame", global_frame_);
    GetInputOrBlackboard("robot_base_frame", robot_base_frame_);
}

BT::NodeStatus DistanceTraveledCondition::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    if (!BT::isStatusActive(status())) {
        if (!autonomy::tasks::utils::getCurrentPose(
                start_pose_, tf_, global_frame_, robot_base_frame_,
                static_cast<float>(transform_tolerance_))) {
            ADEBUG << "Current robot pose is not available.";
        }
        return BT::NodeStatus::FAILURE;
    }

    // Determine distance travelled since we've started this iteration
    commsgs::geometry_msgs::PoseStamped current_pose;
    if (!autonomy::tasks::utils::getCurrentPose(
            current_pose, tf_, global_frame_, robot_base_frame_,
            static_cast<float>(transform_tolerance_))) {
        ADEBUG << "Current robot pose is not available.";
        return BT::NodeStatus::FAILURE;
    }

    // Get euclidean distance
    auto travelled = autonomy::map::costmap_2d::utils::euclidean_distance(
        start_pose_.pose, current_pose.pose);

    if (travelled < distance_) {
        return BT::NodeStatus::FAILURE;
    }

    // Update start pose
    start_pose_ = current_pose;

    return BT::NodeStatus::SUCCESS;
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::
                                 condition::DistanceTraveledCondition>(
        "DistanceTraveled");
}
