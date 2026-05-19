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

#include "autonomy/tasks/behavior_tree/plugins/action/remove_passed_goals_action.hpp"

#include <cmath>

#include "autonomy/common/log.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/navigator/proto/msg.pb.h"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

namespace {
// Helper function to find matching goal in waypoint statuses
int findNextMatchingGoalInWaypointStatuses(
    const std::vector<proto::WaypointStatus>& waypoint_statuses,
    const commsgs::geometry_msgs::PoseStamped& goal) {
    constexpr double tolerance = 0.01;  // 1cm tolerance for position matching

    for (size_t i = 0; i < waypoint_statuses.size(); ++i) {
        const auto& waypoint = waypoint_statuses[i];
        if (waypoint.has_waypoint_pose()) {
            // Convert proto pose to commsgs pose for comparison
            commsgs::geometry_msgs::PoseStamped waypoint_pose =
                commsgs::geometry_msgs::FromProto(waypoint.waypoint_pose());

            // Compare positions (ignore frame_id and timestamp for matching)
            double dx = waypoint_pose.pose.position.x - goal.pose.position.x;
            double dy = waypoint_pose.pose.position.y - goal.pose.position.y;
            double dist = std::hypot(dx, dy);

            if (dist < tolerance) {
                return static_cast<int>(i);
            }
        }
    }
    return -1;
}
}  // namespace

RemovePassedGoals::RemovePassedGoals(const std::string& name,
                                     const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(name, conf), viapoint_achieved_radius_(0.5) {}

void RemovePassedGoals::initialize() {
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    tf_ =
        config().blackboard->get<std::shared_ptr<autonomy::transform::Buffer>>(
            "tf_buffer");
    getInput("transform_tolerance", transform_tolerance_);
    getInput("radius", viapoint_achieved_radius_);
    robot_base_frame_ = DeconflictPortAndParamFrame<std::string>(
        node_, "robot_base_frame", this);
}

BT::NodeStatus RemovePassedGoals::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    commsgs::planning_msgs::Goals goal_poses;
    getInput("input_goals", goal_poses);

    if (goal_poses.goals.empty()) {
        setOutput("output_goals", goal_poses);
        return BT::NodeStatus::SUCCESS;
    }

    commsgs::geometry_msgs::PoseStamped current_pose;
    if (!autonomy::tasks::utils::getCurrentPose(
            current_pose, tf_, goal_poses.goals[0].header.frame_id,
            robot_base_frame_, transform_tolerance_)) {
        return BT::NodeStatus::FAILURE;
    }

    // get the `waypoint_statuses` vector
    std::vector<proto::WaypointStatus> waypoint_statuses;
    auto waypoint_statuses_get_res =
        getInput("input_waypoint_statuses", waypoint_statuses);
    if (!waypoint_statuses_get_res) {
        AERROR << "Missing [input_waypoint_statuses] port input!";
    }

    double dist_to_goal;
    while (goal_poses.goals.size() > 1) {
        dist_to_goal = autonomy::map::costmap_2d::utils::euclidean_distance(
            goal_poses.goals[0].pose, current_pose.pose);

        if (dist_to_goal > viapoint_achieved_radius_) {
            break;
        }

        // mark waypoint statuses before the goal is erased from goals
        if (waypoint_statuses_get_res) {
            auto cur_waypoint_index = findNextMatchingGoalInWaypointStatuses(
                waypoint_statuses, goal_poses.goals[0]);
            if (cur_waypoint_index == -1) {
                AERROR << "Failed to find matching goal in waypoint_statuses";
                return BT::NodeStatus::FAILURE;
            }
            waypoint_statuses[cur_waypoint_index].set_waypoint_status(
                proto::WaypointStatusType::WAYPOINT_STATUS_COMPLETED);
        }

        goal_poses.goals.erase(goal_poses.goals.begin());
    }

    setOutput("output_goals", goal_poses);
    // set `waypoint_statuses` output
    setOutput("output_waypoint_statuses", waypoint_statuses);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::plugins::action::RemovePassedGoals>(
        "RemovePassedGoals");
}
