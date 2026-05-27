/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/proto/action/navigate_through_poses.pb.h"
#include "autonomy/tasks/proto/action/navigate_to_pose.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

commsgs::geometry_msgs::PoseStamped PoseFromActionGoal(
    const proto::NavigateToPoseAction_Goal& goal);

std::vector<commsgs::geometry_msgs::PoseStamped> PosesFromActionGoal(
    const proto::NavigateThroughPosesAction_Goal& goal);

proto::NavigateToPoseAction_Feedback MakeNavigateToPoseFeedback(
    const commsgs::geometry_msgs::PoseStamped& current_pose,
    const commsgs::geometry_msgs::PoseStamped& goal,
    int number_recoveries,
    std::chrono::steady_clock::time_point navigation_start);

proto::NavigateThroughPosesAction_Feedback MakeNavigateThroughPosesFeedback(
    const commsgs::geometry_msgs::PoseStamped& current_pose,
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    int number_recoveries,
    std::chrono::steady_clock::time_point navigation_start);

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
