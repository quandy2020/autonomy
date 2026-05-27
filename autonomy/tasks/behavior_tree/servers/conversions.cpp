/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/servers/conversions.hpp"

#include <chrono>
#include <cmath>

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace {

commsgs::builtin_interfaces::Duration ElapsedDuration(
    std::chrono::steady_clock::time_point start) {
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed)
                          .count();
    return commsgs::builtin_interfaces::Duration::FromNanoseconds(ns);
}

float Distance2D(const commsgs::geometry_msgs::PoseStamped& a,
                 const commsgs::geometry_msgs::PoseStamped& b) {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return static_cast<float>(std::hypot(dx, dy));
}

}  // namespace

commsgs::geometry_msgs::PoseStamped PoseFromActionGoal(
    const proto::NavigateToPoseAction_Goal& goal) {
    if (goal.has_pose()) {
        return commsgs::geometry_msgs::FromProto(goal.pose());
    }
    return {};
}

std::vector<commsgs::geometry_msgs::PoseStamped> PosesFromActionGoal(
    const proto::NavigateThroughPosesAction_Goal& goal) {
    std::vector<commsgs::geometry_msgs::PoseStamped> out;
    out.reserve(goal.poses_size());
    for (const auto& p : goal.poses()) {
        out.push_back(commsgs::geometry_msgs::FromProto(p));
    }
    return out;
}

proto::NavigateToPoseAction_Feedback MakeNavigateToPoseFeedback(
    const commsgs::geometry_msgs::PoseStamped& current_pose,
    const commsgs::geometry_msgs::PoseStamped& goal, int number_recoveries,
    std::chrono::steady_clock::time_point navigation_start) {
    proto::NavigateToPoseAction_Feedback fb;
    *fb.mutable_current_pose() = commsgs::geometry_msgs::ToProto(current_pose);
    *fb.mutable_navigation_time() =
        commsgs::builtin_interfaces::ToProto(ElapsedDuration(navigation_start));
    fb.set_number_of_recoveries(number_recoveries);
    fb.set_distance_remaining(Distance2D(current_pose, goal));
    return fb;
}

proto::NavigateThroughPosesAction_Feedback MakeNavigateThroughPosesFeedback(
    const commsgs::geometry_msgs::PoseStamped& current_pose,
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    int number_recoveries,
    std::chrono::steady_clock::time_point navigation_start) {
    proto::NavigateThroughPosesAction_Feedback fb;
    *fb.mutable_current_pose() = commsgs::geometry_msgs::ToProto(current_pose);
    *fb.mutable_navigation_time() =
        commsgs::builtin_interfaces::ToProto(ElapsedDuration(navigation_start));
    fb.set_number_of_recoveries(number_recoveries);
    if (!goals.empty()) {
        fb.set_distance_remaining(Distance2D(current_pose, goals.back()));
        fb.set_number_of_poses_remaining(static_cast<int32_t>(goals.size()));
    }
    return fb;
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
