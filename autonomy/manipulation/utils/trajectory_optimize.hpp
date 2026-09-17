/*
 * Copyright 2026 The Openbot Authors
 *
 * Shared seed / CHOMP / STOMP optimize helpers.
 */

#pragma once

#include <random>
#include <string>

#include <automsgs/msgs/trajectory_msgs/joint_trajectory.pb.h>

#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/planner/chomp/chomp_params.hpp"
#include "autonomy/manipulation/planner/stomp/stomp_params.hpp"

namespace autonomy {
namespace manipulation {
namespace utils {

automsgs::msgs::trajectory_msgs::JointTrajectory InterpolateSeedTrajectory(
    const planner::MotionPlanRequest& request, int waypoints,
    const std::string& method = "linear");

automsgs::msgs::trajectory_msgs::JointTrajectory PerturbSeed(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& seed,
    std::mt19937* rng, double scale = 0.15);

void ClampTrajectory(const planner::MotionPlanRequest& request,
                     automsgs::msgs::trajectory_msgs::JointTrajectory* traj);

automsgs::msgs::trajectory_msgs::JointTrajectory ChompOptimize(
    const planner::MotionPlanRequest& request,
    automsgs::msgs::trajectory_msgs::JointTrajectory traj,
    const planner::ChompParams& params);

automsgs::msgs::trajectory_msgs::JointTrajectory ChompPolish(
    const planner::MotionPlanRequest& request,
    automsgs::msgs::trajectory_msgs::JointTrajectory traj, int iters);

automsgs::msgs::trajectory_msgs::JointTrajectory StompOptimize(
    const planner::MotionPlanRequest& request,
    automsgs::msgs::trajectory_msgs::JointTrajectory seed,
    const planner::StompParams& params);

}  // namespace utils

/** @brief Historical alias used by CHOMP/STOMP planners. */
namespace trajopt = utils;

}  // namespace manipulation
}  // namespace autonomy
