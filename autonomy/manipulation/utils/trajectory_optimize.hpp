/*
 * Copyright 2026 The Openbot Authors
 *
 * Shared seed / CHOMP / STOMP optimize helpers.
 */

#pragma once

#include <random>
#include <string>

#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/planner/chomp/chomp_params.hpp"
#include "autonomy/manipulation/planner/stomp/stomp_params.hpp"

namespace autonomy {
namespace manipulation {
namespace utils {

automsgs::msgs::trajectory_msgs::JointTrajectory InterpolateSeedTrajectory(
    const MotionPlanRequest& request, int waypoints,
    const std::string& method = "linear");

automsgs::msgs::trajectory_msgs::JointTrajectory PerturbSeed(const automsgs::msgs::trajectory_msgs::JointTrajectory& seed,
                                  std::mt19937* rng, double scale = 0.15);

void ClampTrajectory(const MotionPlanRequest& request,
                     automsgs::msgs::trajectory_msgs::JointTrajectory* traj);

automsgs::msgs::trajectory_msgs::JointTrajectory ChompOptimize(const MotionPlanRequest& request,
                                    automsgs::msgs::trajectory_msgs::JointTrajectory traj,
                                    const ChompParams& params);

automsgs::msgs::trajectory_msgs::JointTrajectory ChompPolish(const MotionPlanRequest& request,
                                  automsgs::msgs::trajectory_msgs::JointTrajectory traj, int iters);

automsgs::msgs::trajectory_msgs::JointTrajectory StompOptimize(const MotionPlanRequest& request,
                                    automsgs::msgs::trajectory_msgs::JointTrajectory seed,
                                    const StompParams& params);

}  // namespace utils
}  // namespace manipulation
}  // namespace autonomy
