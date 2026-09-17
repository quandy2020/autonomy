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
namespace planning {
namespace optimize {

core::RobotTrajectory InterpolateSeedTrajectory(
    const MotionPlanRequest& request, int waypoints,
    const std::string& method = "linear");

core::RobotTrajectory PerturbSeed(const core::RobotTrajectory& seed,
                                  std::mt19937* rng, double scale = 0.15);

void ClampTrajectory(const MotionPlanRequest& request,
                     core::RobotTrajectory* traj);

core::RobotTrajectory ChompOptimize(const MotionPlanRequest& request,
                                    core::RobotTrajectory traj,
                                    const ChompParams& params);

core::RobotTrajectory ChompPolish(const MotionPlanRequest& request,
                                  core::RobotTrajectory traj, int iters);

core::RobotTrajectory StompOptimize(const MotionPlanRequest& request,
                                    core::RobotTrajectory seed,
                                    const StompParams& params);

}  // namespace optimize
}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
