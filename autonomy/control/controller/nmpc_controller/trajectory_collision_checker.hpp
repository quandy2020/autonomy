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

/**
 * @file trajectory_collision_checker.hpp
 * @brief Costmap validation of NMPC predicted rollouts
 */

#pragma once

#include <memory>
#include <vector>

#include "autonomy/control/controller/nmpc_controller/differential_drive_problem.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

/**
 * @class nmpc_controller::TrajectoryCollisionChecker
 * @brief Validates NMPC predicted trajectories against the local costmap
 */
class TrajectoryCollisionChecker {
 public:
    /**
     * @brief Initialize collision checker on bringup
     * @param costmap_wrapper Costmap2DWrapper object of environment
     * @param options Collision-check options
     */
    void Configure(std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                       costmap_wrapper,
                   const proto::NMPCControllerOptions& options);

    /**
     * @brief Check whether all poses along the rollout are collision-free
     * @param states Predicted NMPC state trajectory
     * @return True if the trajectory is free of lethal obstacles
     */
    bool IsTrajectoryFree(
        const std::vector<DifferentialDriveProblem::StateVector>& states) const;

 private:
    /**
     * @brief Cost at a single SE(2) pose (point or footprint)
     * @param x World x [m]
     * @param y World y [m]
     * @param yaw Heading [rad]
     */
    double PoseCost(double x, double y, double yaw) const;

    // Costmap used to evaluate trajectory poses
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    // Collision-check thresholds and footprint settings
    proto::NMPCControllerOptions options_;
};

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
