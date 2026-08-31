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
 * @file trajectory_collision_checker.cpp
 * @brief Implementation of nmpc_controller::TrajectoryCollisionChecker
 */

#include "autonomy/control/controller/nmpc_controller/trajectory_collision_checker.hpp"

#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

void TrajectoryCollisionChecker::Configure(
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper,
    const proto::NMPCControllerOptions& options) {
    costmap_wrapper_ = std::move(costmap_wrapper);
    options_ = options;
}

double TrajectoryCollisionChecker::PoseCost(double x, double y,
                                            double yaw) const {
    if (!costmap_wrapper_) {
        return 0.0;
    }
    auto* costmap = costmap_wrapper_->getCostmap();
    if (costmap == nullptr) {
        return 0.0;
    }

    map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>
        checker(costmap);
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!checker.worldToMap(x, y, mx, my)) {
        return static_cast<double>(map::costmap_2d::LETHAL_OBSTACLE);
    }

    if (options_.use_footprint_collision_check()) {
        const auto footprint = costmap_wrapper_->getRobotFootprint();
        return checker.footprintCostAtPose(x, y, yaw, footprint);
    }
    return static_cast<double>(checker.pointCost(mx, my));
}

bool TrajectoryCollisionChecker::IsTrajectoryFree(
    const std::vector<DifferentialDriveProblem::StateVector>& states) const {
    if (!options_.enable_collision_check() || states.empty()) {
        return true;
    }

    const double threshold = options_.collision_cost_threshold() > 0.0
                                 ? options_.collision_cost_threshold()
                                 : static_cast<double>(
                                       map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

    for (const auto& state : states) {
        if (PoseCost(state(0), state(1), state(2)) >= threshold) {
            return false;
        }
    }
    return true;
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
