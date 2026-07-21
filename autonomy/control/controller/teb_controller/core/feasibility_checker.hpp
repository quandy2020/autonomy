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

#pragma once

#include <vector>

#include "autonomy/control/controller/teb_controller/core/teb_core.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

/**
 * @brief Costmap footprint model for TEB trajectory feasibility checks.
 *
 * Wraps FootprintCollisionChecker. Returns -1 on lethal / out-of-map to match
 * TebOptimalPlanner::isTrajectoryFeasible (ROS1 CostmapModel convention).
 */
class CostmapFeasibilityModel
{
public:
    CostmapFeasibilityModel() = default;

    explicit CostmapFeasibilityModel(map::costmap_2d::Costmap2D* costmap)
        : checker_(costmap) {}

    virtual ~CostmapFeasibilityModel() = default;

    void setCostmap(map::costmap_2d::Costmap2D* costmap) {
        checker_.setCostmap(costmap);
    }

    /**
     * @return cost in [0, 253], or -1 if lethal / outside map / empty footprint
     */
    virtual double footprintCost(double x, double y, double theta,
                                 const std::vector<Point>& footprint,
                                 double /*inscribed_radius*/,
                                 double /*circumscribed_radius*/) const {
        if (footprint.empty() || checker_.getCostmap() == nullptr) {
            return -1.0;
        }
        const double cost =
            checker_.footprintCostAtPose(x, y, theta, footprint);
        if (cost < 0.0 ||
            cost >= static_cast<double>(map::costmap_2d::LETHAL_OBSTACLE)) {
            return -1.0;
        }
        return cost;
    }

private:
    mutable map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>
        checker_{nullptr};
};

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
