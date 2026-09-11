/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/control/controller/mppi_controller/critic_function.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace critics {

/**
 * @brief Obstacle critic driven by MoGe LocalGrid (grid_map 2.5D).
 *        Reads layers "obstacle" / "traversability"; outside FOV / unknown → free.
 *        No-op when GridMapBuffer is empty so laser CostCritic can still run.
 */
class GridObstaclesCritic : public CriticFunction {
public:
    void initialize() override;
    void score(CriticData& data) override;

private:
    bool InCollision(const grid_map::GridMap& map, float x, float y) const;
    float SoftCost(const grid_map::GridMap& map, float x, float y) const;

    unsigned int power_{1};
    float critical_weight_{20.0F};
    float repulsion_weight_{1.5F};
    float collision_cost_{10000.0F};
    float robot_radius_m_{0.22F};
    float collision_margin_m_{0.1F};
    float repulsion_radius_m_{0.6F};
    float obstacle_threshold_{0.5F};
    std::string obstacle_layer_{"obstacle"};
    bool unknown_as_free_{true};
};

}  // namespace critics
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
