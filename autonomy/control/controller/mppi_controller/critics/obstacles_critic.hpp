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

#include <memory>
#include <string>

#include "autonomy/control/controller/mppi_controller/critic_function.hpp"
#include "autonomy/control/controller/mppi_controller/models/state.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"
#include "autonomy/map/costmap_2d/layers/inflation_layer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace critics {

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for avoiding obstacles, allowing it to
 * deviate off the planned path. This is important to tune in tandem with
 * PathAlign to make a balance between path-tracking and dynamic obstacle
 * avoidance capabilities as desirable for a particular application
 */
class ObstaclesCritic : public CriticFunction
{
public:
    /**
     * @brief Initialize critic
     */
    void initialize() override;

    /**
     * @brief Evaluate cost related to obstacle avoidance
     *
     * @param costs [out] add obstacle cost values to this tensor
     */
    void score(CriticData& data) override;

protected:
    /**
     * @brief Checks if cost represents a collision
     * @param cost Costmap cost
     * @return bool if in collision
     */
    inline bool inCollision(float cost) const;

    /**
     * @brief cost at a robot pose
     * @param x X of pose
     * @param y Y of pose
     * @param theta theta of pose
     * @return Collision information at pose
     */
    inline CollisionCost costAtPose(float x, float y, float theta);

    /**
     * @brief Distance to obstacle from cost
     * @param cost Costmap cost
     * @return float Distance to the obstacle represented by cost
     */
    inline float distanceToObstacle(const CollisionCost& cost);

    /**
     * @brief Find the min cost of the inflation decay function for which the
     * robot MAY be in collision in any orientation
     * @param costmap Costmap2DWrapper to get minimum inscribed cost (e.g. 128
     * in inflation layer documentation)
     * @return double circumscribed cost, any higher than this and need to do
     * full footprint collision checking since some element of the robot could
     * be in collision
     */
    float findCircumscribedCost(std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap);

protected:
    map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*> collision_checker_{nullptr};

    bool consider_footprint_{true};
    float collision_cost_{0};
    float inflation_scale_factor_{0}, inflation_radius_{0};

    float possible_collision_cost_;
    float collision_margin_distance_;
    float near_goal_distance_;
    float circumscribed_cost_{0}, circumscribed_radius_{0};

    unsigned int power_{0};
    float repulsion_weight_, critical_weight_{0};
    bool enforce_path_inversion_{false};
    std::string inflation_layer_name_;
};

}  // namespace critics
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy