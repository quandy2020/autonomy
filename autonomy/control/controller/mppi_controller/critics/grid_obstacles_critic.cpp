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

#include "autonomy/control/controller/mppi_controller/critics/grid_obstacles_critic.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"

#include <cmath>
#include <limits>

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace critics {
namespace {

bool LayerOccupied(const grid_map::GridMap& map, const std::string& layer,
                   const grid_map::Position& pos, float threshold,
                   bool unknown_as_free) {
    if (!map.isInside(pos) || !map.exists(layer)) {
        return !unknown_as_free;
    }
    const float value = map.atPosition(layer, pos);
    if (!std::isfinite(value)) {
        return !unknown_as_free;
    }
    return value > threshold;
}

}  // namespace

void GridObstaclesCritic::initialize() {
    power_ = 1;
    critical_weight_ = 20.0F;
    repulsion_weight_ = 1.5F;
    collision_cost_ = 10000.0F;
    robot_radius_m_ = 0.22F;
    collision_margin_m_ = 0.1F;
    repulsion_radius_m_ = 0.6F;
    obstacle_threshold_ = 0.5F;
    obstacle_layer_ = "obstacle";
    unknown_as_free_ = true;
    enabled_ = true;

    if (options_ && options_->has_grid_obstacles_critic()) {
        const auto& critic = options_->grid_obstacles_critic();
        enabled_ = critic.enabled();
        power_ = static_cast<unsigned int>(std::max(1, critic.cost_power()));
        critical_weight_ = static_cast<float>(critic.critical_weight());
        repulsion_weight_ = static_cast<float>(critic.repulsion_weight());
        collision_cost_ = static_cast<float>(critic.collision_cost());
        if (critic.robot_radius_m() > 0.0) {
            robot_radius_m_ = static_cast<float>(critic.robot_radius_m());
        }
        if (critic.collision_margin_m() > 0.0) {
            collision_margin_m_ = static_cast<float>(critic.collision_margin_m());
        }
        if (critic.repulsion_radius_m() > 0.0) {
            repulsion_radius_m_ =
                static_cast<float>(critic.repulsion_radius_m());
        }
        if (critic.obstacle_threshold() > 0.0) {
            obstacle_threshold_ =
                static_cast<float>(critic.obstacle_threshold());
        }
        if (!critic.obstacle_layer().empty()) {
            obstacle_layer_ = critic.obstacle_layer();
        }
        unknown_as_free_ = critic.unknown_as_free();
    }

    if (costmap_ros_ && costmap_ros_->getRobotRadius() > 0.0) {
        robot_radius_m_ =
            static_cast<float>(costmap_ros_->getRobotRadius());
    }

    AINFO << "GridObstaclesCritic: layer=" << obstacle_layer_
          << " radius=" << robot_radius_m_
          << " (grid_map 2.5D, no costmap inject)";
}

bool GridObstaclesCritic::InCollision(const grid_map::GridMap& map, float x,
                                      float y) const {
    const float check_r = robot_radius_m_ + collision_margin_m_;
    constexpr int kSamples = 8;
    if (LayerOccupied(map, obstacle_layer_, grid_map::Position(x, y),
                      obstacle_threshold_, unknown_as_free_)) {
        return true;
    }
    for (int i = 0; i < kSamples; ++i) {
        const float angle =
            static_cast<float>(i) * (2.0F * static_cast<float>(M_PI) /
                                     static_cast<float>(kSamples));
        const float sx = x + check_r * std::cos(angle);
        const float sy = y + check_r * std::sin(angle);
        if (LayerOccupied(map, obstacle_layer_, grid_map::Position(sx, sy),
                          obstacle_threshold_, unknown_as_free_)) {
            return true;
        }
    }
    return false;
}

float GridObstaclesCritic::SoftCost(const grid_map::GridMap& map, float x,
                                    float y) const {
    if (!(repulsion_radius_m_ > robot_radius_m_)) {
        return 0.0F;
    }
    float min_clearance = repulsion_radius_m_;
    constexpr int kRings = 3;
    constexpr int kSamples = 12;
    for (int ring = 1; ring <= kRings; ++ring) {
        const float r = robot_radius_m_ +
                        (repulsion_radius_m_ - robot_radius_m_) *
                            static_cast<float>(ring) /
                            static_cast<float>(kRings);
        for (int i = 0; i < kSamples; ++i) {
            const float angle =
                static_cast<float>(i) *
                (2.0F * static_cast<float>(M_PI) / static_cast<float>(kSamples));
            const float sx = x + r * std::cos(angle);
            const float sy = y + r * std::sin(angle);
            if (LayerOccupied(map, obstacle_layer_, grid_map::Position(sx, sy),
                              obstacle_threshold_, /*unknown_as_free=*/true)) {
                min_clearance = std::min(min_clearance, r);
            }
        }
    }
    return std::max(0.0F, repulsion_radius_m_ - min_clearance);
}

void GridObstaclesCritic::score(CriticData& data) {
    if (!enabled_ || !grid_map_) {
        return;
    }

    grid_map::GridMap map;
    if (!grid_map_->Copy(&map) || !map.exists(obstacle_layer_)) {
        return;
    }

    Eigen::ArrayXf raw_cost = Eigen::ArrayXf::Zero(data.costs.size());
    Eigen::ArrayXf repulsive_cost = Eigen::ArrayXf::Zero(data.costs.size());

    const unsigned int traj_len = data.trajectories.x.cols();
    const unsigned int batch_size = data.trajectories.x.rows();
    bool all_trajectories_collide = true;

    for (unsigned int i = 0; i != batch_size; ++i) {
        bool trajectory_collide = false;
        float traj_cost = 0.0F;
        float soft = 0.0F;
        for (unsigned int j = 0; j != traj_len; ++j) {
            const float x = data.trajectories.x(i, j);
            const float y = data.trajectories.y(i, j);
            if (InCollision(map, x, y)) {
                trajectory_collide = true;
                break;
            }
            soft += SoftCost(map, x, y);
        }
        if (!trajectory_collide) {
            all_trajectories_collide = false;
        }
        raw_cost(i) = trajectory_collide ? collision_cost_ : traj_cost;
        repulsive_cost(i) = soft;
    }

    auto repulsive_normalized =
        (repulsive_cost - repulsive_cost.minCoeff()) /
        static_cast<float>(std::max(1U, traj_len));

    if (power_ > 1U) {
        data.costs += ((critical_weight_ * raw_cost) +
                       (repulsion_weight_ * repulsive_normalized))
                          .pow(power_);
    } else {
        data.costs += (critical_weight_ * raw_cost) +
                      (repulsion_weight_ * repulsive_normalized);
    }
    data.fail_flag = data.fail_flag || all_trajectories_collide;
}

}  // namespace critics
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
