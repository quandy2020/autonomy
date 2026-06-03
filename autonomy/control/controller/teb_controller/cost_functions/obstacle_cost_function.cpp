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

#include "autonomy/control/controller/teb_controller/cost_functions/obstacle_cost_function.hpp"
#include <algorithm>
#include <cmath>
#include "autonomy/common/math/math.hpp"
#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"
#include "autonomy/control/controller/teb_controller/pose2d_utils.hpp"
#include "autonomy/control/controller/teb_controller/footprint.hpp"
#include "autonomy/control/controller/teb_controller/config.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

namespace {

struct ObstacleCostFunctor {
    ObstacleCostFunctor(const TimedElasticBandConfig& config,
                        const RobotFootprint* robot_model,
                        const Obstacle* obstacle, double weight)
        : config_(config),
          robot_model_(robot_model),
          obstacle_(obstacle),
          sqrt_weight_(std::sqrt(weight)) {}

    bool operator()(const double* const x, const double* const y,
                    const double* const theta, double* residual) const {
        const Pose2D pose{*x, *y, *theta};
        double dist = robot_model_->CalculateDistance(pose, obstacle_);
        double penalty = ceres::abs(
            (dist)-ceres::fmax((dist), (config_.obstacles.min_obstacle_dist) +
                                           (config_.optimization.penalty_epsilon)));
        if (config_.optimization.obstacle_cost_exponent != 1.0 &&
            config_.obstacles.min_obstacle_dist > 0.0) {
            penalty = config_.obstacles.min_obstacle_dist *
                      std::pow(penalty / config_.obstacles.min_obstacle_dist,
                               config_.optimization.obstacle_cost_exponent);
        }
        residual[0] = sqrt_weight_ * penalty;
        return true;
    }

    const TimedElasticBandConfig& config_;
    const RobotFootprint* robot_model_;
    const Obstacle* obstacle_;
    double sqrt_weight_;
};

}  // namespace

ceres::CostFunction* CreateObstacleCostFunction(
    const TimedElasticBandConfig& config, const RobotFootprint* robot_model,
    const Obstacle* obstacle, double weight) {
    return new ceres::NumericDiffCostFunction<ObstacleCostFunctor,
                                              ceres::CENTRAL, 1, 1, 1, 1>(
        new ObstacleCostFunctor(config, robot_model, obstacle, weight));
}

namespace {

struct InflatedObstacleCostFunctor {
    InflatedObstacleCostFunctor(const TimedElasticBandConfig& config,
                                const RobotFootprint* robot_model,
                                const Obstacle* obstacle,
                                double weight_obstacle, double weight_inflation)
        : config_(config),
          robot_model_(robot_model),
          obstacle_(obstacle),
          sqrt_w0_(std::sqrt(weight_obstacle)),
          sqrt_w1_(std::sqrt(weight_inflation)) {}

    bool operator()(const double* const x, const double* const y,
                    const double* const theta, double* residual) const {
        const Pose2D pose{*x, *y, *theta};
        double dist = robot_model_->CalculateDistance(pose, obstacle_);
        double penalty0 = ceres::abs(
            (dist)-ceres::fmax((dist), (config_.obstacles.min_obstacle_dist) +
                                           (config_.optimization.penalty_epsilon)));
        if (config_.optimization.obstacle_cost_exponent != 1.0 &&
            config_.obstacles.min_obstacle_dist > 0.0) {
            penalty0 = config_.obstacles.min_obstacle_dist *
                       std::pow(penalty0 / config_.obstacles.min_obstacle_dist,
                                config_.optimization.obstacle_cost_exponent);
        }
        residual[0] = sqrt_w0_ * penalty0;
        residual[1] =
            sqrt_w1_ * ceres::abs((dist)-ceres::fmax(
                           (dist), (config_.obstacles.inflation_dist) + (0.0)));
        return true;
    }

    const TimedElasticBandConfig& config_;
    const RobotFootprint* robot_model_;
    const Obstacle* obstacle_;
    double sqrt_w0_;
    double sqrt_w1_;
};

}  // namespace

ceres::CostFunction* CreateInflatedObstacleCostFunction(
    const TimedElasticBandConfig& config, const RobotFootprint* robot_model,
    const Obstacle* obstacle, double weight_obstacle, double weight_inflation) {
    return new ceres::NumericDiffCostFunction<InflatedObstacleCostFunctor,
                                              ceres::CENTRAL, 2, 1, 1, 1>(
        new InflatedObstacleCostFunctor(config, robot_model, obstacle,
                                        weight_obstacle, weight_inflation));
}

namespace {

struct DynamicObstacleCostFunctor {
    DynamicObstacleCostFunctor(const TimedElasticBandConfig& config,
                               const RobotFootprint* robot_model,
                               const Obstacle* obstacle, double time_offset,
                               double weight_obstacle, double weight_inflation)
        : config_(config),
          robot_model_(robot_model),
          obstacle_(obstacle),
          t_(time_offset),
          sqrt_w0_(std::sqrt(weight_obstacle)),
          sqrt_w1_(std::sqrt(weight_inflation)) {}

    bool operator()(const double* const x, const double* const y,
                    const double* const theta, double* residual) const {
        const Pose2D pose{*x, *y, *theta};
        double dist =
            robot_model_->EstimateSpatioTemporalDistance(pose, obstacle_, t_);
        residual[0] =
            sqrt_w0_ * ceres::abs((dist)-ceres::fmax(
                           (dist), (config_.obstacles.min_obstacle_dist) +
                                       (config_.optimization.penalty_epsilon)));
        residual[1] =
            sqrt_w1_ *
            ceres::abs((dist)-ceres::fmax(
                (dist),
                (config_.obstacles.dynamic_obstacle_inflation_dist) + (0.0)));
        return true;
    }

    const TimedElasticBandConfig& config_;
    const RobotFootprint* robot_model_;
    const Obstacle* obstacle_;
    double t_;
    double sqrt_w0_;
    double sqrt_w1_;
};

}  // namespace

ceres::CostFunction* CreateDynamicObstacleCostFunction(
    const TimedElasticBandConfig& config, const RobotFootprint* robot_model,
    const Obstacle* obstacle, double time_offset, double weight_obstacle,
    double weight_inflation) {
    return new ceres::NumericDiffCostFunction<DynamicObstacleCostFunctor,
                                              ceres::CENTRAL, 2, 1, 1, 1>(
        new DynamicObstacleCostFunctor(config, robot_model, obstacle,
                                       time_offset, weight_obstacle,
                                       weight_inflation));
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
