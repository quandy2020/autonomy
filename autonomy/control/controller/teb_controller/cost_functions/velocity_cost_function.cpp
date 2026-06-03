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

#include <algorithm>
#include <cmath>

#include "autonomy/common/math/math.hpp"
#include "autonomy/control/controller/teb_controller/cost_functions/velocity_cost_function.hpp"
#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"
#include "autonomy/control/controller/teb_controller/pose2d_utils.hpp"
#include "autonomy/control/controller/teb_controller/footprint.hpp"
#include "autonomy/control/controller/teb_controller/config.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

namespace {

struct VelocityCostFunctor {
    VelocityCostFunctor(const TimedElasticBandConfig& config, double weight_vx,
                        double weight_omega)
        : config_(config),
          sqrt_w_vx_(std::sqrt(weight_vx)),
          sqrt_w_omega_(std::sqrt(weight_omega)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const t1,
                    const T* const x2, const T* const y2, const T* const t2,
                    const T* const dt, T* residual) const {
        const T dx = *x2 - *x1;
        const T dy = *y2 - *y1;
        T dist = ceres::sqrt(dx * dx + dy * dy);
        const T angle_diff =
            autonomy::common::NormalizeAngleDifference(*t2 - *t1);
        if (config_.trajectory.exact_arc_length && angle_diff != T(0)) {
            dist = ceres::abs(angle_diff * dist /
                              (T(2) * ceres::sin(angle_diff / T(2))));
        }
        T vel = dist / (*dt);
        vel *= autonomy::common::Sigmoid(
            dx * ceres::cos(*t1) + dy * ceres::sin(*t1), T(100.0));
        const T omega = angle_diff / (*dt);
        residual[0] =
            T(sqrt_w_vx_) * ceres::abs((vel)-autonomy::common::Clamp(
                                (vel),
                                (T(-config_.robot.max_velocity_x_backwards) +
                                 T(config_.optimization.penalty_epsilon)),
                                (T(config_.robot.max_velocity_x) -
                                 T(config_.optimization.penalty_epsilon))));
        residual[1] =
            T(sqrt_w_omega_) * ceres::abs((omega)-autonomy::common::Clamp(
                                   (omega),
                                   -(T(config_.robot.max_angular_velocity)) +
                                       (T(config_.optimization.penalty_epsilon)),
                                   (T(config_.robot.max_angular_velocity)) -
                                       (T(config_.optimization.penalty_epsilon))));
        return true;
    }

    const TimedElasticBandConfig& config_;
    double sqrt_w_vx_;
    double sqrt_w_omega_;
};

}  // namespace

ceres::CostFunction* CreateVelocityCostFunction(const TimedElasticBandConfig& config,
                                                double weight_vx,
                                                double weight_omega) {
    return new ceres::AutoDiffCostFunction<VelocityCostFunctor, 2, 1, 1, 1, 1,
                                           1, 1, 1>(
        new VelocityCostFunctor(config, weight_vx, weight_omega));
}

namespace {

struct VelocityHolonomicCostFunctor {
    VelocityHolonomicCostFunctor(const TimedElasticBandConfig& config, double weight_vx,
                                 double weight_vy, double weight_omega)
        : config_(config),
          sqrt_w_vx_(std::sqrt(weight_vx)),
          sqrt_w_vy_(std::sqrt(weight_vy)),
          sqrt_w_omega_(std::sqrt(weight_omega)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const t1,
                    const T* const x2, const T* const y2, const T* const t2,
                    const T* const dt, T* residual) const {
        const T dx = *x2 - *x1;
        const T dy = *y2 - *y1;
        const T cos1 = ceres::cos(*t1);
        const T sin1 = ceres::sin(*t1);
        const T velocity_x = (cos1 * dx + sin1 * dy) / (*dt);
        const T velocity_y = (-sin1 * dx + cos1 * dy) / (*dt);
        const T omega =
            autonomy::common::NormalizeAngleDifference(*t2 - *t1) / (*dt);
        residual[0] =
            T(sqrt_w_vx_) * ceres::abs((velocity_x)-autonomy::common::Clamp(
                                (velocity_x),
                                (T(-config_.robot.max_velocity_x_backwards) +
                                 T(config_.optimization.penalty_epsilon)),
                                (T(config_.robot.max_velocity_x) -
                                 T(config_.optimization.penalty_epsilon))));
        residual[1] =
            T(sqrt_w_vy_) *
            ceres::abs((velocity_y)-autonomy::common::Clamp(
                (velocity_y), -(T(config_.robot.max_velocity_y)) + (T(0.0)),
                (T(config_.robot.max_velocity_y)) - (T(0.0))));
        residual[2] =
            T(sqrt_w_omega_) * ceres::abs((omega)-autonomy::common::Clamp(
                                   (omega),
                                   -(T(config_.robot.max_angular_velocity)) +
                                       (T(config_.optimization.penalty_epsilon)),
                                   (T(config_.robot.max_angular_velocity)) -
                                       (T(config_.optimization.penalty_epsilon))));
        return true;
    }

    const TimedElasticBandConfig& config_;
    double sqrt_w_vx_;
    double sqrt_w_vy_;
    double sqrt_w_omega_;
};

}  // namespace

ceres::CostFunction* CreateVelocityHolonomicCostFunction(
    const TimedElasticBandConfig& config, double weight_vx, double weight_vy,
    double weight_omega) {
    return new ceres::AutoDiffCostFunction<VelocityHolonomicCostFunctor, 3, 1,
                                           1, 1, 1, 1, 1, 1>(
        new VelocityHolonomicCostFunctor(config, weight_vx, weight_vy,
                                         weight_omega));
}

namespace {

struct VelocityObstacleRatioCostFunctor {
    VelocityObstacleRatioCostFunctor(const TimedElasticBandConfig& config,
                                     const RobotFootprint* robot_model,
                                     const Obstacle* obstacle, double weight)
        : config_(config),
          robot_model_(robot_model),
          obstacle_(obstacle),
          sqrt_w_(std::sqrt(weight)) {}

    bool operator()(const double* const x1, const double* const y1,
                    const double* const t1, const double* const x2,
                    const double* const y2, const double* const t2,
                    const double* const dt, double* residual) const {
        const double dx = *x2 - *x1;
        const double dy = *y2 - *y1;
        double dist = std::hypot(dx, dy);
        const double angle_diff =
            autonomy::common::NormalizeAngleDifference(*t2 - *t1);
        if (config_.trajectory.exact_arc_length && angle_diff != 0.0) {
            dist = std::fabs(angle_diff * dist /
                             (2.0 * std::sin(angle_diff / 2.0)));
        }
        double vel = dist / (*dt);
        vel *= autonomy::common::Sigmoid(
            dx * std::cos(*t1) + dy * std::sin(*t1), 100.0);
        const double omega = angle_diff / (*dt);

        const Pose2D pose1(*x1, *y1, *t1);
        const double dist_to_obstacle =
            robot_model_->CalculateDistance(pose1, obstacle_);

        double ratio = 0.0;
        if (dist_to_obstacle >=
            config_.obstacles.obstacle_proximity_upper_bound) {
            ratio = 1.0;
        } else if (dist_to_obstacle >
                   config_.obstacles.obstacle_proximity_lower_bound) {
            ratio = (dist_to_obstacle -
                     config_.obstacles.obstacle_proximity_lower_bound) /
                    (config_.obstacles.obstacle_proximity_upper_bound -
                     config_.obstacles.obstacle_proximity_lower_bound);
        }
        ratio *= config_.obstacles.obstacle_proximity_max_velocity_ratio;

        const double max_vel_fwd = ratio * config_.robot.max_velocity_x;
        const double max_omega = ratio * config_.robot.max_angular_velocity;
        residual[0] = sqrt_w_ * ceres::abs((vel)-autonomy::common::Clamp(
                                    (vel), -(max_vel_fwd) + (0.0),
                                    (max_vel_fwd) - (0.0)));
        residual[1] =
            sqrt_w_ * ceres::abs((omega)-autonomy::common::Clamp(
                          (omega), -(max_omega) + (0.0), (max_omega) - (0.0)));
        return true;
    }

    const TimedElasticBandConfig& config_;
    const RobotFootprint* robot_model_;
    const Obstacle* obstacle_;
    double sqrt_w_;
};

}  // namespace

ceres::CostFunction* CreateVelocityObstacleRatioCostFunction(
    const TimedElasticBandConfig& config, const RobotFootprint* robot_model,
    const Obstacle* obstacle, double weight) {
    return new ceres::NumericDiffCostFunction<VelocityObstacleRatioCostFunctor,
                                              ceres::CENTRAL, 2, 1, 1, 1, 1, 1,
                                              1, 1>(
        new VelocityObstacleRatioCostFunctor(config, robot_model, obstacle,
                                             weight));
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
