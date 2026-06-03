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

#include "autonomy/control/controller/teb_controller/cost_functions/acceleration_cost_function.hpp"
#include <algorithm>
#include "autonomy/common/math/math.hpp"
#include "autonomy/control/controller/teb_controller/config.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

namespace {

struct AccelerationCostFunctor {
    AccelerationCostFunctor(const TimedElasticBandConfig& config, double weight_acc_x,
                            double weight_acc_theta)
        : config_(config),
          sqrt_w_acc_x_(std::sqrt(weight_acc_x)),
          sqrt_w_acc_theta_(std::sqrt(weight_acc_theta)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const t1,
                    const T* const x2, const T* const y2, const T* const t2,
                    const T* const x3, const T* const y3, const T* const t3,
                    const T* const dt1, const T* const dt2, T* residual) const {
        const T dx1 = *x2 - *x1;
        const T dy1 = *y2 - *y1;
        const T dx2 = *x3 - *x2;
        const T dy2 = *y3 - *y2;
        T dist1 = ceres::sqrt(dx1 * dx1 + dy1 * dy1);
        T dist2 = ceres::sqrt(dx2 * dx2 + dy2 * dy2);
        const T ad1 = autonomy::common::NormalizeAngleDifference(*t2 - *t1);
        const T ad2 = autonomy::common::NormalizeAngleDifference(*t3 - *t2);
        if (config_.trajectory.exact_arc_length) {
            if (ad1 != T(0)) {
                dist1 =
                    ceres::abs(ad1 * dist1 / (T(2) * ceres::sin(ad1 / T(2))));
            }
            if (ad2 != T(0)) {
                dist2 =
                    ceres::abs(ad2 * dist2 / (T(2) * ceres::sin(ad2 / T(2))));
            }
        }
        T vel1 = dist1 / (*dt1);
        T vel2 = dist2 / (*dt2);
        vel1 *= autonomy::common::Sigmoid(
            dx1 * ceres::cos(*t1) + dy1 * ceres::sin(*t1), T(100.0));
        vel2 *= autonomy::common::Sigmoid(
            dx2 * ceres::cos(*t2) + dy2 * ceres::sin(*t2), T(100.0));
        const T acc_lin = (vel2 - vel1) * T(2) / (*dt1 + *dt2);
        const T omega1 = ad1 / (*dt1);
        const T omega2 = ad2 / (*dt2);
        const T acc_rot = (omega2 - omega1) * T(2) / (*dt1 + *dt2);
        residual[0] =
            T(sqrt_w_acc_x_) * ceres::abs((acc_lin)-autonomy::common::Clamp(
                                   (acc_lin),
                                   -(T(config_.robot.max_acceleration_x)) +
                                       (T(config_.optimization.penalty_epsilon)),
                                   (T(config_.robot.max_acceleration_x)) -
                                       (T(config_.optimization.penalty_epsilon))));
        residual[1] =
            T(sqrt_w_acc_theta_) * ceres::abs((acc_rot)-autonomy::common::Clamp(
                                       (acc_rot),
                                       -(T(config_.robot.max_angular_acceleration)) +
                                           (T(config_.optimization.penalty_epsilon)),
                                       (T(config_.robot.max_angular_acceleration)) -
                                           (T(config_.optimization.penalty_epsilon))));
        return true;
    }

    const TimedElasticBandConfig& config_;
    double sqrt_w_acc_x_;
    double sqrt_w_acc_theta_;
};

}  // namespace

ceres::CostFunction* CreateAccelerationCostFunction(const TimedElasticBandConfig& config,
                                                    double weight_acc_x,
                                                    double weight_acc_theta) {
    return new ceres::AutoDiffCostFunction<AccelerationCostFunctor, 2, 1, 1, 1,
                                           1, 1, 1, 1, 1, 1, 1, 1>(
        new AccelerationCostFunctor(config, weight_acc_x, weight_acc_theta));
}

namespace {

struct AccelerationStartCostFunctor {
    AccelerationStartCostFunctor(const TimedElasticBandConfig& config,
                                 double initial_velocity_x,
                                 double initial_angular_velocity,
                                 double weight_acc_x, double weight_acc_theta)
        : config_(config),
          initial_velocity_x_(initial_velocity_x),
          initial_angular_velocity_(initial_angular_velocity),
          sqrt_w_acc_x_(std::sqrt(weight_acc_x)),
          sqrt_w_acc_theta_(std::sqrt(weight_acc_theta)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const t1,
                    const T* const x2, const T* const y2, const T* const t2,
                    const T* const dt, T* residual) const {
        const T dx = *x2 - *x1;
        const T dy = *y2 - *y1;
        T dist = ceres::sqrt(dx * dx + dy * dy);
        const T ad = autonomy::common::NormalizeAngleDifference(*t2 - *t1);
        if (config_.trajectory.exact_arc_length && ad != T(0)) {
            dist = ceres::abs(ad * dist / (T(2) * ceres::sin(ad / T(2))));
        }
        T vel2 = dist / (*dt);
        vel2 *= autonomy::common::Sigmoid(
            dx * ceres::cos(*t1) + dy * ceres::sin(*t1), T(100.0));
        const T acc_lin = (vel2 - T(initial_velocity_x_)) / (*dt);
        const T omega2 = ad / (*dt);
        const T acc_rot = (omega2 - T(initial_angular_velocity_)) / (*dt);
        residual[0] =
            T(sqrt_w_acc_x_) * ceres::abs((acc_lin)-autonomy::common::Clamp(
                                   (acc_lin),
                                   -(T(config_.robot.max_acceleration_x)) +
                                       (T(config_.optimization.penalty_epsilon)),
                                   (T(config_.robot.max_acceleration_x)) -
                                       (T(config_.optimization.penalty_epsilon))));
        residual[1] =
            T(sqrt_w_acc_theta_) * ceres::abs((acc_rot)-autonomy::common::Clamp(
                                       (acc_rot),
                                       -(T(config_.robot.max_angular_acceleration)) +
                                           (T(config_.optimization.penalty_epsilon)),
                                       (T(config_.robot.max_angular_acceleration)) -
                                           (T(config_.optimization.penalty_epsilon))));
        return true;
    }

    const TimedElasticBandConfig& config_;
    double initial_velocity_x_;
    double initial_angular_velocity_;
    double sqrt_w_acc_x_;
    double sqrt_w_acc_theta_;
};

}  // namespace

ceres::CostFunction* CreateAccelerationStartCostFunction(
    const TimedElasticBandConfig& config, double initial_velocity_x,
    double initial_angular_velocity, double weight_acc_x,
    double weight_acc_theta) {
    return new ceres::AutoDiffCostFunction<AccelerationStartCostFunctor, 2, 1,
                                           1, 1, 1, 1, 1, 1>(
        new AccelerationStartCostFunctor(config, initial_velocity_x,
                                         initial_angular_velocity, weight_acc_x,
                                         weight_acc_theta));
}

namespace {

struct AccelerationGoalCostFunctor {
    AccelerationGoalCostFunctor(const TimedElasticBandConfig& config, double goal_velocity_x,
                                double goal_angular_velocity,
                                double weight_acc_x, double weight_acc_theta)
        : config_(config),
          goal_velocity_x_(goal_velocity_x),
          goal_angular_velocity_(goal_angular_velocity),
          sqrt_w_acc_x_(std::sqrt(weight_acc_x)),
          sqrt_w_acc_theta_(std::sqrt(weight_acc_theta)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const t1,
                    const T* const x2, const T* const y2, const T* const t2,
                    const T* const dt, T* residual) const {
        const T dx = *x2 - *x1;
        const T dy = *y2 - *y1;
        T dist = ceres::sqrt(dx * dx + dy * dy);
        const T ad = autonomy::common::NormalizeAngleDifference(*t2 - *t1);
        if (config_.trajectory.exact_arc_length && ad != T(0)) {
            dist = ceres::abs(ad * dist / (T(2) * ceres::sin(ad / T(2))));
        }
        T vel1 = dist / (*dt);
        vel1 *= autonomy::common::Sigmoid(
            dx * ceres::cos(*t1) + dy * ceres::sin(*t1), T(100.0));
        const T acc_lin = (T(goal_velocity_x_) - vel1) / (*dt);
        const T omega1 = ad / (*dt);
        const T acc_rot = (T(goal_angular_velocity_) - omega1) / (*dt);
        residual[0] =
            T(sqrt_w_acc_x_) * ceres::abs((acc_lin)-autonomy::common::Clamp(
                                   (acc_lin),
                                   -(T(config_.robot.max_acceleration_x)) +
                                       (T(config_.optimization.penalty_epsilon)),
                                   (T(config_.robot.max_acceleration_x)) -
                                       (T(config_.optimization.penalty_epsilon))));
        residual[1] =
            T(sqrt_w_acc_theta_) * ceres::abs((acc_rot)-autonomy::common::Clamp(
                                       (acc_rot),
                                       -(T(config_.robot.max_angular_acceleration)) +
                                           (T(config_.optimization.penalty_epsilon)),
                                       (T(config_.robot.max_angular_acceleration)) -
                                           (T(config_.optimization.penalty_epsilon))));
        return true;
    }

    const TimedElasticBandConfig& config_;
    double goal_velocity_x_;
    double goal_angular_velocity_;
    double sqrt_w_acc_x_;
    double sqrt_w_acc_theta_;
};

}  // namespace

ceres::CostFunction* CreateAccelerationGoalCostFunction(
    const TimedElasticBandConfig& config, double goal_velocity_x,
    double goal_angular_velocity, double weight_acc_x,
    double weight_acc_theta) {
    return new ceres::AutoDiffCostFunction<AccelerationGoalCostFunctor, 2, 1, 1,
                                           1, 1, 1, 1, 1>(
        new AccelerationGoalCostFunctor(config, goal_velocity_x,
                                        goal_angular_velocity, weight_acc_x,
                                        weight_acc_theta));
}

namespace {

struct AccelerationHolonomicCostFunctor {
    AccelerationHolonomicCostFunctor(const TimedElasticBandConfig& config, double weight_x,
                                     double weight_y, double weight_theta)
        : config_(config),
          sqrt_w_x_(std::sqrt(weight_x)),
          sqrt_w_y_(std::sqrt(weight_y)),
          sqrt_w_theta_(std::sqrt(weight_theta)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const t1,
                    const T* const x2, const T* const y2, const T* const t2,
                    const T* const x3, const T* const y3, const T* const t3,
                    const T* const dt1, const T* const dt2, T* residual) const {
        const T dx1 = *x2 - *x1;
        const T dy1 = *y2 - *y1;
        const T dx2 = *x3 - *x2;
        const T dy2 = *y3 - *y2;
        const T c1 = ceres::cos(*t1);
        const T s1 = ceres::sin(*t1);
        const T c2 = ceres::cos(*t2);
        const T s2 = ceres::sin(*t2);
        const T v1x = (c1 * dx1 + s1 * dy1) / (*dt1);
        const T v1y = (-s1 * dx1 + c1 * dy1) / (*dt1);
        const T v2x = (c2 * dx2 + s2 * dy2) / (*dt2);
        const T v2y = (-s2 * dx2 + c2 * dy2) / (*dt2);
        const T dt12 = *dt1 + *dt2;
        const T acc_x = (v2x - v1x) * T(2) / dt12;
        const T acc_y = (v2y - v1y) * T(2) / dt12;
        const T o1 =
            autonomy::common::NormalizeAngleDifference(*t2 - *t1) / (*dt1);
        const T o2 =
            autonomy::common::NormalizeAngleDifference(*t3 - *t2) / (*dt2);
        const T acc_rot = (o2 - o1) * T(2) / dt12;
        residual[0] =
            T(sqrt_w_x_) * ceres::abs((acc_x)-autonomy::common::Clamp(
                               (acc_x),
                               -(T(config_.robot.max_acceleration_x)) +
                                   (T(config_.optimization.penalty_epsilon)),
                               (T(config_.robot.max_acceleration_x)) -
                                   (T(config_.optimization.penalty_epsilon))));
        residual[1] =
            T(sqrt_w_y_) * ceres::abs((acc_y)-autonomy::common::Clamp(
                               (acc_y),
                               -(T(config_.robot.max_acceleration_y)) +
                                   (T(config_.optimization.penalty_epsilon)),
                               (T(config_.robot.max_acceleration_y)) -
                                   (T(config_.optimization.penalty_epsilon))));
        residual[2] =
            T(sqrt_w_theta_) * ceres::abs((acc_rot)-autonomy::common::Clamp(
                                   (acc_rot),
                                   -(T(config_.robot.max_angular_acceleration)) +
                                       (T(config_.optimization.penalty_epsilon)),
                                   (T(config_.robot.max_angular_acceleration)) -
                                       (T(config_.optimization.penalty_epsilon))));
        return true;
    }

    const TimedElasticBandConfig& config_;
    double sqrt_w_x_;
    double sqrt_w_y_;
    double sqrt_w_theta_;
};

}  // namespace

ceres::CostFunction* CreateAccelerationHolonomicCostFunction(
    const TimedElasticBandConfig& config, double weight_x, double weight_y,
    double weight_theta) {
    return new ceres::AutoDiffCostFunction<AccelerationHolonomicCostFunctor, 3,
                                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
        new AccelerationHolonomicCostFunctor(config, weight_x, weight_y,
                                             weight_theta));
}

namespace {

struct AccelerationHolonomicStartCostFunctor {
    AccelerationHolonomicStartCostFunctor(const TimedElasticBandConfig& config,
                                          double initial_velocity_x,
                                          double initial_velocity_y,
                                          double initial_angular_velocity,
                                          double weight_x, double weight_y,
                                          double weight_theta)
        : config_(config),
          initial_velocity_x_(initial_velocity_x),
          initial_velocity_y_(initial_velocity_y),
          initial_angular_velocity_(initial_angular_velocity),
          sqrt_w_x_(std::sqrt(weight_x)),
          sqrt_w_y_(std::sqrt(weight_y)),
          sqrt_w_theta_(std::sqrt(weight_theta)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const t1,
                    const T* const x2, const T* const y2, const T* const t2,
                    const T* const dt, T* residual) const {
        const T dx = *x2 - *x1;
        const T dy = *y2 - *y1;
        const T c1 = ceres::cos(*t1);
        const T s1 = ceres::sin(*t1);
        const T v2x = (c1 * dx + s1 * dy) / (*dt);
        const T v2y = (-s1 * dx + c1 * dy) / (*dt);
        const T acc_x = (v2x - T(initial_velocity_x_)) / (*dt);
        const T acc_y = (v2y - T(initial_velocity_y_)) / (*dt);
        const T omega2 =
            autonomy::common::NormalizeAngleDifference(*t2 - *t1) / (*dt);
        const T acc_rot = (omega2 - T(initial_angular_velocity_)) / (*dt);
        residual[0] =
            T(sqrt_w_x_) * ceres::abs((acc_x)-autonomy::common::Clamp(
                               (acc_x),
                               -(T(config_.robot.max_acceleration_x)) +
                                   (T(config_.optimization.penalty_epsilon)),
                               (T(config_.robot.max_acceleration_x)) -
                                   (T(config_.optimization.penalty_epsilon))));
        residual[1] =
            T(sqrt_w_y_) * ceres::abs((acc_y)-autonomy::common::Clamp(
                               (acc_y),
                               -(T(config_.robot.max_acceleration_y)) +
                                   (T(config_.optimization.penalty_epsilon)),
                               (T(config_.robot.max_acceleration_y)) -
                                   (T(config_.optimization.penalty_epsilon))));
        residual[2] =
            T(sqrt_w_theta_) * ceres::abs((acc_rot)-autonomy::common::Clamp(
                                   (acc_rot),
                                   -(T(config_.robot.max_angular_acceleration)) +
                                       (T(config_.optimization.penalty_epsilon)),
                                   (T(config_.robot.max_angular_acceleration)) -
                                       (T(config_.optimization.penalty_epsilon))));
        return true;
    }

    const TimedElasticBandConfig& config_;
    double initial_velocity_x_;
    double initial_velocity_y_;
    double initial_angular_velocity_;
    double sqrt_w_x_;
    double sqrt_w_y_;
    double sqrt_w_theta_;
};

}  // namespace

ceres::CostFunction* CreateAccelerationHolonomicStartCostFunction(
    const TimedElasticBandConfig& config, double initial_velocity_x,
    double initial_velocity_y, double initial_angular_velocity, double weight_x,
    double weight_y, double weight_theta) {
    return new ceres::AutoDiffCostFunction<
        AccelerationHolonomicStartCostFunctor, 3, 1, 1, 1, 1, 1, 1, 1>(
        new AccelerationHolonomicStartCostFunctor(
            config, initial_velocity_x, initial_velocity_y,
            initial_angular_velocity, weight_x, weight_y, weight_theta));
}

namespace {

struct AccelerationHolonomicGoalCostFunctor {
    AccelerationHolonomicGoalCostFunctor(const TimedElasticBandConfig& config,
                                         double goal_velocity_x,
                                         double goal_velocity_y,
                                         double goal_angular_velocity,
                                         double weight_x, double weight_y,
                                         double weight_theta)
        : config_(config),
          goal_velocity_x_(goal_velocity_x),
          goal_velocity_y_(goal_velocity_y),
          goal_angular_velocity_(goal_angular_velocity),
          sqrt_w_x_(std::sqrt(weight_x)),
          sqrt_w_y_(std::sqrt(weight_y)),
          sqrt_w_theta_(std::sqrt(weight_theta)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const t1,
                    const T* const x2, const T* const y2, const T* const t2,
                    const T* const dt, T* residual) const {
        const T dx = *x2 - *x1;
        const T dy = *y2 - *y1;
        const T c1 = ceres::cos(*t1);
        const T s1 = ceres::sin(*t1);
        const T v1x = (c1 * dx + s1 * dy) / (*dt);
        const T v1y = (-s1 * dx + c1 * dy) / (*dt);
        const T acc_x = (T(goal_velocity_x_) - v1x) / (*dt);
        const T acc_y = (T(goal_velocity_y_) - v1y) / (*dt);
        const T omega1 =
            autonomy::common::NormalizeAngleDifference(*t2 - *t1) / (*dt);
        const T acc_rot = (T(goal_angular_velocity_) - omega1) / (*dt);
        residual[0] =
            T(sqrt_w_x_) * ceres::abs((acc_x)-autonomy::common::Clamp(
                               (acc_x),
                               -(T(config_.robot.max_acceleration_x)) +
                                   (T(config_.optimization.penalty_epsilon)),
                               (T(config_.robot.max_acceleration_x)) -
                                   (T(config_.optimization.penalty_epsilon))));
        residual[1] =
            T(sqrt_w_y_) * ceres::abs((acc_y)-autonomy::common::Clamp(
                               (acc_y),
                               -(T(config_.robot.max_acceleration_y)) +
                                   (T(config_.optimization.penalty_epsilon)),
                               (T(config_.robot.max_acceleration_y)) -
                                   (T(config_.optimization.penalty_epsilon))));
        residual[2] =
            T(sqrt_w_theta_) * ceres::abs((acc_rot)-autonomy::common::Clamp(
                                   (acc_rot),
                                   -(T(config_.robot.max_angular_acceleration)) +
                                       (T(config_.optimization.penalty_epsilon)),
                                   (T(config_.robot.max_angular_acceleration)) -
                                       (T(config_.optimization.penalty_epsilon))));
        return true;
    }

    const TimedElasticBandConfig& config_;
    double goal_velocity_x_;
    double goal_velocity_y_;
    double goal_angular_velocity_;
    double sqrt_w_x_;
    double sqrt_w_y_;
    double sqrt_w_theta_;
};

}  // namespace

ceres::CostFunction* CreateAccelerationHolonomicGoalCostFunction(
    const TimedElasticBandConfig& config, double goal_velocity_x, double goal_velocity_y,
    double goal_angular_velocity, double weight_x, double weight_y,
    double weight_theta) {
    return new ceres::AutoDiffCostFunction<AccelerationHolonomicGoalCostFunctor,
                                           3, 1, 1, 1, 1, 1, 1, 1>(
        new AccelerationHolonomicGoalCostFunctor(
            config, goal_velocity_x, goal_velocity_y, goal_angular_velocity,
            weight_x, weight_y, weight_theta));
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
