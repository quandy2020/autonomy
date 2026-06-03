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

#include "autonomy/control/controller/teb_controller/cost_functions/kinematics_cost_function.hpp"
#include "autonomy/common/math/math.hpp"
#include "autonomy/control/controller/teb_controller/config.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

namespace {

struct KinematicsDiffDriveCostFunctor {
    KinematicsDiffDriveCostFunctor(double weight_nh, double weight_forward)
        : sqrt_w_nh_(std::sqrt(weight_nh)),
          sqrt_w_fwd_(std::sqrt(weight_forward)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const t1,
                    const T* const x2, const T* const y2, const T* const t2,
                    T* residual) const {
        const T dx = *x2 - *x1;
        const T dy = *y2 - *y1;
        const T c1 = ceres::cos(*t1);
        const T c2 = ceres::cos(*t2);
        const T s1 = ceres::sin(*t1);
        const T s2 = ceres::sin(*t2);
        residual[0] =
            T(sqrt_w_nh_) * ceres::abs((c1 + c2) * dy - (s1 + s2) * dx);
        const T dot = dx * c1 + dy * s1;
        residual[1] = T(sqrt_w_fwd_) *
                      ceres::abs((dot)-ceres::fmax((dot), (T(0.0)) + (T(0.0))));
        return true;
    }

    double sqrt_w_nh_;
    double sqrt_w_fwd_;
};

}  // namespace

ceres::CostFunction* CreateKinematicsDiffDriveCostFunction(
    double weight_nh, double weight_forward) {
    return new ceres::AutoDiffCostFunction<KinematicsDiffDriveCostFunctor, 2, 1,
                                           1, 1, 1, 1, 1>(
        new KinematicsDiffDriveCostFunctor(weight_nh, weight_forward));
}

namespace {

struct KinematicsCarlikeCostFunctor {
    explicit KinematicsCarlikeCostFunctor(const TimedElasticBandConfig& config,
                                          double weight_nh,
                                          double weight_radius)
        : config_(config),
          sqrt_w_nh_(std::sqrt(weight_nh)),
          sqrt_w_radius_(std::sqrt(weight_radius)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const t1,
                    const T* const x2, const T* const y2, const T* const t2,
                    T* residual) const {
        const T dx = *x2 - *x1;
        const T dy = *y2 - *y1;
        const T c1 = ceres::cos(*t1);
        const T c2 = ceres::cos(*t2);
        const T s1 = ceres::sin(*t1);
        const T s2 = ceres::sin(*t2);
        residual[0] =
            T(sqrt_w_nh_) * ceres::abs((c1 + c2) * dy - (s1 + s2) * dx);
        const T angle_diff =
            autonomy::common::NormalizeAngleDifference(*t2 - *t1);
        const T dist = ceres::sqrt(dx * dx + dy * dy);
        if (angle_diff == T(0)) {
            residual[1] = T(0);
        } else if (config_.trajectory.exact_arc_length) {
            residual[1] =
                T(sqrt_w_radius_) *
                ceres::abs(
                    (ceres::abs(dist /
                                (T(2) * ceres::sin(angle_diff / T(2))))) -
                    ceres::fmax(
                        (ceres::abs(dist /
                                    (T(2) * ceres::sin(angle_diff / T(2))))),
                        (T(config_.robot.min_turning_radius)) + (T(0.0))));
        } else {
            residual[1] =
                T(sqrt_w_radius_) *
                ceres::abs(
                    (ceres::abs(dist / ceres::abs(angle_diff))) -
                    ceres::fmax(
                        (ceres::abs(dist / ceres::abs(angle_diff))),
                        (T(config_.robot.min_turning_radius)) + (T(0.0))));
        }
        return true;
    }

    const TimedElasticBandConfig& config_;
    double sqrt_w_nh_;
    double sqrt_w_radius_;
};

}  // namespace

ceres::CostFunction* CreateKinematicsCarlikeCostFunction(
    const TimedElasticBandConfig& config, double weight_nh, double weight_radius) {
    return new ceres::AutoDiffCostFunction<KinematicsCarlikeCostFunctor, 2, 1,
                                           1, 1, 1, 1, 1>(
        new KinematicsCarlikeCostFunctor(config, weight_nh, weight_radius));
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
