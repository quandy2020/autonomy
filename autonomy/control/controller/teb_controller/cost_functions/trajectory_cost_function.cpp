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

#include "autonomy/control/controller/teb_controller/cost_functions/trajectory_cost_function.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

namespace {

struct TimeOptimalCostFunctor {
    explicit TimeOptimalCostFunctor(double weight)
        : sqrt_weight_(std::sqrt(weight)) {}

    template <typename T>
    bool operator()(const T* const dt, T* residual) const {
        residual[0] = T(sqrt_weight_) * (*dt);
        return true;
    }

    double sqrt_weight_;
};

}  // namespace

ceres::CostFunction* CreateTimeOptimalCostFunction(double weight) {
    return new ceres::AutoDiffCostFunction<TimeOptimalCostFunctor, 1, 1>(
        new TimeOptimalCostFunctor(weight));
}

namespace {

struct ShortestPathCostFunctor {
    explicit ShortestPathCostFunctor(double weight)
        : sqrt_weight_(std::sqrt(weight)) {}

    template <typename T>
    bool operator()(const T* const x1, const T* const y1, const T* const x2,
                    const T* const y2, T* residual) const {
        residual[0] = T(sqrt_weight_) * ceres::sqrt(ceres::pow(*x2 - *x1, 2) +
                                                    ceres::pow(*y2 - *y1, 2));
        return true;
    }

    double sqrt_weight_;
};

}  // namespace

ceres::CostFunction* CreateShortestPathCostFunction(double weight) {
    return new ceres::AutoDiffCostFunction<ShortestPathCostFunctor, 1, 1, 1, 1,
                                           1>(
        new ShortestPathCostFunctor(weight));
}

namespace {

struct ViaPointCostFunctor {
    ViaPointCostFunctor(double via_point_x, double via_point_y, double weight)
        : via_point_x_(via_point_x),
          via_point_y_(via_point_y),
          sqrt_weight_(std::sqrt(weight)) {}

    template <typename T>
    bool operator()(const T* const x, const T* const y, T* residual) const {
        residual[0] =
            T(sqrt_weight_) * ceres::sqrt(ceres::pow(*x - T(via_point_x_), 2) +
                                          ceres::pow(*y - T(via_point_y_), 2));
        return true;
    }

    double via_point_x_;
    double via_point_y_;
    double sqrt_weight_;
};

}  // namespace

ceres::CostFunction* CreateViaPointCostFunction(double via_point_x,
                                                double via_point_y,
                                                double weight) {
    return new ceres::AutoDiffCostFunction<ViaPointCostFunctor, 1, 1, 1>(
        new ViaPointCostFunctor(via_point_x, via_point_y, weight));
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
