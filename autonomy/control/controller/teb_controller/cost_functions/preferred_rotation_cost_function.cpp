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

#include "autonomy/control/controller/teb_controller/cost_functions/preferred_rotation_cost_function.hpp"
#include <algorithm>

#include "autonomy/common/math/math.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {
namespace {

struct PreferredRotationDirectionCostFunctor {
    PreferredRotationDirectionCostFunctor(double rotation_sign, double weight)
        : rotation_sign_(rotation_sign), sqrt_weight_(std::sqrt(weight)) {}

    template <typename T>
    bool operator()(const T* const t1, const T* const t2, T* residual) const {
        const T signed_yaw_change =
            T(rotation_sign_) *
            autonomy::common::NormalizeAngleDifference(*t2 - *t1);
        residual[0] =
            T(sqrt_weight_) * ceres::abs((signed_yaw_change)-ceres::fmax(
                                  (signed_yaw_change), (T(0.0)) + (T(0.0))));
        return true;
    }

    double rotation_sign_;
    double sqrt_weight_;
};

}  // namespace

ceres::CostFunction* CreatePreferredRotationDirectionCostFunction(
    double rotation_sign, double weight) {
    return new ceres::AutoDiffCostFunction<
        PreferredRotationDirectionCostFunctor, 1, 1, 1>(
        new PreferredRotationDirectionCostFunctor(rotation_sign, weight));
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
