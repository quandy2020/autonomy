/*
 * Copyright 2026 The Openbot Authors
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

/**
 * @file pose_prior_cost_function.hpp
 * @brief Unary SE3 prior. Used for a loose visual pose on a lidar state.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_POSE_PRIOR_COST_FUNCTION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_POSE_PRIOR_COST_FUNCTION_HPP_

#include "Eigen/Geometry"
#include "ceres/ceres.h"

#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {

/**
 * @brief \(\mathrm{Log}(T_{\mathrm{meas}}^{-1} T) \in \mathbb{R}^6\).
 *
 * Parameter block: 6-D `[angle_axis(3), translation(3)]`.
 */
class PosePriorCostFunctor {
public:
    PosePriorCostFunctor(const SE3& measured, double sqrt_info)
        : measured_(measured), sqrt_info_(sqrt_info) {}

    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        using Vector3 = Eigen::Matrix<T, 3, 1>;
        using Matrix3 = Eigen::Matrix<T, 3, 3>;

        const Vector3 aa = Eigen::Map<const Vector3>(pose);
        const Vector3 translation = Eigen::Map<const Vector3>(pose + 3);
        const T angle = aa.norm();
        Matrix3 rotation = Matrix3::Identity();
        if (angle >= T(1e-12)) {
            rotation = Eigen::AngleAxis<T>(angle, aa / angle).toRotationMatrix();
        }

        const Matrix3 R_meas = measured_.rotation().cast<T>();
        const Vector3 t_meas = measured_.translation().cast<T>();
        const Matrix3 R_err = R_meas.transpose() * rotation;
        const Vector3 t_err = R_meas.transpose() * (translation - t_meas);

        Eigen::AngleAxis<T> aa_err(R_err);
        Vector3 aa_vec = aa_err.axis() * aa_err.angle();
        Eigen::Map<Vector3> r_rot(residuals);
        Eigen::Map<Vector3> r_trans(residuals + 3);
        r_rot = T(sqrt_info_) * aa_vec;
        r_trans = T(sqrt_info_) * t_err;
        return true;
    }

    static ceres::CostFunction* Create(const SE3& measured, double sqrt_info) {
        return new ceres::AutoDiffCostFunction<PosePriorCostFunctor, 6, 6>(
            new PosePriorCostFunctor(measured, sqrt_info));
    }

private:
    SE3 measured_;
    double sqrt_info_;
};

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_POSE_PRIOR_COST_FUNCTION_HPP_
