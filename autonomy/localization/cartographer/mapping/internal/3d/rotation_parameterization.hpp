/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_ROTATION_PARAMETERIZATION_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_ROTATION_PARAMETERIZATION_H_

#include "autonomy/localization/cartographer/common/math.hpp"
#include "ceres/jet.h"
#include "ceres/rotation.h"

namespace cartographer {
namespace mapping {

struct YawOnlyQuaternionPlus {
    template <typename T>
    bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
        return Plus(x, delta, x_plus_delta);
    }

    template <typename T>
    bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
        const T clamped_delta = common::Clamp(delta[0], T(-0.5), T(0.5));
        T q_delta[4];
        q_delta[0] = ceres::sqrt(1. - clamped_delta * clamped_delta);
        q_delta[1] = T(0.);
        q_delta[2] = T(0.);
        q_delta[3] = clamped_delta;
        ceres::QuaternionProduct(q_delta, x, x_plus_delta);
        return true;
    }

    template <typename T>
    bool Minus(const T* x, const T* y, T* x_minus_y) const {
        // For yaw-only quaternion, we compute the yaw difference
        T q_inv[4];
        q_inv[0] = x[0];
        q_inv[1] = -x[1];
        q_inv[2] = -x[2];
        q_inv[3] = -x[3];
        T q_diff[4];
        ceres::QuaternionProduct(q_inv, y, q_diff);
        // Extract yaw from quaternion difference
        x_minus_y[0] = ceres::atan2(2.0 * (q_diff[0] * q_diff[3] + q_diff[1] * q_diff[2]),
                                    1.0 - 2.0 * (q_diff[2] * q_diff[2] + q_diff[3] * q_diff[3]));
        return true;
    }
};

struct ConstantYawQuaternionPlus {
    template <typename T>
    bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
        return Plus(x, delta, x_plus_delta);
    }

    template <typename T>
    bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
        const T delta_norm = ceres::sqrt(common::Pow2(delta[0]) + common::Pow2(delta[1]));
        const T sin_delta_over_delta = delta_norm < 1e-6 ? T(1.) : ceres::sin(delta_norm) / delta_norm;
        T q_delta[4];
        q_delta[0] = delta_norm < 1e-6 ? T(1.) : ceres::cos(delta_norm);
        q_delta[1] = sin_delta_over_delta * delta[0];
        q_delta[2] = sin_delta_over_delta * delta[1];
        q_delta[3] = T(0.);
        // We apply the 'delta' which is interpreted as an angle-axis rotation
        // vector in the xy-plane of the submap frame. This way we can align to
        // gravity because rotations around the z-axis in the submap frame do
        // not change gravity alignment, while disallowing random rotations of
        // the map that have nothing to do with gravity alignment (i.e. we
        // disallow steps just changing "yaw" of the complete map).
        ceres::QuaternionProduct(x, q_delta, x_plus_delta);
        return true;
    }

    template <typename T>
    bool Minus(const T* x, const T* y, T* x_minus_y) const {
        // Compute the difference quaternion
        T q_inv[4];
        q_inv[0] = x[0];
        q_inv[1] = -x[1];
        q_inv[2] = -x[2];
        q_inv[3] = -x[3];
        T q_diff[4];
        ceres::QuaternionProduct(q_inv, y, q_diff);
        // Extract xy-plane rotation from quaternion difference
        // The rotation axis should be in the xy-plane (z-component should be 0)
        T angle = ceres::acos(common::Clamp(q_diff[0], T(-1.0), T(1.0))) * T(2.0);
        T sin_half_angle = ceres::sin(angle / T(2.0));
        if (sin_half_angle < T(1e-6)) {
            x_minus_y[0] = T(0.);
            x_minus_y[1] = T(0.);
        } else {
            x_minus_y[0] = q_diff[1] / sin_half_angle * angle;
            x_minus_y[1] = q_diff[2] / sin_half_angle * angle;
        }
        return true;
    }
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_ROTATION_PARAMETERIZATION_H_
