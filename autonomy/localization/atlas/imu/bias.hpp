/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_IMU_BIAS_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_IMU_BIAS_HPP_

#include "autonomy/localization/atlas/type.hpp"

namespace autonomy::localization::atlas {
namespace imu {

//! Constant accelerometer / gyroscope bias.
struct bias {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3_t acc = Vec3_t::Zero();
    Vec3_t gyro = Vec3_t::Zero();

    bias() = default;
    bias(const Vec3_t& ba, const Vec3_t& bg)
        : acc(ba), gyro(bg) {}

    Vec6_t to_vector() const {
        Vec6_t v;
        v << acc, gyro;
        return v;
    }

    static bias from_vector(const Vec6_t& v) {
        return bias(v.head<3>(), v.tail<3>());
    }
};

}  // namespace imu
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_IMU_BIAS_HPP_
