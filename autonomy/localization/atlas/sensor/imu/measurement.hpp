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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_IMU_MEASUREMENT_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_IMU_MEASUREMENT_HPP_

#include "autonomy/localization/atlas/type.hpp"

namespace autonomy::localization::atlas {
namespace imu {

//! Single IMU sample (acceleration [m/s^2], angular velocity [rad/s]).
struct measurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double timestamp = 0.0;
    Vec3_t a = Vec3_t::Zero();
    Vec3_t w = Vec3_t::Zero();
};

}  // namespace imu
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_IMU_MEASUREMENT_HPP_
