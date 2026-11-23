/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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
 * @file types.hpp
 * @brief Shared POD types for neupan_controller.
 */

#pragma once

#include <Eigen/Core>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

/** @brief Cached laser extrinsic and range limits (no protobuf on hot path). */
struct LaserExtrinsic {
    double x{0.0};
    double y{0.0};
    double cos_yaw{1.0};
    double sin_yaw{0.0};
    double range_min{0.05};
    double range_max{30.0};
};

using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;
using Mat2X = Eigen::Matrix2Xd;
using Mat3X = Eigen::Matrix3Xd;
using Mat22 = Eigen::Matrix2d;
using Mat33 = Eigen::Matrix3d;
using Mat32 = Eigen::Matrix<double, 3, 2>;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

/** @brief Diff: (v, omega). Omni: (speed, travel direction). */
enum class Kinematics { kDiff, kOmni };

/** @brief NRMP / adjust weights (NeuPAN paper). */
struct NrmpParams {
    int max_num{10};
    Vec3 q_s{Vec3::Ones()};
    double p_u{1.0};
    double eta{10.0};
    double d_max{1.0};
    double d_min{0.1};
    double ro_obs{400.0};
    double bk{0.1};
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
