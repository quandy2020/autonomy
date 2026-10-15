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

#include <limits>

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

/**
 * @brief Nearest-obstacle latent features used by MotionPlanner.
 *
 * Only the closest obstacle is retained (full-batch dual tensors are not kept).
 */
struct DistanceFeatures {
    /** @brief Minimum latent distance, or +inf when N = 0. */
    double min_distance{std::numeric_limits<double>::infinity()};
    /** @brief Dual direction lambda of the nearest point in the world frame. */
    Eigen::Vector2d nearest_dual_direction{Eigen::Vector2d::Zero()};
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
