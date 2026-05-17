/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_COMMON_NETWORK_POSTPROCESS_INTERNAL_MATH_HPP_
#define AUTONOMY_COMMON_NETWORK_POSTPROCESS_INTERNAL_MATH_HPP_

#include "autonomy/common/network/detail/postprocess/types.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace common {
namespace network {
namespace postprocess_internal {

/**
 * @file math.hpp
 * @brief Shared detection math (sigmoid, intersection-over-union)
 */

/**
 * @brief Logistic activation for raw class/objectness logits
 * @param x Logit value
 * @return Sigmoid in (0, 1)
 */
inline float Sigmoid(float x) {
    return 1.f / (1.f + std::exp(-x));
}

/**
 * @brief Axis-aligned IoU between two detections in source coordinates
 * @param a First box
 * @param b Second box
 * @return IoU in [0, 1], or 0 when union area is negligible
 */
inline float ComputeIoU(const Detection& a, const Detection& b) {
    const float ix1 = std::max(a.x1, b.x1);
    const float iy1 = std::max(a.y1, b.y1);
    const float ix2 = std::min(a.x2, b.x2);
    const float iy2 = std::min(a.y2, b.y2);
    const float inter = std::max(0.f, ix2 - ix1) * std::max(0.f, iy2 - iy1);
    const float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    const float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    const float union_area = area_a + area_b - inter;
    return union_area > 1e-6f ? inter / union_area : 0.f;
}

}  // namespace postprocess_internal
}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_POSTPROCESS_INTERNAL_MATH_HPP_
