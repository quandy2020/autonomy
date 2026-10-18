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
 * @file timestamped_transform.hpp
 * @brief Time-stamped Rigid3d + SLERP interpolation (Cartographer port).
 *
 * Time is double seconds (Argus convention); no proto.
 */

#ifndef ARGUS_COMMON_MATH_TRANSFORM_TIMESTAMPED_TRANSFORM_HPP_
#define ARGUS_COMMON_MATH_TRANSFORM_TIMESTAMPED_TRANSFORM_HPP_

#include "argus/common/math/transform/rigid_transform.hpp"

namespace argus {
namespace transform {

/** @brief Rigid pose with associated time (seconds). */
struct TimestampedTransform {
    double time_sec = 0.0; /**< @brief Timestamp in seconds. */
    Rigid3d transform = Rigid3d::Identity(); /**< @brief Pose at @c time_sec. */
};

/**
 * @brief Interpolate pose between @p start and @p end at @p time_sec.
 * @param start Earlier sample (translation linear, rotation SLERP).
 * @param end Later sample.
 * @param time_sec Query time; should satisfy start.time_sec ≤ time_sec ≤
 * end.time_sec.
 * @return Interpolated timestamped transform at @p time_sec.
 */
TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 double time_sec);

}  // namespace transform
}  // namespace argus

#endif  // ARGUS_COMMON_MATH_TRANSFORM_TIMESTAMPED_TRANSFORM_HPP_
