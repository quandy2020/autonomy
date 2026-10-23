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
 * @brief Time-stamped @ref Rigid3d and SLERP interpolation (Cartographer port).
 * Timestamps use double seconds (Argus convention); no protobuf dependency.
 */

#ifndef ATLAS_COMMON_TIMESTAMPED_TRANSFORM_HPP_
#define ATLAS_COMMON_TIMESTAMPED_TRANSFORM_HPP_

#include "common/transform/rigid_transform.hpp"

namespace atlas {
namespace transform {

/**
 * @brief Rigid pose paired with a scalar timestamp.
 * @var time_sec  Time in seconds (caller-defined epoch, often Unix or bag
 *                time).
 * @var transform Pose valid at @c time_sec.
 */
struct TimestampedTransform {
  double time_sec = 0.0;
  Rigid3d transform = Rigid3d::Identity();
};

/**
 * @brief Interpolate pose between two timestamped samples.
 * Translation is linearly interpolated; rotation uses quaternion SLERP.
 * @param[in] start    Earlier sample (`start.time_sec` ≤ query time).
 * @param[in] end      Later sample (`end.time_sec` ≥ query time).
 * @param[in] time_sec Query time in seconds; should satisfy
 *                     `start.time_sec ≤ time_sec ≤ end.time_sec`.
 * @return @ref TimestampedTransform whose `time_sec` equals @p time_sec and
 *         whose `transform` is the interpolated pose. If `end.time_sec ==
 *         start.time_sec`, the interpolation factor is treated as zero.
 */
TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 double time_sec);

}  // namespace transform
}  // namespace atlas

#endif  // ATLAS_COMMON_TIMESTAMPED_TRANSFORM_HPP_
