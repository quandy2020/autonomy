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
 * @file transform_interpolation_buffer.hpp
 * @brief Time-ordered Rigid3d buffer with interpolated lookup.
 */

#ifndef ARGUS_COMMON_MATH_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_HPP_
#define ARGUS_COMMON_MATH_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_HPP_

#include <cstddef>
#include <deque>
#include <limits>

#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/common/math/transform/timestamped_transform.hpp"

namespace argus {
namespace transform {

/** @brief No cap on stored transforms (@ref TransformInterpolationBuffer
 * default). */
constexpr std::size_t kUnlimitedBufferSize =
    std::numeric_limits<std::size_t>::max();

/**
 * @brief Time-ordered deque of @ref Rigid3d poses with spherical-linear
 * interpolation.
 *
 * Timestamps are seconds; @ref Lookup requires @ref Has to be true for the
 * query time.
 */
class TransformInterpolationBuffer
{
public:
    TransformInterpolationBuffer() = default;

    /**
     * @brief Cap the number of stored poses (oldest dropped after @ref Push).
     * @param buffer_size_limit Maximum entries, or @ref kUnlimitedBufferSize.
     */
    void SetSizeLimit(std::size_t buffer_size_limit);
    /**
     * @brief Append a pose; maintains monotonic non-decreasing time order.
     * @param time_sec Timestamp in seconds.
     * @param transform Pose at that time.
     */
    void Push(double time_sec, const Rigid3d& transform);
    /** @brief Remove all stored transforms. */
    void Clear();

    /**
     * @brief Whether @p time_sec lies within the buffered interval (inclusive
     * endpoints).
     * @param time_sec Query time in seconds.
     */
    bool Has(double time_sec) const;
    /**
     * @brief Interpolate pose at @p time_sec (undefined if @ref Has is false).
     * @param time_sec Query time in seconds.
     * @return Interpolated @ref Rigid3d.
     */
    Rigid3d Lookup(double time_sec) const;

    /** @return Timestamp of the oldest stored pose, or 0 if empty. */
    double earliest_time() const;
    /** @return Timestamp of the newest stored pose, or 0 if empty. */
    double latest_time() const;
    /** @return True if no poses are stored. */
    bool empty() const;
    /** @return Configured size cap (@ref kUnlimitedBufferSize means unbounded).
     */
    std::size_t size_limit() const;
    /** @return Number of stored poses. */
    std::size_t size() const;

private:
    /** @brief Drop oldest entries when @ref size exceeds @ref size_limit. */
    void RemoveOldTransformsIfNeeded();

    std::deque<TimestampedTransform> timestamped_transforms_;
    std::size_t buffer_size_limit_ = kUnlimitedBufferSize;
};

}  // namespace transform
}  // namespace argus

#endif  // ARGUS_COMMON_MATH_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_HPP_
