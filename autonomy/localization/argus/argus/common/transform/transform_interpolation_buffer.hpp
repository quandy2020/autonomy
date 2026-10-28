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
 * @brief Time-ordered @ref Rigid3d buffer with interpolated pose lookup.
 */

#ifndef ARGUS_COMMON_TRANSFORM_INTERPOLATION_BUFFER_HPP_
#define ARGUS_COMMON_TRANSFORM_INTERPOLATION_BUFFER_HPP_

#include <cstddef>
#include <deque>
#include <limits>

#include "common/transform/rigid_transform.hpp"
#include "common/transform/timestamped_transform.hpp"

namespace argus {
namespace transform {

/**
 * @brief Sentinel size meaning the buffer does not drop old poses.
 */
constexpr std::size_t kUnlimitedBufferSize =
    std::numeric_limits<std::size_t>::max();

/**
 * @brief Time-ordered deque of poses with spherical-linear interpolation.
 * Timestamps are seconds. @ref Lookup requires @ref Has(@p time_sec) to be
 * true; otherwise behavior is undefined (asserted in debug builds).
 */
class TransformInterpolationBuffer {
 public:
  /**
   * @brief Construct an empty buffer with unlimited capacity.
   */
  TransformInterpolationBuffer() = default;

  /**
   * @brief Cap the number of stored poses; oldest entries are dropped.
   * @param[in] buffer_size_limit Maximum entries after each @ref Push, or
   *                              @ref kUnlimitedBufferSize for no cap.
   */
  void SetSizeLimit(std::size_t buffer_size_limit);

  /**
   * @brief Append a pose; times must be non-decreasing.
   * @param[in] time_sec  Timestamp in seconds (≥ previous latest time).
   * @param[in] transform Pose at @p time_sec.
   */
  void Push(double time_sec, const Rigid3d& transform);

  /**
   * @brief Remove all stored transforms.
   */
  void Clear();

  /**
   * @brief Whether @p time_sec lies inside the buffered time interval.
   * @param[in] time_sec Query time in seconds.
   * @return `true` if the buffer is non-empty and
   *         `earliest_time() ≤ time_sec ≤ latest_time()`; else `false`.
   */
  bool Has(double time_sec) const;

  /**
   * @brief Interpolate the pose at @p time_sec.
   * @param[in] time_sec Query time; must satisfy @ref Has(@p time_sec).
   * @return Interpolated @ref Rigid3d. Undefined if @ref Has is false.
   */
  Rigid3d Lookup(double time_sec) const;

  /**
   * @brief Timestamp of the oldest stored pose.
   * @return Earliest `time_sec`, or asserts if empty.
   */
  double earliest_time() const;

  /**
   * @brief Timestamp of the newest stored pose.
   * @return Latest `time_sec`, or asserts if empty.
   */
  double latest_time() const;

  /**
   * @brief Whether no poses are stored.
   * @return `true` if empty; `false` otherwise.
   */
  bool empty() const;

  /**
   * @brief Configured maximum number of stored poses.
   * @return Size cap; @ref kUnlimitedBufferSize means unbounded.
   */
  std::size_t size_limit() const;

  /**
   * @brief Number of poses currently stored.
   * @return Current entry count.
   */
  std::size_t size() const;

 private:
  /**
   * @brief Drop oldest entries while `size() > size_limit()`.
   */
  void RemoveOldTransformsIfNeeded();

  std::deque<TimestampedTransform> timestamped_transforms_;
  std::size_t buffer_size_limit_ = kUnlimitedBufferSize;
};

}  // namespace transform
}  // namespace argus

#endif  // ARGUS_COMMON_TRANSFORM_INTERPOLATION_BUFFER_HPP_
