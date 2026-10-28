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

#include "common/transform/transform_interpolation_buffer.hpp"

#include <algorithm>
#include <cassert>

namespace argus {
namespace transform {

void TransformInterpolationBuffer::Push(const double time_sec,
                                        const Rigid3d& transform) {
  if (!timestamped_transforms_.empty()) {
    assert(time_sec >= latest_time());
  }
  timestamped_transforms_.push_back(TimestampedTransform{time_sec, transform});
  RemoveOldTransformsIfNeeded();
}

void TransformInterpolationBuffer::SetSizeLimit(
    const std::size_t buffer_size_limit) {
  buffer_size_limit_ = buffer_size_limit;
  RemoveOldTransformsIfNeeded();
}

void TransformInterpolationBuffer::Clear() { timestamped_transforms_.clear(); }

bool TransformInterpolationBuffer::Has(const double time_sec) const {
  if (timestamped_transforms_.empty()) {
    return false;
  }
  return earliest_time() <= time_sec && time_sec <= latest_time();
}

Rigid3d TransformInterpolationBuffer::Lookup(const double time_sec) const {
  assert(Has(time_sec));
  const auto end = std::lower_bound(
      timestamped_transforms_.begin(), timestamped_transforms_.end(), time_sec,
      [](const TimestampedTransform& timestamped_transform, const double t) {
        return timestamped_transform.time_sec < t;
      });
  if (end->time_sec == time_sec) {
    return end->transform;
  }
  const auto start = std::prev(end);
  return Interpolate(*start, *end, time_sec).transform;
}

void TransformInterpolationBuffer::RemoveOldTransformsIfNeeded() {
  while (timestamped_transforms_.size() > buffer_size_limit_) {
    timestamped_transforms_.pop_front();
  }
}

double TransformInterpolationBuffer::earliest_time() const {
  assert(!empty());
  return timestamped_transforms_.front().time_sec;
}

double TransformInterpolationBuffer::latest_time() const {
  assert(!empty());
  return timestamped_transforms_.back().time_sec;
}

bool TransformInterpolationBuffer::empty() const {
  return timestamped_transforms_.empty();
}

std::size_t TransformInterpolationBuffer::size_limit() const {
  return buffer_size_limit_;
}

std::size_t TransformInterpolationBuffer::size() const {
  return timestamped_transforms_.size();
}

}  // namespace transform
}  // namespace argus
