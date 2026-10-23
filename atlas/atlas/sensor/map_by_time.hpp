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

#ifndef ATLAS_SENSOR_MAP_BY_TIME_HPP_
#define ATLAS_SENSOR_MAP_BY_TIME_HPP_

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <vector>

#include "common/logging.hpp"

namespace atlas {
namespace sensor {

// DataType must have a `double time` member. Samples on one trajectory are kept
// in increasing time order.
template <typename DataType>
class MapByTime {
 public:
  void Append(int trajectory_id, const DataType& data) {
    ATLAS_CHECK_GE(trajectory_id, 0);
    auto& trajectory = data_[trajectory_id];
    if (!trajectory.empty()) {
      ATLAS_CHECK_GT(data.time, std::prev(trajectory.end())->first);
    }
    trajectory.emplace(data.time, data);
  }

  // Drops samples strictly inside (gap_start, gap_end), keeping the samples
  // that sit on the gap edges so neighbors can still be interpolated.
  void Trim(int trajectory_id, double gap_start, double gap_end) {
    ATLAS_CHECK_LT(gap_start, gap_end);
    if (data_.count(trajectory_id) == 0) {
      return;
    }
    auto& trajectory = data_.at(trajectory_id);
    auto data_it = trajectory.lower_bound(gap_start);
    auto data_end = trajectory.upper_bound(gap_end);
    if (data_it == data_end) {
      return;
    }
    if (gap_end < std::numeric_limits<double>::max()) {
      data_end = std::prev(data_end);
      if (data_it == data_end) {
        return;
      }
    }
    if (gap_start > std::numeric_limits<double>::lowest()) {
      data_it = std::next(data_it);
    }
    while (data_it != data_end) {
      data_it = trajectory.erase(data_it);
    }
    if (trajectory.empty()) {
      data_.erase(trajectory_id);
    }
  }

  bool HasTrajectory(int trajectory_id) const {
    return data_.count(trajectory_id) != 0;
  }

  class ConstIterator {
   public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = DataType;
    using difference_type = std::int64_t;
    using pointer = const DataType*;
    using reference = const DataType&;

    explicit ConstIterator(
        typename std::map<double, DataType>::const_iterator iterator)
        : iterator_(iterator) {}

    const DataType& operator*() const { return iterator_->second; }
    const DataType* operator->() const { return &iterator_->second; }
    ConstIterator& operator++() {
      ++iterator_;
      return *this;
    }
    ConstIterator& operator--() {
      --iterator_;
      return *this;
    }
    bool operator==(const ConstIterator& other) const {
      return iterator_ == other.iterator_;
    }
    bool operator!=(const ConstIterator& other) const {
      return !(*this == other);
    }

   private:
    typename std::map<double, DataType>::const_iterator iterator_;
  };

  ConstIterator BeginOfTrajectory(int trajectory_id) const {
    return ConstIterator(data_.at(trajectory_id).begin());
  }
  ConstIterator EndOfTrajectory(int trajectory_id) const {
    return ConstIterator(data_.at(trajectory_id).end());
  }
  ConstIterator lower_bound(int trajectory_id, double time) const {
    return ConstIterator(data_.at(trajectory_id).lower_bound(time));
  }

  std::vector<int> trajectory_ids() const {
    std::vector<int> ids;
    ids.reserve(data_.size());
    for (const auto& entry : data_) {
      ids.push_back(entry.first);
    }
    return ids;
  }

 private:
  std::map<int, std::map<double, DataType>> data_;
};

}  // namespace sensor
}  // namespace atlas

#endif  // ATLAS_SENSOR_MAP_BY_TIME_HPP_
