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
 * @file point_cloud.hpp
 * @brief Ranging point cloud. Intensities are optional and, when present,
 *        have the same length as the points.
 */

#ifndef ARGUS_SENSOR_POINT_CLOUD_HPP_
#define ARGUS_SENSOR_POINT_CLOUD_HPP_

#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

#include "common/transform/rigid_transform.hpp"
#include "sensor/rangefinder_point.hpp"

namespace argus {
namespace sensor {

class PointCloud {
 public:
  using PointType = RangefinderPoint;
  using ConstIterator = std::vector<PointType>::const_iterator;

  PointCloud() = default;
  explicit PointCloud(std::vector<PointType> points);
  PointCloud(std::vector<PointType> points, std::vector<float> intensities);

  size_t size() const { return points_.size(); }
  bool empty() const { return points_.empty(); }

  const std::vector<PointType>& points() const { return points_; }
  const std::vector<float>& intensities() const { return intensities_; }
  const PointType& operator[](size_t index) const { return points_[index]; }

  ConstIterator begin() const { return points_.begin(); }
  ConstIterator end() const { return points_.end(); }

  void push_back(PointType value);

  template <class UnaryPredicate>
  PointCloud copy_if(UnaryPredicate predicate) const {
    std::vector<PointType> points;
    std::vector<float> intensities;
    if (intensities_.empty()) {
      for (size_t index = 0; index < size(); ++index) {
        if (predicate(points_[index])) {
          points.push_back(points_[index]);
        }
      }
    } else {
      for (size_t index = 0; index < size(); ++index) {
        if (predicate(points_[index])) {
          points.push_back(points_[index]);
          intensities.push_back(intensities_[index]);
        }
      }
    }
    return PointCloud(std::move(points), std::move(intensities));
  }

 private:
  std::vector<PointType> points_;
  std::vector<float> intensities_;
};

using TimedPointCloud = std::vector<TimedRangefinderPoint>;

PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform);
PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_POINT_CLOUD_HPP_
