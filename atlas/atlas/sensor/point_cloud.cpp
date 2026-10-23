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

#include "sensor/point_cloud.hpp"

#include <utility>

namespace atlas {
namespace sensor {

PointCloud::PointCloud(std::vector<PointType> points)
    : points_(std::move(points)) {}

PointCloud::PointCloud(std::vector<PointType> points,
                       std::vector<float> intensities)
    : points_(std::move(points)), intensities_(std::move(intensities)) {
  if (!intensities_.empty() && intensities_.size() != points_.size()) {
    throw std::invalid_argument(
        "PointCloud intensities must be empty or match the point count");
  }
}

void PointCloud::push_back(PointType value) {
  points_.push_back(std::move(value));
}

PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform) {
  std::vector<RangefinderPoint> points;
  points.reserve(point_cloud.size());
  for (const RangefinderPoint& point : point_cloud.points()) {
    points.push_back(transform * point);
  }
  return PointCloud(points, point_cloud.intensities());
}

TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform) {
  TimedPointCloud result;
  result.reserve(point_cloud.size());
  for (const TimedRangefinderPoint& point : point_cloud) {
    result.push_back(transform * point);
  }
  return result;
}

PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z) {
  return point_cloud.copy_if([min_z, max_z](const RangefinderPoint& point) {
    return min_z <= point.position.z() && point.position.z() <= max_z;
  });
}

}  // namespace sensor
}  // namespace atlas
