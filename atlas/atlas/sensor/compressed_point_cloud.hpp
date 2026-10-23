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

#ifndef ATLAS_SENSOR_COMPRESSED_POINT_CLOUD_HPP_
#define ATLAS_SENSOR_COMPRESSED_POINT_CLOUD_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <iterator>
#include <vector>

#include "sensor/point_cloud.hpp"

namespace atlas {
namespace sensor {

// Block encoding of a point cloud at 1 mm. Each block stores its origin and
// packs every point into one int32 with 10 bits per coordinate.
class CompressedPointCloud {
 public:
  class ConstIterator;

  CompressedPointCloud() : num_points_(0) {}
  explicit CompressedPointCloud(const PointCloud& point_cloud);
  CompressedPointCloud(size_t num_points, std::vector<int32_t> point_data);

  PointCloud Decompress() const;
  bool empty() const;
  size_t size() const;
  const std::vector<int32_t>& point_data() const { return point_data_; }
  ConstIterator begin() const;
  ConstIterator end() const;
  bool operator==(const CompressedPointCloud& other) const;

 private:
  std::vector<int32_t> point_data_;
  size_t num_points_;
};

class CompressedPointCloud::ConstIterator {
 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type = RangefinderPoint;
  using difference_type = std::int64_t;
  using pointer = const RangefinderPoint*;
  using reference = const RangefinderPoint&;

  explicit ConstIterator(const CompressedPointCloud* compressed_point_cloud);
  static ConstIterator EndIterator(
      const CompressedPointCloud* compressed_point_cloud);

  RangefinderPoint operator*() const;
  ConstIterator& operator++();
  bool operator!=(const ConstIterator& other) const;

 private:
  void ReadNextPoint();

  const CompressedPointCloud* compressed_point_cloud_ = nullptr;
  size_t remaining_points_ = 0;
  int32_t remaining_points_in_current_block_ = 0;
  Eigen::Vector3f current_point_ = Eigen::Vector3f::Zero();
  Eigen::Vector3i current_block_coordinates_ = Eigen::Vector3i::Zero();
  std::vector<int32_t>::const_iterator input_;
};

}  // namespace sensor
}  // namespace atlas

#endif  // ATLAS_SENSOR_COMPRESSED_POINT_CLOUD_HPP_
