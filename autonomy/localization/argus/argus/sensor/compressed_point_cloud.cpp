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

#include "sensor/compressed_point_cloud.hpp"

#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include "common/logging.hpp"

namespace argus {
namespace sensor {
namespace {

constexpr float kPrecision = 0.001f;
constexpr int kBitsPerCoordinate = 10;
constexpr int kCoordinateMask = (1 << kBitsPerCoordinate) - 1;
constexpr int kMaxBitsPerDirection = 23;

struct BlockKey {
  int x = 0;
  int y = 0;
  int z = 0;
  bool operator<(const BlockKey& other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

}  // namespace

CompressedPointCloud::ConstIterator::ConstIterator(
    const CompressedPointCloud* compressed_point_cloud)
    : compressed_point_cloud_(compressed_point_cloud),
      remaining_points_(compressed_point_cloud->num_points_),
      remaining_points_in_current_block_(0),
      input_(compressed_point_cloud->point_data_.begin()) {
  if (remaining_points_ > 0) {
    ReadNextPoint();
  }
}

CompressedPointCloud::ConstIterator
CompressedPointCloud::ConstIterator::EndIterator(
    const CompressedPointCloud* compressed_point_cloud) {
  ConstIterator end_iterator(compressed_point_cloud);
  end_iterator.remaining_points_ = 0;
  return end_iterator;
}

RangefinderPoint CompressedPointCloud::ConstIterator::operator*() const {
  ARGUS_CHECK_GT(remaining_points_, 0u);
  return {current_point_};
}

CompressedPointCloud::ConstIterator&
CompressedPointCloud::ConstIterator::operator++() {
  --remaining_points_;
  if (remaining_points_ > 0) {
    ReadNextPoint();
  }
  return *this;
}

bool CompressedPointCloud::ConstIterator::operator!=(
    const ConstIterator& other) const {
  return remaining_points_ != other.remaining_points_;
}

void CompressedPointCloud::ConstIterator::ReadNextPoint() {
  if (remaining_points_in_current_block_ == 0) {
    remaining_points_in_current_block_ = *input_++;
    for (int i = 0; i < 3; ++i) {
      current_block_coordinates_[i] = *input_++ << kBitsPerCoordinate;
    }
  }
  --remaining_points_in_current_block_;
  const int point = *input_++;
  current_point_[0] =
      (current_block_coordinates_[0] + (point & kCoordinateMask)) * kPrecision;
  current_point_[1] = (current_block_coordinates_[1] +
                       ((point >> kBitsPerCoordinate) & kCoordinateMask)) *
                      kPrecision;
  current_point_[2] =
      (current_block_coordinates_[2] + (point >> (2 * kBitsPerCoordinate))) *
      kPrecision;
}

CompressedPointCloud::CompressedPointCloud(const PointCloud& point_cloud)
    : num_points_(point_cloud.size()) {
  struct RasterPoint {
    Eigen::Array3i point;
    int index = 0;
  };
  std::map<BlockKey, std::vector<RasterPoint>> blocks;
  ARGUS_CHECK_LE(point_cloud.size(),
                 static_cast<size_t>(std::numeric_limits<int>::max()));
  for (int point_index = 0; point_index < static_cast<int>(point_cloud.size());
       ++point_index) {
    const RangefinderPoint& point = point_cloud[point_index];
    ARGUS_CHECK_LT(point.position.cwiseAbs().maxCoeff() / kPrecision,
                   static_cast<float>(1 << kMaxBitsPerDirection));
    Eigen::Array3i raster_point;
    BlockKey block;
    int block_coordinate[3];
    for (int i = 0; i < 3; ++i) {
      raster_point[i] =
          static_cast<int>(std::lround(point.position[i] / kPrecision));
      block_coordinate[i] = raster_point[i] >> kBitsPerCoordinate;
      raster_point[i] &= kCoordinateMask;
    }
    block = {block_coordinate[0], block_coordinate[1], block_coordinate[2]};
    blocks[block].push_back({raster_point, point_index});
  }
  point_data_.reserve(4 * blocks.size() + point_cloud.size());
  for (const auto& entry : blocks) {
    point_data_.push_back(static_cast<int32_t>(entry.second.size()));
    point_data_.push_back(entry.first.x);
    point_data_.push_back(entry.first.y);
    point_data_.push_back(entry.first.z);
    for (const RasterPoint& raster_point : entry.second) {
      point_data_.push_back((((raster_point.point.z() << kBitsPerCoordinate) +
                              raster_point.point.y())
                             << kBitsPerCoordinate) +
                            raster_point.point.x());
    }
  }
}

CompressedPointCloud::CompressedPointCloud(size_t num_points,
                                           std::vector<int32_t> point_data)
    : point_data_(std::move(point_data)), num_points_(num_points) {}

bool CompressedPointCloud::empty() const { return num_points_ == 0; }
size_t CompressedPointCloud::size() const { return num_points_; }

CompressedPointCloud::ConstIterator CompressedPointCloud::begin() const {
  return ConstIterator(this);
}
CompressedPointCloud::ConstIterator CompressedPointCloud::end() const {
  return ConstIterator::EndIterator(this);
}

PointCloud CompressedPointCloud::Decompress() const {
  PointCloud decompressed;
  for (const RangefinderPoint& point : *this) {
    decompressed.push_back(point);
  }
  return decompressed;
}

bool CompressedPointCloud::operator==(const CompressedPointCloud& other) const {
  return point_data_ == other.point_data_ && num_points_ == other.num_points_;
}

}  // namespace sensor
}  // namespace argus
