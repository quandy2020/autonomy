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

#include "sensor/internal/voxel_filter.hpp"

#include <cmath>
#include <cstdint>
#include <random>
#include <unordered_map>
#include <utility>

namespace argus {
namespace sensor {
namespace {

using VoxelKeyType = uint64_t;

VoxelKeyType GetVoxelCellIndex(const Eigen::Vector3f& point, float resolution) {
  const Eigen::Array3f index = point.array() / resolution;
  const uint64_t x = static_cast<uint64_t>(std::lround(index.x()));
  const uint64_t y = static_cast<uint64_t>(std::lround(index.y()));
  const uint64_t z = static_cast<uint64_t>(std::lround(index.z()));
  return (x << 42) + (y << 21) + z;
}

template <class T, class PointFunction>
std::vector<bool> RandomizedVoxelFilterIndices(
    const std::vector<T>& point_cloud, float resolution,
    PointFunction&& point_function) {
  std::minstd_rand0 generator;
  std::unordered_map<VoxelKeyType, std::pair<int, int>>
      voxel_count_and_point_index;
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    auto& voxel = voxel_count_and_point_index[GetVoxelCellIndex(
        point_function(point_cloud[i]), resolution)];
    voxel.first++;
    if (voxel.first == 1) {
      voxel.second = static_cast<int>(i);
    } else {
      std::uniform_int_distribution<> distribution(1, voxel.first);
      if (distribution(generator) == voxel.first) {
        voxel.second = static_cast<int>(i);
      }
    }
  }
  std::vector<bool> points_used(point_cloud.size(), false);
  for (const auto& voxel_and_index : voxel_count_and_point_index) {
    points_used[static_cast<size_t>(voxel_and_index.second.second)] = true;
  }
  return points_used;
}

template <class T, class PointFunction>
std::vector<T> RandomizedVoxelFilter(const std::vector<T>& point_cloud,
                                     float resolution,
                                     PointFunction&& point_function) {
  const std::vector<bool> points_used =
      RandomizedVoxelFilterIndices(point_cloud, resolution, point_function);
  std::vector<T> results;
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    if (points_used[i]) {
      results.push_back(point_cloud[i]);
    }
  }
  return results;
}

PointCloud FilterByMaxRange(const PointCloud& point_cloud, float max_range) {
  return point_cloud.copy_if([max_range](const RangefinderPoint& point) {
    return point.position.norm() <= max_range;
  });
}

PointCloud AdaptivelyVoxelFiltered(const AdaptiveVoxelFilterOptions& options,
                                   const PointCloud& point_cloud) {
  if (static_cast<float>(point_cloud.size()) <= options.min_num_points) {
    return point_cloud;
  }
  PointCloud result = VoxelFilter(point_cloud, options.max_length);
  if (static_cast<float>(result.size()) >= options.min_num_points) {
    return result;
  }
  for (float high_length = options.max_length;
       high_length > 1e-2f * options.max_length; high_length /= 2.f) {
    float low_length = high_length / 2.f;
    result = VoxelFilter(point_cloud, low_length);
    if (static_cast<float>(result.size()) >= options.min_num_points) {
      while ((high_length - low_length) / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloud candidate = VoxelFilter(point_cloud, mid_length);
        if (static_cast<float>(candidate.size()) >= options.min_num_points) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

}  // namespace

std::vector<RangefinderPoint> VoxelFilter(
    const std::vector<RangefinderPoint>& points, float resolution) {
  return RandomizedVoxelFilter(
      points, resolution,
      [](const RangefinderPoint& point) { return point.position; });
}

PointCloud VoxelFilter(const PointCloud& point_cloud, float resolution) {
  const std::vector<bool> points_used = RandomizedVoxelFilterIndices(
      point_cloud.points(), resolution,
      [](const RangefinderPoint& point) { return point.position; });
  std::vector<RangefinderPoint> filtered_points;
  std::vector<float> filtered_intensities;
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    if (!points_used[i]) {
      continue;
    }
    filtered_points.push_back(point_cloud[i]);
    if (!point_cloud.intensities().empty()) {
      filtered_intensities.push_back(point_cloud.intensities()[i]);
    }
  }
  return PointCloud(std::move(filtered_points),
                    std::move(filtered_intensities));
}

TimedPointCloud VoxelFilter(const TimedPointCloud& timed_point_cloud,
                            float resolution) {
  return RandomizedVoxelFilter(
      timed_point_cloud, resolution,
      [](const TimedRangefinderPoint& point) { return point.position; });
}

std::vector<TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(
    const std::vector<TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements,
    float resolution) {
  return RandomizedVoxelFilter(
      range_measurements, resolution,
      [](const TimedPointCloudOriginData::RangeMeasurement& measurement) {
        return measurement.point_time.position;
      });
}

PointCloud AdaptiveVoxelFilter(const PointCloud& point_cloud,
                               const AdaptiveVoxelFilterOptions& options) {
  return AdaptivelyVoxelFiltered(
      options, FilterByMaxRange(point_cloud, options.max_range));
}

}  // namespace sensor
}  // namespace argus
