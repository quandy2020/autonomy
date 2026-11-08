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
 * @file local_mapping.hpp
 * @brief Sliding-window eigenvalue BA for the lidar frontend.
 *
 * The oldest pose is the gauge. Each iteration bins the window into voxels and
 * keeps a voxel when its smallest covariance eigenvalue marks a plane. Free
 * poses are then aligned to that plane. IMU samples since the previous scan
 * add a preintegrated pose residual. A pose that slides out produces one
 * solver-independent relative-pose constraint for the backend.
 */

#ifndef ATLAS_FRONTEND_LOCAL_MAPPING_LOCAL_MAPPING_HPP_
#define ATLAS_FRONTEND_LOCAL_MAPPING_LOCAL_MAPPING_HPP_

#include <cstddef>
#include <deque>
#include <vector>

#include "common/keyframe.hpp"
#include "common/relative_pose_constraint.hpp"
#include "frontend/frontend_state.hpp"
#include "frontend/odometry/plane_map.hpp"
#include "sensor/imu_data.hpp"
#include "sensor/point_cloud.hpp"

namespace atlas {
namespace frontend {
namespace local_mapping {

using ::atlas::frontend::odometry::PlaneMap;

struct WindowScan {
  common::KeyframeId id = common::kInvalidKeyframeId;
  FrontendState state;
  sensor::TimedPointCloud cloud;
  std::vector<sensor::ImuData> imu_from_previous;
};

class LocalMapping {
 public:
  struct Config {
    std::size_t window_size = 10;
    int max_iterations = 4;
    int min_points = 5;
    double voxel_size = 1.0;
    double min_eigenvalue = 0.01;
    double plane_ratio = 1.0;
    double imu_weight = 1e-4;
  };

  LocalMapping() = default;
  explicit LocalMapping(Config config) : config_(config) {}

  void Add(const FrontendState& state);
  std::size_t size() const { return window_.size(); }
  const std::deque<FrontendState>& window() const { return window_; }

  std::vector<common::RelativePoseConstraint> Push(
      common::KeyframeId id, const FrontendState& state,
      const sensor::TimedPointCloud& cloud, const PlaneMap& map,
      const std::vector<sensor::ImuData>& imu_from_previous = {});

  const std::deque<WindowScan>& scans() const { return scans_; }

 private:
  void Optimize(const PlaneMap& map);

  Config config_;
  std::deque<FrontendState> window_;
  std::deque<WindowScan> scans_;
};

}  // namespace local_mapping
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_LOCAL_MAPPING_LOCAL_MAPPING_HPP_
