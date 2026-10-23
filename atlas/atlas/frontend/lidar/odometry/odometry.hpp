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
 * @file odometry.hpp
 * @brief Lidar odometry: IMU predict, then point-to-plane scan-to-map.
 *
 * Predict propagates the 15-dimensional error state (rotation, position,
 * velocity, gyro bias, accelerometer bias). Update is the iterated EKF in
 * Voxel-SLAM `lio_state_estimation`: point-to-plane on the pose, with the
 * covariance prior carrying the correction into velocity and bias.
 *
 * The map is read through `PlaneMap`. Repeated failed alignments increment a
 * counter; past `degrade_bound` the scan is marked diverged.
 */

#ifndef ATLAS_FRONTEND_LIDAR_ODOMETRY_HPP_
#define ATLAS_FRONTEND_LIDAR_ODOMETRY_HPP_

#include <vector>

#include "frontend/lidar/odometry/plane_map.hpp"
#include "frontend/lidar/state.hpp"
#include "sensor/imu_types.hpp"
#include "sensor/point_cloud.hpp"

namespace atlas {
namespace frontend {
namespace lidar {

class Odometry {
 public:
  struct Config {
    int max_iterations = 4;
    int degrade_bound = 10;
    double min_normal_eigenvalue = 14.0;
    double gyro_noise = 0.1;
    double accel_noise = 0.1;
    double gyro_bias_noise = 1e-4;
    double accel_bias_noise = 1e-4;
  };

  struct PredictResult {
    State state;
    bool propagated = false;
  };

  struct UpdateResult {
    State state;
    int match_count = 0;
    bool aligned = false;
    bool diverged = false;
  };

  Odometry() = default;
  explicit Odometry(Config config) : config_(config) {}

  PredictResult Predict(const State& state,
                        const std::vector<sensor::ImuData>& imu) const;

  UpdateResult Update(const State& predicted,
                      const sensor::TimedPointCloud& cloud,
                      const PlaneMap& map);

  void ResetDegeneracy() { degrade_count_ = 0; }
  int degrade_count() const { return degrade_count_; }

 private:
  Config config_;
  int degrade_count_ = 0;
};

}  // namespace lidar
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_LIDAR_ODOMETRY_HPP_
