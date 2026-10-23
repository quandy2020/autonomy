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
 * @file initializer.hpp
 * @brief Lidar-inertial initialization.
 *
 * `TryInitialize` is the static case: mean specific force aligns with world +Z.
 * `TryInitializeMotion` solves initial velocity and gravity from several poses
 * and the IMU samples between them, then rotates the window so gravity points
 * along world −Z. The voxel bundle adjustment inside Voxel-SLAM `motion_init`
 * stays with the sliding-window optimizer.
 */

#ifndef ATLAS_FRONTEND_LIDAR_INITIALIZER_HPP_
#define ATLAS_FRONTEND_LIDAR_INITIALIZER_HPP_

#include <vector>

#include "frontend/lidar/state.hpp"
#include "sensor/imu_types.hpp"

namespace atlas {
namespace frontend {
namespace lidar {

struct MotionFrame {
  double time = 0.0;
  transform::Rigid3d pose = transform::Rigid3d::Identity();
  std::vector<sensor::ImuData> imu_until_next;
};

class Initializer {
 public:
  struct Config {
    int min_imu_samples = 20;
    int min_motion_frames = 3;
    double gravity_magnitude = 9.81;
    double gravity_tolerance = 0.5;
    double max_accel_variance = 0.05;
  };

  Initializer() = default;
  explicit Initializer(Config config) : config_(config) {}

  bool TryInitialize(const std::vector<sensor::ImuData>& imu,
                     State* state) const;

  bool TryInitializeMotion(const std::vector<MotionFrame>& frames,
                           std::vector<State>* states) const;

 private:
  Config config_;
};

}  // namespace lidar
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_LIDAR_INITIALIZER_HPP_
