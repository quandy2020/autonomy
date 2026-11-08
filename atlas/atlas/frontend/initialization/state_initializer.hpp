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
 * @file state_initializer.hpp
 * @brief Lidar-inertial initialization.
 *
 * `TryInitialize` is the static case: mean specific force aligns with world +Z.
 * `TryInitializeMotion` solves initial velocity and gravity from several poses
 * and the IMU samples between them, then rotates the window so gravity points
 * along world −Z. The voxel bundle adjustment inside Voxel-SLAM `motion_init`
 * stays with the sliding-window optimizer.
 */

#ifndef ATLAS_FRONTEND_INITIALIZATION_STATE_INITIALIZER_HPP_
#define ATLAS_FRONTEND_INITIALIZATION_STATE_INITIALIZER_HPP_

#include <vector>

#include "frontend/frontend_state.hpp"
#include "frontend/initialization/initialization_options.hpp"
#include "sensor/imu_data.hpp"

namespace atlas {
namespace frontend {

struct MotionFrame {
  double time = 0.0;
  transform::Rigid3d pose = transform::Rigid3d::Identity();
  std::vector<sensor::ImuData> imu_until_next;
};

class StateInitializer {
 public:
  StateInitializer() = default;
  explicit StateInitializer(InitializationOptions options)
      : options_(options) {}

  bool TryInitialize(const std::vector<sensor::ImuData>& imu,
                     FrontendState* state) const;

  bool TryInitializeMotion(const std::vector<MotionFrame>& frames,
                           std::vector<FrontendState>* states) const;

 private:
  InitializationOptions options_;
};

}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_INITIALIZATION_STATE_INITIALIZER_HPP_
