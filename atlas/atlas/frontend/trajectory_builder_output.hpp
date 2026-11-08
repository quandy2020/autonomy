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
 * @file trajectory_builder_output.hpp
 * @brief Result and status returned by one trajectory-building cycle.
 */

#ifndef ATLAS_FRONTEND_TRAJECTORY_BUILDER_OUTPUT_HPP_
#define ATLAS_FRONTEND_TRAJECTORY_BUILDER_OUTPUT_HPP_

#include "sensor/imu_data.hpp"
#include "sensor/point_types.hpp"
#include "sensor/states.hpp"

namespace atlas {
namespace odometry {

/**
 * @brief Outcome of one trajectory-building cycle.
 * @c Process may still return true when status is a soft skip.
 */
enum class ProcessStatus {
  /** @brief Normal update completed. */
  kOk = 0,
  /** @brief Sync not ready; @ref Process returns false. */
  kNoPackage,
  /** @brief Lidar package empty after undistortion. */
  kEmptyLidar,
  /** @brief Wait / unknown flag; IMU propagate only. */
  kPropagateOnly,
};

/**
 * @brief Stable string label for logging and metrics.
 * @param[in] status Status value to stringify.
 * @return Lowercase snake_case label, or `"unknown"` for out-of-range values.
 */
inline const char* ProcessStatusName(ProcessStatus status) {
  switch (status) {
    case ProcessStatus::kOk:
      return "ok";
    case ProcessStatus::kNoPackage:
      return "no_package";
    case ProcessStatus::kEmptyLidar:
      return "empty_lidar";
    case ProcessStatus::kPropagateOnly:
      return "propagate_only";
    default:
      return "unknown";
  }
}

/**
 * @brief One odometry cycle output (state, clouds, observability status).
 * @var state             Filter state after this cycle.
 * @var ekf_phase         Internal EKF phase flag for the front-end.
 * @var timestamp         Output stamp in seconds.
 * @var undistorted       Motion-compensated scan (may be null).
 * @var downsampled_body  Body-frame downsample for mapping (may be null).
 * @var map_updated       True when the voxel / visual map changed.
 * @var status            Outcome code for host observability.
 * @var soft_skip         True when status != kOk but the package was consumed.
 */
struct OdometryOutput {
  sensor::StatesGroup state;
  sensor::EkfState ekf_phase = sensor::kWait;
  double timestamp = 0.0;
  sensor::PointCloudXYZI::Ptr undistorted;
  sensor::PointCloudXYZI::Ptr downsampled_body;
  bool map_updated = false;
  ProcessStatus status = ProcessStatus::kOk;
  bool soft_skip = false;
};

}  // namespace odometry
}  // namespace atlas

#endif  // ATLAS_FRONTEND_TRAJECTORY_BUILDER_OUTPUT_HPP_
