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
 * @file voxel_slam_system.hpp
 * @brief Driver around the ported Voxel-SLAM LiDAR-inertial pipeline.
 */

#ifndef ARGUS_APP_VOXEL_SLAM_SYSTEM_HPP_
#define ARGUS_APP_VOXEL_SLAM_SYSTEM_HPP_

#include <deque>
#include <string>

#include "frontend/lidar/voxel_slam/voxel_slam.hpp"
#include "sensor/imu_types.hpp"
#include "sensor/point_types.hpp"
#include "slam_system.hpp"
#include "slam_system_config.hpp"

namespace argus {
namespace app {

/**
 * @brief Owns one Voxel-SLAM odometry instance and records its trajectory.
 *
 * The pipeline consumes IMU and LiDAR pushed through @ref AddImu / @ref
 * AddScan. Every processed scan appends the current state to @ref trajectory.
 */
class VoxelSlamSystem {
 public:
  explicit VoxelSlamSystem(const SlamSystemConfig& config);

  bool Start();
  bool Stop(std::string* error = nullptr);

  /** @brief Forward one IMU sample to the EKF / preintegration buffers. */
  void AddImu(const sensor::ImuData& imu);

  /**
   * @brief Push one scan and drain all completed odometry packages.
   * @return False when the system is stopped.
   */
  bool AddScan(const sensor::PointCloudXYZI& cloud, double time);

  bool started() const { return started_; }
  bool initialized() const;
  sensor::PointCloudXYZI GlobalMapSnapshot() const;

  const std::deque<vxslam::ImuState>& trajectory() const { return trajectory_; }
  vxslam::Odometry& odometry() { return odometry_; }
  const vxslam::Odometry& odometry() const { return odometry_; }

  /** @brief Write the recorded trajectory as TUM. */
  bool WriteTrajectory(const std::string& path) const;

 private:
  SlamSystemConfig config_;
  vxslam::Odometry odometry_;
  std::deque<vxslam::ImuState> trajectory_;
  bool started_ = false;
};

/**
 * @brief Replay a text dataset through @p system in timestamp order.
 * @return Accepted scan count, or `-1` on failure.
 */
int ReplayVoxelDataset(VoxelSlamSystem* system, const FileReplaySource& source);

}  // namespace app
}  // namespace argus

#endif  // ARGUS_APP_VOXEL_SLAM_SYSTEM_HPP_
