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

#include "voxel_slam_system.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <memory>
#include <utility>

#include "sensor/point_types.hpp"

namespace atlas {
namespace app {
namespace {

vxslam::OdometryConfig MakeVoxelConfig(const SlamSystemConfig& config) {
  vxslam::OdometryConfig voxel;
  voxel.win_size = config.voxel_win_size;
  voxel.worker_thread_count = config.voxel_thread_num;
  voxel.degrade_bound = config.voxel_degrade_bound;
  voxel.down_size = config.voxel_down_size;
  voxel.voxel_size = config.voxel_size;
  voxel.min_eigen_value = config.voxel_min_eigen_value;
  voxel.imu_coef = config.voxel_imu_coef;
  voxel.bias_gyro_random_walk = config.voxel_bias_gyro_random_walk;
  voxel.bias_accel_random_walk = config.voxel_bias_accel_random_walk;
  voxel.minimum_bundle_adjustment_points =
      config.voxel_minimum_bundle_adjustment_points;
  voxel.enable_loop = config.voxel_enable_loop;
  voxel.enable_gba = config.voxel_enable_gba;
  voxel.enable_local_ba = config.voxel_enable_local_ba;
  voxel.enable_gravity_ba = config.voxel_enable_gravity_ba;
  voxel.enable_triangle = config.voxel_enable_triangle;
  voxel.icp_eigval = config.voxel_icp_eigval;
  voxel.jud_default = config.voxel_jud_default;
  voxel.ratio_drift = config.voxel_ratio_drift;
  voxel.curr_halt = config.voxel_curr_halt;
  voxel.prev_halt = config.voxel_prev_halt;
  voxel.gba_local_win = config.voxel_gba_local_win;
  voxel.gba_merge_size = config.voxel_gba_merge_size;
  voxel.gba_voxel_size = config.voxel_gba_voxel_size;
  voxel.gba_minimum_eigenvalue = config.voxel_gba_minimum_eigenvalue;
  voxel.gba_maximum_iteration_count =
      config.voxel_gba_maximum_iteration_count;
  voxel.save_path = config.voxel_save_path;
  voxel.bagname = config.voxel_bagname;
  voxel.previous_map = config.voxel_previous_map;
  return voxel;
}

}  // namespace

VoxelSlamSystem::VoxelSlamSystem(const SlamSystemConfig& config)
    : config_(config), odometry_(MakeVoxelConfig(config)) {}

bool VoxelSlamSystem::Start() {
  if (!ValidateSlamSystemConfig(config_)) {
    return false;
  }
  odometry_.Initialize();
  started_ = true;
  return true;
}

bool VoxelSlamSystem::Stop(std::string* error) {
  const bool saved = odometry_.Finalize(error);
  started_ = false;
  return saved;
}

void VoxelSlamSystem::AddImu(const sensor::ImuData& imu) {
  if (!started_ || !std::isfinite(imu.time) ||
      !imu.angular_velocity.allFinite() ||
      !imu.linear_acceleration.allFinite()) {
    return;
  }
  odometry_.AddImu(imu);
}

bool VoxelSlamSystem::AddScan(const sensor::PointCloudXYZI& cloud,
                              double time) {
  if (!started_ || !std::isfinite(time) || cloud.empty() ||
      std::any_of(cloud.points.begin(), cloud.points.end(),
                  [](const sensor::PointType& point) {
                    return !std::isfinite(point.x) || !std::isfinite(point.y) ||
                           !std::isfinite(point.z) ||
                           !std::isfinite(point.intensity) ||
                           !std::isfinite(point.curvature);
                  })) {
    return false;
  }
  odometry_.AddLidar(std::make_shared<sensor::PointCloudXYZI>(cloud), time);
  ::atlas::odometry::OdometryOutput output;
  while (odometry_.Process(&output)) {
    if (output.status == ::atlas::odometry::ProcessStatus::kOk) {
      trajectory_.push_back(odometry_.x_curr());
    }
  }
  return true;
}

bool VoxelSlamSystem::initialized() const { return !trajectory_.empty(); }

sensor::PointCloudXYZI VoxelSlamSystem::GlobalMapSnapshot() const {
  return odometry_.GlobalMapSnapshot();
}

bool VoxelSlamSystem::WriteTrajectory(const std::string& path) const {
  if (path.empty()) {
    return false;
  }
  std::ofstream output(path);
  if (!output) {
    return false;
  }
  std::deque<vxslam::ImuState> trajectory = trajectory_;
  for (const vxslam::ImuState& corrected : odometry_.KeyframePoseSnapshot()) {
    const auto it =
        std::lower_bound(trajectory.begin(), trajectory.end(), corrected.t,
                         [](const vxslam::ImuState& state, double time) {
                           return state.t < time;
                         });
    auto nearest = it;
    if (it != trajectory.end() && it != trajectory.begin()) {
      const auto previous = std::prev(it);
      if (std::abs(previous->t - corrected.t) < std::abs(it->t - corrected.t)) {
        nearest = previous;
      }
    } else if (it == trajectory.end() && !trajectory.empty()) {
      nearest = std::prev(trajectory.end());
    }
    if (nearest != trajectory.end() &&
        std::abs(nearest->t - corrected.t) <= 1e-3) {
      *nearest = corrected;
    }
  }
  output.precision(12);
  for (const vxslam::ImuState& state : trajectory) {
    const Eigen::Quaterniond quaternion(state.R);
    output << state.t << ' ' << state.p.x() << ' ' << state.p.y() << ' '
           << state.p.z() << ' ' << quaternion.x() << ' ' << quaternion.y()
           << ' ' << quaternion.z() << ' ' << quaternion.w() << '\n';
  }
  return static_cast<bool>(output);
}

int ReplayVoxelDataset(VoxelSlamSystem* system,
                       const FileReplaySource& source) {
  if (system == nullptr || !system->started()) {
    return -1;
  }
  std::size_t imu_index = 0;
  int accepted = 0;
  for (std::size_t scan_index = 0; scan_index < source.scans().size();
       ++scan_index) {
    const ScanRef& scan = source.scans()[scan_index];
    while (imu_index < source.imu().size() &&
           source.imu()[imu_index].time <= scan.time) {
      system->AddImu(source.imu()[imu_index]);
      ++imu_index;
    }
    sensor::PointCloudXYZI cloud;
    std::string error;
    if (!source.LoadScan(scan_index, &cloud, &error)) {
      return -1;
    }
    if (!system->AddScan(cloud, scan.time)) {
      return -1;
    }
    ++accepted;
  }
  while (imu_index < source.imu().size()) {
    system->AddImu(source.imu()[imu_index]);
    ++imu_index;
  }
  return accepted;
}

}  // namespace app
}  // namespace atlas
