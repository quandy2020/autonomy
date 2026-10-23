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
 * @file slam_system.hpp
 * @brief Online lidar frontend driver and text-dataset replay.
 *
 * Dataset layout matches `app/tools/convert`: `imu.txt` plus `scans.txt`.
 */

#ifndef ATLAS_APP_SLAM_SYSTEM_HPP_
#define ATLAS_APP_SLAM_SYSTEM_HPP_

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include "common/component/component_registry.hpp"
#include "frontend/lidar/initialization/initializer.hpp"
#include "frontend/lidar/local_mapping/local_mapping.hpp"
#include "frontend/lidar/odometry/odometry.hpp"
#include "sensor/imu_data.hpp"
#include "sensor/point_types.hpp"
#include "slam_system_config.hpp"

namespace atlas {
namespace app {

/** @brief One scan listed in `scans.txt`. `path` is relative to the dataset
 * root. */
struct ScanRef {
  double time = 0.0;
  std::string path;
};

/**
 * @brief Reads the text dataset written by `rosbag_to_voxel_slam.py`.
 * IMU rows are `timestamp wx wy wz ax ay az`. Scan rows are `timestamp
 * relative_path`.
 */
class FileReplaySource {
 public:
  bool Load(const std::string& directory);

  /**
   * @brief Read one scan file into an XYZINormal cloud.
   * Rows are `x y z intensity relative_time`; `relative_time` is stored in
   * @c curvature, matching the Voxel-SLAM preprocessing contract.
   * @param[in]  index Zero-based index into @ref scans.
   * @param[out] cloud Destination cloud; cleared on success.
   * @param[out] error Optional failure reason.
   * @return False on an out-of-range index or malformed file.
   */
  bool LoadScan(std::size_t index, sensor::PointCloudXYZI* cloud,
                std::string* error) const;

  const std::string& directory() const { return directory_; }
  const std::vector<sensor::ImuData>& imu() const { return imu_; }
  const std::vector<ScanRef>& scans() const { return scans_; }

 private:
  std::string directory_;
  std::vector<sensor::ImuData> imu_;
  std::vector<ScanRef> scans_;
};

/**
 * @brief Owns the lidar initializer, IMU predict step, and local-mapping
 * window. Visual pipelines load and register components, but @ref AddScan stays
 * idle until a visual tracker consumes images.
 */
class SlamSystem {
 public:
  explicit SlamSystem(SlamSystemConfig config);

  const SlamSystemConfig& config() const { return config_; }
  bool started() const { return started_; }
  bool initialized() const { return initialized_; }

  bool Start();
  void Stop();

  /** @brief Buffer one IMU sample. Ignored until @ref Start. */
  void AddImu(const sensor::ImuData& imu);

  /**
   * @brief Fold IMU up to @p time into initialization or the predict step.
   * @return `false` when the system is stopped, @p time is not finite, or the
   *         pipeline is not `lidar`. A lidar scan that is still waiting for
   *         enough IMU returns `true` and leaves @ref initialized false.
   */
  bool AddScan(double time);

  const frontend::lidar::State& state() const { return state_; }
  const frontend::lidar::LocalMapping& local_mapping() const {
    return local_mapping_;
  }
  const std::deque<frontend::lidar::State>& trajectory() const {
    return trajectory_;
  }

  /** @brief Write the trajectory as TUM: `time tx ty tz qx qy qz qw`. */
  bool WriteTrajectory(const std::string& path) const;

  common::ComponentRegistry& registry() { return registry_; }
  const common::ComponentRegistry& registry() const { return registry_; }

 private:
  SlamSystemConfig config_;
  frontend::lidar::Initializer initializer_;
  frontend::lidar::Odometry odometry_;
  frontend::lidar::LocalMapping local_mapping_;
  common::ComponentRegistry registry_;
  std::vector<sensor::ImuData> imu_;
  std::deque<frontend::lidar::State> trajectory_;
  frontend::lidar::State state_;
  bool started_ = false;
  bool initialized_ = false;
};

/**
 * @brief Push @p source through @p system in timestamp order.
 * IMU samples at or before each scan are added first. Samples after the last
 * scan are kept in the IMU buffer and do not create another state.
 * @return Accepted scan count, or `-1` when @p system is null or not started.
 */
int ReplayDataset(SlamSystem* system, const FileReplaySource& source);

}  // namespace app
}  // namespace atlas

#endif  // ATLAS_APP_SLAM_SYSTEM_HPP_
