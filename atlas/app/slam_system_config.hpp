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
 * @file slam_system_config.hpp
 * @brief YAML configuration for the voxel SLAM process.
 */

#ifndef ATLAS_APP_SLAM_SYSTEM_CONFIG_HPP_
#define ATLAS_APP_SLAM_SYSTEM_CONFIG_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace atlas {
namespace app {

/** @brief One entry in the component registry. `type` is a builtin creator key.
 */
struct ComponentSpec {
  std::string name;
  std::string type;
};

/**
 * @brief Process configuration.
 * `pipeline` is `lidar`, `visual`, or `voxel_slam`. Initialization and window
 * fields feed the lidar frontend; the `voxel_*` fields feed the ported
 * Voxel-SLAM pipeline. `components` are registered by @ref SlamSystemBuilder.
 */
struct SlamSystemConfig {
  std::string pipeline = "lidar";
  int min_imu_samples = 20;
  double gravity_magnitude = 9.81;
  double max_accel_variance = 0.05;
  std::size_t window_size = 10;
  std::string trajectory_path;
  std::vector<ComponentSpec> components{{"lidar_frontend", "lidar_frontend"}};

  // Voxel-SLAM (pipeline == "voxel_slam").
  int voxel_win_size = 10;
  int voxel_thread_num = 4;
  int voxel_degrade_bound = 50;
  double voxel_down_size = 0.5;
  double voxel_size = 1.0;
  double voxel_min_eigen_value = 0.01;
  double voxel_imu_coef = 1e-4;
  bool voxel_points_have_timestamps = true;
  double voxel_bias_gyro_random_walk = 1e-4;
  double voxel_bias_accel_random_walk = 1e-4;
  int voxel_minimum_bundle_adjustment_points = 20;
  bool voxel_enable_loop = true;
  bool voxel_enable_gba = true;
  bool voxel_enable_local_ba = true;
  bool voxel_enable_gravity_ba = false;
  bool voxel_enable_triangle = true;
  double voxel_icp_eigval = 0.01;
  double voxel_jud_default = 0.45;
  double voxel_ratio_drift = 0.05;
  int voxel_curr_halt = 10;
  int voxel_prev_halt = 30;
  int voxel_gba_local_win = 10;
  int voxel_gba_merge_size = 5;
  double voxel_gba_voxel_size = 1.0;
  double voxel_gba_minimum_eigenvalue = 0.01;
  int voxel_gba_maximum_iteration_count = 1;
  std::string voxel_save_path;
  std::string voxel_bagname = "session0";
  std::string voxel_previous_map;
};

/** @brief Built-in defaults, including one lidar frontend component. */
SlamSystemConfig DefaultSlamSystemConfig();

/** @brief `false` when a field is out of range or a component name repeats. */
bool ValidateSlamSystemConfig(const SlamSystemConfig& config);

/**
 * @brief Read a YAML subset into @p config.
 * Unknown keys, malformed numbers, and values that fail @ref
 * ValidateSlamSystemConfig return `false`. @p config is unchanged on failure.
 */
bool LoadSlamSystemConfig(const std::string& path, SlamSystemConfig* config);

/** @brief Write the canonical nested YAML form consumed by @ref
 * LoadSlamSystemConfig. */
bool SaveSlamSystemConfig(const SlamSystemConfig& config,
                          const std::string& path);

}  // namespace app
}  // namespace atlas

#endif  // ATLAS_APP_SLAM_SYSTEM_CONFIG_HPP_
