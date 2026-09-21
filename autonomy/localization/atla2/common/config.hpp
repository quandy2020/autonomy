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

#pragma once

#include <string>

#include "autonomy/localization/atla2/common/types.hpp"

namespace autonomy::localization::atla2 {

struct Atla2Config {
  FrontendMode mode = FrontendMode::kVio;
  FusionStyle fusion = FusionStyle::kLoose;
  BackendType backend = BackendType::kIekf;

  bool vo_enabled = false;
  bool vio_enabled = true;
  bool lio_enabled = false;
  bool gps_enabled = false;

  double imu_acc_noise = 0.1;
  double imu_gyr_noise = 0.01;
  double imu_acc_bias_rw = 0.001;
  double imu_gyr_bias_rw = 0.0001;

  double sync_tol_ms = 20.0;
  int max_local_map_points = 50000;
  double voxel_size = 0.2;

  // Ceres / sliding-window graph
  int window_size = 10;
  int ceres_max_iterations = 15;
  double ceres_pose_weight = 10.0;
  double ceres_imu_weight = 1.0;
  double ceres_visual_weight = 1.0;
  double cam_fx = 320.0;
  double cam_fy = 320.0;
  double cam_cx = 320.0;
  double cam_cy = 240.0;

  std::string platform_name = "default";
  std::string config_path;

  //! When true, SlamSystem may recreate frontend from HealthReport.recommended_mode.
  bool enable_mode_hot_switch = true;
};

//! Load platform YAML (config/platforms/*.yaml). Returns false on parse error.
bool LoadConfig(const std::string& path, Atla2Config* out);

}  // namespace autonomy::localization::atla2
