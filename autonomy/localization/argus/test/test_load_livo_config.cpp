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

#include <fstream>
#include <iostream>
#include <string>

#include "argus/odometry/load_livo_config.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  const std::string path = "config/livo_esikf.yaml";
  {
    std::ifstream in(path);
    if (!in) {
      return Fail("missing config/livo_esikf.yaml (run from argus root)");
    }
  }

  const auto r = argus::odometry::LoadLivoOdometryConfig(path);
  if (!r.ok) {
    std::cerr << r.error << std::endl;
    return Fail("LoadLivoOdometryConfig");
  }
  if (r.config.slam_mode != argus::frontend::LIVO) {
    return Fail("expected LIVO mode from modalities");
  }
  if (!r.config.enable_lidar || !r.config.enable_imu || !r.config.enable_image) {
    return Fail("modalities flags");
  }
  if (r.config.voxel.max_iterations_ != 3) {
    return Fail("backend max_iterations");
  }
  if (std::abs(r.config.voxel.max_voxel_size_ - 0.2) > 1e-9) {
    return Fail("map resolution");
  }
  // Degeneracy / intensity / mapping / reloc default on; gating/loop/clean off.
  if (!r.config.enable_degeneracy) {
    return Fail("degeneracy should default true");
  }
  if (!r.config.enable_intensity_assist) {
    return Fail("intensity_assist should default true");
  }
  if (!r.config.enable_mapping) {
    return Fail("mapping should default true");
  }
  if (!r.config.relocalization.enable) {
    return Fail("relocalization should default true");
  }
  if (r.config.enable_confidence_gating || r.config.loop_closure.enable ||
      r.config.map_cleaning.enable) {
    return Fail("gating/loop/cleaning should default false");
  }
  // Nested degeneracy thresholds still loaded for when enable flips on.
  if (std::abs(r.config.degeneracy.eigenvalue_threshold - 0.05) > 1e-9) {
    return Fail("nested degeneracy.yaml not applied");
  }
  if (r.config.vio.camera_calib_file.find("calibration/camera_pinhole.yaml") ==
      std::string::npos) {
    return Fail("camera/calib_file not resolved under calibration/");
  }
  if (r.config.vio.Rcl.size() != 9 || r.config.vio.Pcl.size() != 3) {
    return Fail("extrinsic lidar_camera Rcl/Pcl");
  }
  if (!r.config.ext_R.isApprox(Eigen::Matrix3d::Identity()) ||
      r.config.ext_T.norm() > 1e-12) {
    return Fail("extrinsic imu_lidar defaults");
  }
  if (r.config.vio.patch_size != 8 || r.config.vio.grid_size != 5) {
    return Fail("vio patch/grid defaults from yaml");
  }

  std::cout << "OK load_livo_config (mode=LIVO res="
            << r.config.voxel.max_voxel_size_ << ")\n";
  return 0;
}
