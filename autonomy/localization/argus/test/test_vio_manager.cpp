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
 * @file test_vio_manager.cpp
 * @brief Visual ESIKF smoke: InitializeVIO + ProcessFrame (no bag / automsgs).
 */

#include <iostream>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include "argus/common/camera/pinhole_camera.hpp"
#include "argus/frontend/types.hpp"
#include "argus/map/voxel_map.hpp"
#include "argus/odometry/vio_manager.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  using argus::camera::PinholeCamera;
  using argus::frontend::PointWithVar;
  using argus::frontend::StatesGroup;
  using argus::map::VoxelLocation;
  using argus::map::VoxelOctoTree;
  using argus::odometry::VIOManager;

  // Large enough that border=(patch_half+1)*2^pyramid fits in-frame.
  constexpr int kW = 320;
  constexpr int kH = 240;
  PinholeCamera cam(kW, kH, 1.0, 160.0, 160.0, 160.0, 120.0);

  StatesGroup state;
  StatesGroup state_prop;
  state.rot_end = Eigen::Matrix3d::Identity();
  state.pos_end = Eigen::Vector3d::Zero();
  state_prop = state;

  VIOManager vio;
  vio.cam = &cam;
  vio.grid_size = 40;
  vio.grid_n_width = 8;
  vio.grid_n_height = 6;
  vio.patch_size = 8;
  vio.patch_pyramid_level = 1;  // border=10; fits 320x240
  vio.max_iterations = 3;
  vio.outlier_threshold = 1000.0;
  vio.image_point_cov = 100.0;
  vio.enable_normal = true;
  vio.enable_exposure_estimate = false;
  vio.state = &state;
  vio.state_propagated = &state_prop;
  vio.SetImuToLidarExtrinsic(Eigen::Vector3d::Zero(),
                             Eigen::Matrix3d::Identity());
  std::vector<double> Rcl = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::vector<double> Pcl = {0, 0, 0};
  vio.SetLidarToCameraExtrinsic(Rcl, Pcl);
  vio.InitializeVIO();

  if (vio.width != kW || vio.height != kH) {
    return Fail("InitializeVIO size");
  }

  cv::Mat img(kH, kW, CV_8UC1);
  for (int y = 0; y < kH; ++y) {
    for (int x = 0; x < kW; ++x) {
      img.at<uint8_t>(y, x) =
          static_cast<uint8_t>((((x / 16) + (y / 16)) % 2) ? 220 : 40);
    }
  }

  // GenerateVisualMapPoints requires pg.size() > 10.
  std::vector<PointWithVar> pg;
  pg.reserve(24);
  for (int i = 0; i < 24; ++i) {
    PointWithVar p;
    const double u = (i % 6) - 2.5;
    const double v = (i / 6) - 1.5;
    p.point_w = Eigen::Vector3d(0.15 * u, 0.15 * v, 2.0);
    p.point_b = p.point_w;
    p.normal = Eigen::Vector3d(0, 0, -1);
    p.var = Eigen::Matrix3d::Identity() * 0.01;
    pg.push_back(p);
  }

  std::unordered_map<VoxelLocation, VoxelOctoTree*> plane_map;
  vio.ProcessFrame(img, pg, plane_map, 0.0);

  if (vio.frame_count < 1) {
    return Fail("ProcessFrame did not advance frame_count");
  }
  // First frame seeds sparse visual map from LiDAR projections.
  if (vio.visual_map.LandmarkCount() == 0) {
    return Fail("expected visual landmarks after GenerateVisualMapPoints");
  }

  std::cout << "OK vio_manager (frames=" << vio.frame_count
            << " landmarks=" << vio.visual_map.LandmarkCount() << ")\n";
  return 0;
}
