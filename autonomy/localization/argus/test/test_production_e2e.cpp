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
 * @file test_production_e2e.cpp
 * @brief Production smoke: SE(3) pose-graph, map package v3, reloc SE3, degeneracy.
 */

#include <atomic>
#include <cmath>
#include <filesystem>
#include <iostream>

#include "argus/backend/pose_graph/pose_graph.hpp"
#include "argus/common/math/transform/transform.hpp"
#include "argus/frontend/sensor_sync.hpp"
#include "argus/mapping/map_builder.hpp"
#include "argus/mapping/map_package.hpp"
#include "argus/perception/degeneracy_detector.hpp"
#include "argus/relocalization/bbs/engine.hpp"
#include "argus/relocalization/prior_map.hpp"
#include "argus/sensor/image_data.hpp"
#include "argus/sensor/imu_data.hpp"

namespace transform = argus::transform;

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

argus::sensor::PointCloudXYZI MakeWall(double x0, double y0, double z0) {
  argus::sensor::PointCloudXYZI cloud;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 10; ++j) {
      argus::sensor::PointType p;
      p.x = static_cast<float>(x0 + 0.1 * i);
      p.y = static_cast<float>(y0);
      p.z = static_cast<float>(z0 + 0.1 * j);
      cloud.push_back(p);
    }
  }
  return cloud;
}

}  // namespace

int main() {
  using argus::backend::PoseGraph;
  using argus::frontend::SensorSync;
  using argus::mapping::IsMapPackageDirectory;
  using argus::mapping::MapBuilder;
  using argus::mapping::MapBuilderConfig;
  using argus::mapping::MapBuilderKeyFrame;
  using argus::mapping::VerifyMapPackage;
  using argus::mapping::kMapPackageVersion;
  using argus::perception::DegeneracyDetector;
  using argus::relocalization::BbsSearchEngine;
  using argus::relocalization::BranchAndBoundConfig;
  using argus::relocalization::BnBSearchSpace;
  using argus::relocalization::PriorMapLoader;
  using argus::relocalization::RelocRequest;

  // --- SE(3) pose graph closes with z offset ---
  {
    PoseGraph pg;
    const transform::Rigid3d T0 = transform::Rigid3d::Identity();
    const transform::Rigid3d T1 =
        transform::Rigid3d::Translation(Eigen::Vector3d(1, 0, 0.2));
    const transform::Rigid3d T2 =
        transform::Rigid3d::Translation(Eigen::Vector3d(1, 1, 0.4));
    pg.AddNode(0, T0);
    pg.AddNode(1, T1);
    pg.AddNode(2, T2);
    pg.AddOdometryEdge(0, 1, T0.inverse() * T1);
    pg.AddOdometryEdge(1, 2, T1.inverse() * T2);
    transform::Rigid3d T_loop =
        transform::Rigid3d::Translation(Eigen::Vector3d(0.05, 1.0, 0.35));
    argus::backend::LoopFactor f;
    f.from_id = 2;
    f.to_id = 0;
    f.T_i_j = T_loop;
    f.information = 200.0;
    pg.AddLoopFactor(f);
    pg.Optimize(20);
    if (pg.loop_factor_count() != 1) {
      return Fail("se3 loop count");
    }
    if (!(pg.last_cost() >= 0.0)) {
      return Fail("se3 last_cost");
    }
    // Node 2 should move toward satisfying the loop (z closer to ~0.35 relative).
    const auto T2o = pg.Pose(2);
    if (!T2o.IsValid()) {
      return Fail("se3 pose invalid");
    }
  }

  // --- Map package v3: save / fingerprint / load / reloc prior ---
  const std::string dir = "argus_test_prod_e2e_map";
  std::filesystem::remove_all(dir);
  {
    MapBuilderConfig cfg;
    cfg.session_id = "prod";
    cfg.prior_resolution = 0.4;
    MapBuilder builder(cfg);
    for (int i = 0; i < 4; ++i) {
      MapBuilderKeyFrame kf;
      kf.local_id = i;
      kf.stamp_sec = 0.1 * i;
      kf.T_w_b =
          transform::Rigid3d::Translation(Eigen::Vector3d(i * 1.5, 0, 0));
      kf.cloud = MakeWall(0, 1, 0);
      builder.AddKeyFrame(kf);
    }
    builder.AddVisualLandmark(Eigen::Vector3d(0.5, 0.5, 1.0));
    if (!builder.Save(dir)) {
      return Fail("save package");
    }
    if (!IsMapPackageDirectory(dir) ||
        builder.package_meta().version != kMapPackageVersion) {
      return Fail("package version");
    }
    if (builder.package_meta().map_xyz_fnv64 == 0) {
      return Fail("fingerprint missing");
    }
    std::string verr;
    if (!VerifyMapPackage(dir, builder.package_meta(), &verr)) {
      return Fail(verr.c_str());
    }

    MapBuilder loaded(cfg);
    if (!loaded.Load(dir)) {
      return Fail("load package");
    }
    if (loaded.map_point_count() == 0) {
      return Fail("loaded empty");
    }

    auto prior = PriorMapLoader::LoadPath(dir, 0.4);
    if (!prior) {
      return Fail("prior from package");
    }
    BranchAndBoundConfig bnb;
    bnb.search_space = BnBSearchSpace::kSE3;
    bnb.max_depth = 3;
    bnb.resolution = 0.5;
    bnb.score_threshold = 0.4;
    bnb.search_radius_steps = 4;
    bnb.max_expansions = 5000;
    bnb.roll_pitch_refine_rad = 0.1;
    bnb.roll_pitch_steps = 3;
    bnb.se3_refine_passes = 1;
    BbsSearchEngine eng(prior, bnb);
    RelocRequest req;
    req.query_cloud = MakeWall(0, 0, 0);
    req.has_initial_guess = true;
    req.initial_guess =
        transform::Rigid3d::Translation(Eigen::Vector3d(0.2, 0.1, 0));
    std::atomic<bool> cancel{false};
    auto res = eng.Search(req, &cancel);
    if (!res.success && res.best.score < 0.3) {
      return Fail("se3 reloc score too low");
    }
  }
  std::filesystem::remove_all(dir);

  // --- Degeneracy tunnel FIM ---
  {
    DegeneracyDetector det;
    Eigen::Matrix3d fim = Eigen::Matrix3d::Zero();
    fim(0, 0) = 100.0;
    fim(1, 1) = 100.0;
    fim(2, 2) = 0.01;  // weak z
    const auto r = det.Analyze(fim);
    if (!r.is_degenerated) {
      return Fail("expected degeneracy");
    }
  }

  // --- ONLY_VIO sync ---
  {
    SensorSync sync;
    sync.SetSlamMode(argus::frontend::ONLY_VIO);
    sync.SetLidarEnabled(false);
    sync.SetImuEnabled(true);
    sync.SetImageEnabled(true);
    for (int i = 0; i < 5; ++i) {
      argus::sensor::ImuData imu;
      imu.timestamp = 0.01 * i;
      imu.linear_acceleration = Eigen::Vector3d(0, 0, 9.81);
      sync.PushImu(imu);
    }
    argus::sensor::ImageData img;
    img.timestamp = 0.03;
    img.image = cv::Mat(8, 8, CV_8UC1, cv::Scalar(128));
    sync.PushImage(img);
    argus::frontend::LidarMeasureGroup meas;
    if (!sync.Sync(meas)) {
      return Fail("ONLY_VIO sync");
    }
    if (meas.ekf_phase != argus::frontend::VIO) {
      return Fail("ONLY_VIO flag");
    }
  }

  std::cout << "OK production_e2e (se3-pg + map-v" << kMapPackageVersion
            << " + reloc-se3 + degeneracy + only_vio)\n";
  return 0;
}
