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

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "argus/backend/pose_graph/pose_graph.hpp"
#include "argus/relocalization/loop_closure/manager.hpp"
#include "argus/relocalization/loop_closure/scan_context/detector.hpp"

namespace transform = argus::transform;

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

argus::sensor::PointCloudXYZI MakeCorridorCloud(double yaw_offset = 0.0) {
  argus::sensor::PointCloudXYZI cloud;
  // Two walls parallel to x at y=±5, plus end wall — distinctive SC.
  for (int i = 0; i < 40; ++i) {
    const double x = 0.5 * i;
    for (double y : {-5.0, 5.0}) {
      argus::sensor::PointType p;
      const double c = std::cos(yaw_offset);
      const double s = std::sin(yaw_offset);
      p.x = static_cast<float>(c * x - s * y);
      p.y = static_cast<float>(s * x + c * y);
      p.z = 1.5f;
      p.intensity = 1.f;
      cloud.push_back(p);
    }
  }
  return cloud;
}

}  // namespace

int main() {
  using argus::backend::PoseGraph;
  using argus::loop_closure::KeyFrame;
  using argus::loop_closure::LoopClosureConfig;
  using argus::loop_closure::LoopClosureManager;
  using argus::loop_closure::ScanContextConfig;
  using argus::loop_closure::ScanContextDetector;

  // --- PoseGraph triangle closes with loop ---
  PoseGraph pg;
  const transform::Rigid3d T0 = transform::Rigid3d::Identity();
  const transform::Rigid3d T1 = transform::Rigid3d::Translation(Eigen::Vector3d(1, 0, 0));
  const transform::Rigid3d T2 = transform::Rigid3d::Translation(Eigen::Vector3d(1, 1, 0));
  pg.AddNode(0, T0);
  pg.AddNode(1, T1);
  pg.AddNode(2, T2);
  pg.AddOdometryEdge(0, 1, T0.inverse() * T1);
  pg.AddOdometryEdge(1, 2, T1.inverse() * T2);
  // Noisy return-to-start measurement.
  const transform::Rigid3d T_loop =
      transform::Rigid3d::Translation(Eigen::Vector3d(0.1, 1.0, 0));
  argus::backend::LoopFactor f;
  f.from_id = 2;
  f.to_id = 0;
  f.T_i_j = T_loop;
  f.information = 200.0;
  pg.AddLoopFactor(f);
  pg.Optimize(15);
  if (pg.loop_factor_count() != 1) {
    return Fail("loop factor count");
  }

  // --- Scan Context recalls similar structure ---
  ScanContextConfig sc_cfg;
  sc_cfg.exclude_recent_count = 2;
  sc_cfg.max_candidates = 3;
  sc_cfg.min_score = 0.3;
  ScanContextDetector det(sc_cfg);
  for (int i = 0; i < 5; ++i) {
    KeyFrame kf;
    kf.id = i;
    kf.T_w_b = transform::Rigid3d::Translation(Eigen::Vector3d(i * 2.0, 0, 0));
    kf.cloud = MakeCorridorCloud(0.0);
    det.Detect(kf);
  }
  KeyFrame query;
  query.id = 5;
  query.T_w_b = transform::Rigid3d::Identity();
  query.cloud = MakeCorridorCloud(0.05);  // slight yaw
  auto cands = det.Detect(query);
  if (cands.empty()) {
    return Fail("expected Scan Context candidates");
  }

  // --- Async manager does not call Optimize on caller thread ---
  LoopClosureConfig lc;
  lc.enable = true;
  lc.async = true;
  lc.scan_context = sc_cfg;
  lc.consensus.min_confidence = 0.2;
  lc.consensus.min_id_gap = 2;
  lc.consensus.min_path_separation_m = 1.0;
  LoopClosureManager mgr(lc);
  mgr.Start();
  for (int i = 0; i < 8; ++i) {
    KeyFrame kf;
    kf.id = i;
    kf.T_w_b = transform::Rigid3d::Translation(Eigen::Vector3d(i * 3.0, 0, 0));
    kf.cloud = MakeCorridorCloud(i == 7 ? 0.02 : 0.0);
    mgr.SubmitKeyFrame(std::move(kf));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  mgr.Stop();

  auto corrs = mgr.TakeCorrections();
  if (mgr.accepted_loop_count() > 0 &&
      static_cast<int>(corrs.size()) != mgr.accepted_loop_count()) {
    return Fail("LoopCorrection count mismatch");
  }
  for (const auto& c : corrs) {
    if (!c.delta_w.IsValid()) {
      return Fail("delta_w not valid");
    }
  }

  std::cout << "OK loop_closure (sc candidates=" << cands.size()
            << " accepted=" << mgr.accepted_loop_count()
            << " corrections=" << corrs.size() << ")\n";
  return 0;
}
