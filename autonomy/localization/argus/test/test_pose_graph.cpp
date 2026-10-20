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

#include <cmath>
#include <iostream>

#include "argus/backend/pose_graph/pose_graph.hpp"

namespace transform = argus::transform;

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  using argus::backend::LoopFactor;
  using argus::backend::PoseGraph;
  using argus::backend::PoseGraphFprConfig;

  PoseGraph pg;
  pg.AddNode(0, transform::Rigid3d::Identity());
  pg.AddNode(1, transform::Rigid3d::Translation(Eigen::Vector3d(1, 0, 0)));
  pg.AddNode(2, transform::Rigid3d::Translation(Eigen::Vector3d(1, 1, 0)));
  pg.AddOdometryEdge(0, 1, transform::Rigid3d::Translation(Eigen::Vector3d(1, 0, 0)));
  pg.AddOdometryEdge(1, 2, transform::Rigid3d::Translation(Eigen::Vector3d(0, 1, 0)));

  const auto snap = pg.Snapshot();
  if (snap.poses.size() != 3) {
    return Fail("snapshot size");
  }

  LoopFactor good;
  good.from_id = 2;
  good.to_id = 0;
  good.T_i_j = transform::Rigid3d::Translation(Eigen::Vector3d(0.05, 1.0, 0));
  good.information = 200.0;

  PoseGraphFprConfig fpr;
  fpr.enable = true;
  fpr.max_loop_residual = 5.0;
  auto ok = pg.TryAcceptLoops({good}, fpr, 20);
  if (!ok.ok) {
    return Fail("good loop rejected");
  }
  if (pg.loop_factor_count() != 1) {
    return Fail("loop count after accept");
  }

  // Outrageous loop should be rejected and leave graph unchanged.
  const int loops_before = pg.loop_factor_count();
  const transform::Rigid3d p2_before = pg.Pose(2);
  LoopFactor bad;
  bad.from_id = 2;
  bad.to_id = 0;
  bad.T_i_j = transform::Rigid3d::Translation(Eigen::Vector3d(50, 50, 50));
  bad.information = 200.0;
  fpr.max_loop_residual = 0.5;
  auto bad_res = pg.TryAcceptLoops({bad}, fpr, 20);
  if (bad_res.ok || !bad_res.rejected_by_fpr) {
    return Fail("bad loop should FPR-reject");
  }
  if (pg.loop_factor_count() != loops_before) {
    return Fail("loop count changed after reject");
  }
  if ((pg.Pose(2).translation() - p2_before.translation()).norm() > 1e-6) {
    return Fail("pose changed after reject");
  }

  pg.Restore(snap);
  if (pg.loop_factor_count() != 0) {
    return Fail("restore loop count");
  }

#if defined(ARGUS_HAS_CERES) && ARGUS_HAS_CERES
  std::cout << "OK pose_graph (ceres+fpr)" << std::endl;
#else
  std::cout << "OK pose_graph (gn+fpr)" << std::endl;
#endif
  return 0;
}
