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
#include <string>
#include <vector>

#include "argus/backend/pose_graph/pose_graph.hpp"
#include "argus/relocalization/loop_closure/cross_session.hpp"
#include "argus/map/map_session.hpp"

namespace transform = argus::transform;

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

std::vector<argus::map::SessionKeyFrame> MakeSquareTraj(int n_side) {
  std::vector<argus::map::SessionKeyFrame> traj;
  int id = 0;
  auto push = [&](double x, double y) {
    argus::map::SessionKeyFrame kf;
    kf.local_id = id++;
    kf.stamp_sec = 0.1 * kf.local_id;
    kf.T_session_b = transform::Rigid3d::Translation(Eigen::Vector3d(x, y, 0));
    traj.push_back(kf);
  };
  for (int i = 0; i <= n_side; ++i) {
    push(static_cast<double>(i), 0.0);
  }
  for (int i = 1; i <= n_side; ++i) {
    push(static_cast<double>(n_side), static_cast<double>(i));
  }
  for (int i = 1; i <= n_side; ++i) {
    push(static_cast<double>(n_side - i), static_cast<double>(n_side));
  }
  for (int i = 1; i < n_side; ++i) {
    push(0.0, static_cast<double>(n_side - i));
  }
  return traj;
}

}  // namespace

int main() {
  using argus::backend::PoseGraph;
  using argus::loop_closure::CrossSessionConfig;
  using argus::loop_closure::CrossSessionLink;
  using argus::loop_closure::CrossSessionLinker;
  using argus::map::MapSession;
  using argus::map::SessionMeta;

  // --- Serialize / load registry ---
  MapSession reg;
  SessionMeta A;
  A.session_id = "A";
  A.t_start_sec = 0.0;
  A.t_end_sec = 10.0;
  A.node_id_base = 1000;
  A.mean_confidence = 0.9;
  A.T_world_session = transform::Rigid3d::Identity();
  reg.Register(A);
  auto traj_a = MakeSquareTraj(4);
  reg.SetTrajectory("A", traj_a);

  const std::string path = "argus_test_session_A.txt";
  if (!reg.Save(path)) {
    return Fail("save");
  }
  MapSession loaded;
  if (!loaded.Load(path)) {
    return Fail("load");
  }
  if (!loaded.Has("A") || loaded.Trajectory("A").size() != traj_a.size()) {
    return Fail("load traj size");
  }

  // --- Dual session: B drifted, chain link pulls toward A ---
  SessionMeta B;
  B.session_id = "B";
  B.node_id_base = 0;
  B.T_world_session =
      transform::Rigid3d::Translation(Eigen::Vector3d(0.5, -0.3, 0));  // drift
  reg.Register(B);

  auto traj_b = MakeSquareTraj(4);
  // Inject translational drift along the path.
  for (auto& kf : traj_b) {
    const Eigen::Vector3d t =
        kf.T_session_b.translation() +
        Eigen::Vector3d(0.05 * kf.local_id, 0, 0);
    kf.T_session_b = transform::Rigid3d(t, kf.T_session_b.rotation());
  }
  reg.SetTrajectory("B", traj_b);

  CrossSessionConfig xcfg;
  xcfg.enable = true;
  xcfg.min_confidence = 0.5;
  xcfg.loop_information_scale = 300.0;
  xcfg.freeze_prior_nodes = true;
  CrossSessionLinker linker(&reg, xcfg);

  PoseGraph graph;
  if (linker.AddSessionToGraph(graph, "A") <= 0) {
    return Fail("load A into graph");
  }
  if (linker.AddSessionToGraph(graph, "B") <= 0) {
    return Fail("load B into graph");
  }

  // True relative at first keyframe should be identity in shared world if
  // anchors were correct; we measure from drifted B world pose vs A.
  transform::Rigid3d T_w_a0;
  transform::Rigid3d T_w_b0;
  if (!reg.WorldPose("A", 0, &T_w_a0) || !reg.WorldPose("B", 0, &T_w_b0)) {
    return Fail("world pose");
  }
  const double err_before = (T_w_b0.translation() - T_w_a0.translation()).norm();

  CrossSessionLink link;
  link.session_prior = "A";
  link.session_current = "B";
  link.local_id_prior = 0;
  link.local_id_current = 0;
  // Measurement: B0 coincides with A0 in world → T_b←a ≈ T_wb^{-1} T_wa
  // but we want the *correct* relative (identity in bodies when co-located).
  link.T_cur_prior = transform::Rigid3d::Identity();
  link.confidence = 0.95;

  argus::backend::PoseGraphFprConfig fpr;
  fpr.enable = true;
  fpr.max_loop_residual = 5.0;

  if (!linker.LinkAndOptimize(graph, link, fpr, 20)) {
    return Fail("LinkAndOptimize");
  }

  const transform::Rigid3d T_opt_b0 = graph.Pose(reg.GlobalNodeId("B", 0));
  const transform::Rigid3d T_opt_a0 = graph.Pose(reg.GlobalNodeId("A", 0));
  const double err_after =
      (T_opt_b0.translation() - T_opt_a0.translation()).norm();

  if (!(err_after < err_before * 0.5 + 1e-3)) {
    std::cerr << "err_before=" << err_before << " err_after=" << err_after
              << std::endl;
    return Fail("cross-session did not reduce alignment error");
  }

  const auto* b_meta = reg.Get("B");
  if (b_meta == nullptr) {
    return Fail("B meta");
  }
  // Anchor should move toward identity (prior A world stays fixed).
  if (b_meta->T_world_session.translation().norm() > 0.15) {
    return Fail("anchor not refreshed");
  }
  // Prior must remain gauge-fixed.
  if (graph.Pose(reg.GlobalNodeId("A", 0)).translation().norm() > 1e-6) {
    return Fail("prior session drifted");
  }

  // Outrageous measurement must FPR-reject and leave loop count unchanged.
  const int loops_ok = graph.loop_factor_count();
  CrossSessionLink bad = link;
  bad.T_cur_prior =
      transform::Rigid3d::Translation(Eigen::Vector3d(80, 80, 80));
  fpr.max_loop_residual = 0.5;
  argus::backend::PoseGraphOptimizeResult bad_opt;
  if (linker.LinkAndOptimize(graph, bad, fpr, 20, &bad_opt)) {
    return Fail("bad link should fail");
  }
  if (!bad_opt.rejected_by_fpr) {
    return Fail("bad link should set rejected_by_fpr");
  }
  if (graph.loop_factor_count() != loops_ok) {
    return Fail("loop count changed after FPR reject");
  }

  std::cout << "OK cross_session (err " << err_before << " -> " << err_after
            << ", |t_ws_B|=" << b_meta->T_world_session.translation().norm()
            << ", fpr_reject)\n";
  return 0;
}
