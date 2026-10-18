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

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "argus/relocalization/bbs_search_engine.hpp"
#include "argus/relocalization/prior_map_loader.hpp"
#include "argus/relocalization/relocalization_manager.hpp"
#include "argus/relocalization/relocalization_thread_pool.hpp"

namespace transform = argus::transform;

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

argus::sensor::PointCloudXYZI MakeWallCloud(double x0, double y0,
                                            double yaw = 0.0) {
  argus::sensor::PointCloudXYZI cloud;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (int i = 0; i < 30; ++i) {
    for (int j = -5; j <= 5; ++j) {
      const double lx = 0.3 * i;
      const double ly = 0.4 * j;
      argus::sensor::PointType p;
      p.x = static_cast<float>(x0 + c * lx - s * ly);
      p.y = static_cast<float>(y0 + s * lx + c * ly);
      p.z = 1.0f;
      p.intensity = 1.f;
      cloud.push_back(p);
    }
  }
  return cloud;
}

}  // namespace

int main() {
  using argus::relocalization::BbsSearchEngine;
  using argus::relocalization::BranchAndBoundConfig;
  using argus::relocalization::PriorMapLoader;
  using argus::relocalization::RelocRequest;
  using argus::relocalization::RelocalizationConfig;
  using argus::relocalization::RelocalizationManager;
  using argus::relocalization::RelocalizationPoolConfig;
  using argus::relocalization::RelocalizationThreadPool;

  // --- Pool drops oldest when full ---
  RelocalizationPoolConfig pcfg;
  pcfg.worker_count = 1;
  pcfg.max_queue_size = 2;
  pcfg.on_full = RelocalizationPoolConfig::OnFull::kDropOldest;
  RelocalizationThreadPool pool(pcfg);
  pool.Start();
  std::atomic<int> ran{0};
  auto blocker = [&](std::atomic<bool>& cancel) {
    while (!cancel.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ran.fetch_add(1);
  };
  // Fill: 1 running + 2 queued; further submits drop oldest.
  pool.Submit(blocker);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  pool.Submit([&](std::atomic<bool>&) { ran.fetch_add(1); });
  pool.Submit([&](std::atomic<bool>&) { ran.fetch_add(1); });
  pool.Submit([&](std::atomic<bool>&) { ran.fetch_add(1); });  // drops one
  if (pool.dropped_count() < 1) {
    return Fail("expected drop on full queue");
  }
  pool.CancelAll();
  pool.Stop();

  // --- BnB finds shifted corridor ---
  const auto prior_cloud = MakeWallCloud(0.0, 0.0, 0.0);
  auto prior = PriorMapLoader::FromCloud(prior_cloud, 0.4);
  if (!prior || prior->size() == 0) {
    return Fail("prior empty");
  }

  BranchAndBoundConfig bnb;
  bnb.max_depth = 4;
  bnb.resolution = 0.5;
  bnb.yaw_step_rad = 0.15;
  bnb.score_threshold = 0.55;
  bnb.search_radius_steps = 6;
  bnb.max_expansions = 8000;
  bnb.search_space = argus::relocalization::BnBSearchSpace::kSE2;
  bnb.roll_pitch_refine_rad = 0.0;
  BbsSearchEngine engine(prior, bnb);

  RelocRequest req;
  req.id = 1;
  // Query = same structure, true pose near (1, 0.5, 0).
  req.query_cloud = MakeWallCloud(0.0, 0.0, 0.0);  // body-frame pattern
  req.has_initial_guess = true;
  req.initial_guess =
      transform::Rigid3d::Translation(Eigen::Vector3d(0.2, 0.1, 0));

  // Build prior so that body cloud at T=(1,0.5) matches: insert transformed.
  {
    auto aligned = MakeWallCloud(0.0, 0.0, 0.0);
    argus::sensor::PointCloudXYZI world;
    const transform::Rigid3d T =
        transform::Rigid3d::Translation(Eigen::Vector3d(1.0, 0.5, 0));
    for (std::size_t i = 0; i < aligned.size(); ++i) {
      Eigen::Vector3d pw =
          T * Eigen::Vector3d(aligned[i].x, aligned[i].y, aligned[i].z);
      argus::sensor::PointType p;
      p.x = static_cast<float>(pw.x());
      p.y = static_cast<float>(pw.y());
      p.z = static_cast<float>(pw.z());
      world.push_back(p);
    }
    prior = PriorMapLoader::FromCloud(world, 0.4);
    engine.SetMap(prior);
    req.initial_guess =
        transform::Rigid3d::Translation(Eigen::Vector3d(0.3, 0.2, 0));
  }

  std::atomic<bool> cancel{false};
  auto result = engine.Search(req, &cancel);
  if (!result.success) {
    std::cerr << "best score=" << result.best.score << std::endl;
    return Fail("BnB should succeed");
  }
  const Eigen::Vector3d t = result.best.T_w_b.translation();
  const transform::Rigid3d T_gt =
      transform::Rigid3d::Translation(Eigen::Vector3d(1.0, 0.5, 0));
  if ((t - T_gt.translation()).head<2>().norm() > 0.6) {
    std::cerr << "t=" << t.transpose() << std::endl;
    return Fail("pose too far from truth");
  }
  {
    using argus::relocalization::EvaluateAgainstGroundTruth;
    const auto m =
        EvaluateAgainstGroundTruth(result.best.T_w_b, T_gt, 0.7, 0.5);
    if (!m.within_tolerance) {
      std::cerr << "trans=" << m.translation_error_m << " rot=" << m.rotation_error_rad
                << std::endl;
      return Fail("GT eval within_tolerance");
    }
  }

  // --- SE3Yaw recovers z offset ---
  {
    BranchAndBoundConfig bnb3 = bnb;
    bnb3.search_space = argus::relocalization::BnBSearchSpace::kSE3Yaw;
    bnb3.z_resolution = 0.5;
    bnb3.search_radius_z_steps = 4;
    bnb3.max_expansions = 25000;
    bnb3.roll_pitch_refine_rad = 0.1;
    bnb3.roll_pitch_steps = 3;

    argus::sensor::PointCloudXYZI world_z;
    const transform::Rigid3d Tz =
        transform::Rigid3d::Translation(Eigen::Vector3d(1.0, 0.5, 1.0));
    auto body = MakeWallCloud(0.0, 0.0, 0.0);
    for (std::size_t i = 0; i < body.size(); ++i) {
      Eigen::Vector3d pw =
          Tz * Eigen::Vector3d(body[i].x, body[i].y, body[i].z);
      argus::sensor::PointType p;
      p.x = static_cast<float>(pw.x());
      p.y = static_cast<float>(pw.y());
      p.z = static_cast<float>(pw.z());
      world_z.push_back(p);
    }
    auto prior_z = PriorMapLoader::FromCloud(world_z, 0.4);
    BbsSearchEngine eng3(prior_z, bnb3);
    RelocRequest reqz = req;
    reqz.initial_guess =
        transform::Rigid3d::Translation(Eigen::Vector3d(0.3, 0.2, 0.2));
    std::atomic<bool> cz{false};
    auto rz = eng3.Search(reqz, &cz);
    if (!rz.success) {
      std::cerr << "se3 score=" << rz.best.score << std::endl;
      return Fail("SE3Yaw BnB should succeed");
    }
    const double z_err =
        std::abs(rz.best.T_w_b.translation().z() - 1.0);
    if (z_err > 0.6) {
      std::cerr << "z=" << rz.best.T_w_b.translation().z() << std::endl;
      return Fail("SE3Yaw z too far");
    }
    const auto mz = argus::relocalization::EvaluateAgainstGroundTruth(
        rz.best.T_w_b, Tz, 0.8, 0.5);
    if (!mz.within_tolerance) {
      return Fail("SE3Yaw GT tol");
    }
  }

  // --- Manager async ---
  RelocalizationConfig mcfg;
  mcfg.enable = true;
  mcfg.thread_pool.worker_count = 2;
  mcfg.branch_and_bound = bnb;
  RelocalizationManager mgr(mcfg);
  mgr.SetPriorMap(prior);
  mgr.Start();
  RelocRequest req2 = req;
  req2.id = 0;
  if (!mgr.RequestRelocalization(std::move(req2))) {
    return Fail("RequestRelocalization");
  }
  std::vector<argus::relocalization::RelocResult> results;
  for (int i = 0; i < 200; ++i) {
    results = mgr.TakeResults();
    if (!results.empty()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  mgr.Stop();
  if (results.empty()) {
    return Fail("manager produced no result");
  }
  if (!results.front().success) {
    std::cerr << "async score=" << results.front().best.score << std::endl;
    return Fail("manager async result not successful");
  }

  // --- IsTrackingLost + TryAutoTrigger streak / cooldown ---
  {
    using argus::relocalization::IsTrackingLost;
    using argus::relocalization::TrackingLostSignals;
    TrackingLostSignals ok;
    if (IsTrackingLost(ok)) {
      return Fail("healthy signals should not be lost");
    }
    TrackingLostSignals gate;
    gate.enable_gating = true;
    gate.lidar_valid = false;
    if (!IsTrackingLost(gate)) {
      return Fail("gating reject should be lost");
    }
  }

  RelocalizationConfig acfg;
  acfg.enable = true;
  acfg.trigger = "tracking_lost";
  acfg.min_lost_streak = 3;
  acfg.cooldown_sec = 10.0;
  acfg.thread_pool.worker_count = 1;
  acfg.branch_and_bound = bnb;
  RelocalizationManager auto_mgr(acfg);
  auto_mgr.SetPriorMap(prior);
  auto_mgr.Start();
  RelocRequest areq = req;
  areq.id = 0;
  if (auto_mgr.TryAutoTrigger(true, 1.0, areq)) {
    return Fail("auto should wait for streak");
  }
  if (auto_mgr.TryAutoTrigger(true, 1.1, areq)) {
    return Fail("auto should wait for streak=2");
  }
  if (!auto_mgr.TryAutoTrigger(true, 1.2, areq)) {
    return Fail("auto should fire at streak=3");
  }
  if (auto_mgr.TryAutoTrigger(true, 1.3, areq)) {
    return Fail("auto should respect cooldown");
  }
  if (auto_mgr.auto_trigger_count() != 1) {
    return Fail("auto_trigger_count");
  }
  // Recovery clears streak.
  if (auto_mgr.TryAutoTrigger(false, 20.0, areq)) {
    return Fail("healthy frame must not trigger");
  }
  if (auto_mgr.lost_streak() != 0) {
    return Fail("streak reset");
  }
  auto_mgr.Stop();

  std::cout << "OK relocalization (score=" << result.best.score
            << " t=[" << t.x() << "," << t.y() << "] dropped="
            << pool.dropped_count() << ")\n";
  return 0;
}
