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
#include <vector>

#include "argus/map/map_session.hpp"
#include "argus/relocalization/loop_closure/manager.hpp"

namespace transform = argus::transform;

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

argus::sensor::PointCloudXYZI MakeCloud(float ox) {
  argus::sensor::PointCloudXYZI cloud;
  for (int i = 0; i < 8; ++i) {
    argus::sensor::PointType p;
    p.x = ox + 0.1f * static_cast<float>(i);
    p.y = 0.f;
    p.z = 0.f;
    p.intensity = 1.f;
    cloud.push_back(p);
  }
  return cloud;
}

}  // namespace

int main() {
  using argus::loop_closure::KeyFrame;
  using argus::loop_closure::LoopClosureConfig;
  using argus::loop_closure::LoopClosureManager;
  using argus::map::SessionKeyFrame;
  using argus::map::SessionMeta;

  LoopClosureConfig cfg;
  cfg.enable = true;
  cfg.async = false;  // no worker needed
  cfg.lta.enable = true;
  cfg.lta.downsample_resolution_m = 0.0;  // keep all points
  cfg.lta.prior_neighbor_keyframes = 1;
  cfg.lta_reload_prior_on_cross_session = true;
  LoopClosureManager mgr(cfg);

  SessionMeta prior;
  prior.session_id = "priorA";
  prior.node_id_base = 1000;
  prior.T_world_session =
      transform::Rigid3d::Translation(Eigen::Vector3d(10, 0, 0));

  std::vector<SessionKeyFrame> traj;
  std::vector<KeyFrame> sc_kfs;
  for (int i = 0; i < 5; ++i) {
    SessionKeyFrame sk;
    sk.local_id = i;
    sk.T_session_b =
        transform::Rigid3d::Translation(Eigen::Vector3d(i * 1.0, 0, 0));
    traj.push_back(sk);

    KeyFrame kf;
    kf.id = prior.node_id_base + i;
    kf.session_id = prior.session_id;
    kf.session_local_id = i;
    kf.cloud = MakeCloud(static_cast<float>(i));
    sc_kfs.push_back(kf);
  }

  if (!mgr.RegisterPriorSession(prior, traj, sc_kfs)) {
    return Fail("register prior");
  }
  if (mgr.prior_cloud_count() != 5) {
    return Fail("prior cloud buffer size");
  }

  // Match local_id=2 → neighbors 1,2,3 with radius=1.
  const std::size_t n = mgr.ReloadPriorNeighborhood("priorA", 2);
  if (n != 3) {
    std::cerr << "enqueued=" << n << std::endl;
    return Fail("neighbor enqueue count");
  }
  if (mgr.map_reload().queue_size() != 3) {
    return Fail("reload queue size");
  }

  argus::sensor::PointCloudXYZI out;
  const std::size_t drained = mgr.map_reload().Drain(&out, 10000);
  if (drained == 0 || out.empty()) {
    return Fail("drain empty");
  }
  // Body (0,0,0) at local_id=2 → world ≈ (10+2, 0, 0) = (12,0,0).
  bool found = false;
  for (const auto& p : out.points) {
    if (std::abs(p.x - 12.f) < 0.5f && std::abs(p.y) < 0.5f) {
      found = true;
      break;
    }
  }
  if (!found) {
    return Fail("world-transformed prior point missing");
  }

  std::cout << "OK prior_lta_reload (enqueued=" << n
            << " drained=" << drained << ")" << std::endl;
  return 0;
}
