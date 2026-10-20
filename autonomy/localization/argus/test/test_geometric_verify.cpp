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

#include "argus/common/math/transform/transform.hpp"
#include "argus/relocalization/loop_closure/geometric_verify.hpp"

namespace transform = argus::transform;

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

argus::sensor::PointCloudXYZI MakeBox(double ox) {
  argus::sensor::PointCloudXYZI cloud;
  for (int x = 0; x < 20; ++x) {
    for (int y = 0; y < 20; ++y) {
      argus::sensor::PointType p;
      p.x = static_cast<float>(ox + 0.2 * x);
      p.y = static_cast<float>(0.2 * y);
      p.z = 0.5f;
      p.intensity = 1.f;
      cloud.push_back(p);
    }
  }
  return cloud;
}

}  // namespace

int main() {
  using argus::loop_closure::GeometricVerifyConfig;
  using argus::loop_closure::VerifyAndRefineLoop;

  const auto match = MakeBox(0.0);
  // Query = match rotated+translated (known ground truth).
  const transform::Rigid3d T_true =
      transform::FromXyzRpy(0.3, -0.2, 0.0, 0.0, 0.0, 0.15);
  argus::sensor::PointCloudXYZI query;
  query.reserve(match.size());
  for (const auto& p : match.points) {
    const Eigen::Vector3d pw = T_true * Eigen::Vector3d(p.x, p.y, p.z);
    argus::sensor::PointType q = p;
    q.x = static_cast<float>(pw.x());
    q.y = static_cast<float>(pw.y());
    q.z = static_cast<float>(pw.z());
    query.push_back(q);
  }

  GeometricVerifyConfig cfg;
  cfg.subsample_stride = 2;
  cfg.max_fitness_m = 0.15;
  cfg.min_correspondences = 30;
  cfg.min_inlier_ratio = 0.2;
  // Regular grid is lattice-ambiguous; pin to single seed for accuracy check.
  cfg.seed_xy_steps = 0;
  cfg.seed_yaw_steps = 0;
  const transform::Rigid3d seed =
      transform::FromXyzRpy(0.0, 0.0, 0.0, 0.0, 0.0, 0.05);
  const auto ok = VerifyAndRefineLoop(query, match, seed, cfg);
  if (!ok.ok) {
    std::cerr << "fitness=" << ok.fitness_m << " n=" << ok.correspondences
              << std::endl;
    return Fail("should accept");
  }
  const double err_t =
      (ok.T_query_match.translation() - T_true.translation()).norm();
  if (err_t > 0.25) {
    std::cerr << "err_t=" << err_t << std::endl;
    return Fail("translation refine");
  }

  // Multi-seed path should still accept a near-correct seed.
  GeometricVerifyConfig multi = cfg;
  multi.seed_xy_steps = 1;
  multi.seed_xy_step_m = 0.15;
  multi.seed_yaw_steps = 1;
  multi.seed_yaw_step_rad = 0.05;
  const auto multi_ok = VerifyAndRefineLoop(query, match, seed, multi);
  if (!multi_ok.ok) {
    return Fail("multi-seed should accept");
  }

  // Outrageous seed / wrong pair should reject.
  const auto bad =
      VerifyAndRefineLoop(query, MakeBox(50.0), transform::Rigid3d::Identity(),
                          cfg);
  if (bad.ok) {
    return Fail("should reject distant clouds");
  }

  std::cout << "OK geometric_verify (fitness=" << ok.fitness_m
            << " err_t=" << err_t << ")" << std::endl;
  return 0;
}
