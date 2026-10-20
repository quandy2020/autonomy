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

#include "argus/map/map_reload_worker.hpp"

namespace transform = argus::transform;

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  argus::map::MapReloadConfig cfg;
  cfg.enable = true;
  cfg.points_per_frame = 10;
  cfg.downsample_resolution_m = 0.5;
  argus::map::MapReloadWorker worker(cfg);

  argus::sensor::PointCloudXYZI cloud;
  for (int i = 0; i < 100; ++i) {
    argus::sensor::PointType p;
    p.x = static_cast<float>(0.1 * i);
    p.y = 0.f;
    p.z = 0.f;
    p.intensity = 1.f;
    cloud.push_back(p);
  }
  const transform::Rigid3d delta =
      transform::Rigid3d::Translation(Eigen::Vector3d(1, 2, 3));
  worker.EnqueueTransformed(cloud, delta);

  argus::sensor::PointCloudXYZI out;
  const std::size_t n1 = worker.Drain(&out, 10);
  if (n1 == 0 || n1 > 10) {
    return Fail("drain budget");
  }
  const std::size_t n2 = worker.Drain(&out, 10000);
  if (out.empty()) {
    return Fail("empty out");
  }
  // First point ~ (0,0,0) → (1,2,3) after delta (unless downsampled away).
  bool found = false;
  for (const auto& p : out.points) {
    if (std::abs(p.x - 1.f) < 0.6f && std::abs(p.y - 2.f) < 0.6f &&
        std::abs(p.z - 3.f) < 0.6f) {
      found = true;
      break;
    }
  }
  if (!found) {
    return Fail("transformed point missing");
  }
  (void)n2;
  std::cout << "OK map_reload (drained=" << out.size() << ")" << std::endl;
  return 0;
}
