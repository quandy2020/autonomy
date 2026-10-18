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

#include "argus/frontend/spatiotemporal_dynamic_filter.hpp"
#include "argus/frontend/types.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

argus::sensor::PointCloudXYZI MakeRayCloud(float range) {
  argus::sensor::PointCloudXYZI cloud;
  argus::sensor::PointType p;
  p.x = range;
  p.y = 0.f;
  p.z = 0.f;
  p.intensity = 1.f;
  cloud.push_back(p);
  return cloud;
}

}  // namespace

int main() {
  using argus::frontend::SpatiotemporalDynamicFilter;
  using argus::frontend::SpatiotemporalDynamicFilterConfig;
  using argus::frontend::StatesGroup;
  using argus::frontend::DynamicFilterBase;

  SpatiotemporalDynamicFilterConfig cfg;
  cfg.range_diff_m = 0.5;
  cfg.azimuth_res_deg = 1.0;
  cfg.elevation_res_deg = 1.0;
  cfg.min_range_m = 0.5;
  SpatiotemporalDynamicFilter filter(cfg);
  StatesGroup state;

  // Frame 1: static wall at 10 m — keep all, seed history.
  auto c1 = MakeRayCloud(10.f);
  auto m1 = filter.FilterLidar(c1, state);
  if (m1.keep.size() != 1 || m1.keep[0] != 1) {
    return Fail("first frame must keep");
  }

  // Frame 2: same bearing, object at 5 m — reject as dynamic.
  auto c2 = MakeRayCloud(5.f);
  auto m2 = filter.FilterLidar(c2, state);
  if (m2.keep.size() != 1 || m2.keep[0] != 0) {
    return Fail("closer intrusion should be rejected");
  }
  if (m2.RejectedRatio() < 0.99) {
    return Fail("rejected ratio should be ~1");
  }

  DynamicFilterBase::ApplyLidarMask(m2, &c2);
  if (!c2.empty()) {
    return Fail("ApplyLidarMask should remove dynamic point");
  }

  // Static: same range again after reset history via new filter.
  SpatiotemporalDynamicFilter filter2(cfg);
  auto a = MakeRayCloud(8.f);
  filter2.FilterLidar(a, state);
  auto b = MakeRayCloud(8.f);
  auto mb = filter2.FilterLidar(b, state);
  if (mb.keep[0] != 1) {
    return Fail("static second frame must keep");
  }

  std::cout << "OK dynamic_filter spatiotemporal\n";
  return 0;
}
