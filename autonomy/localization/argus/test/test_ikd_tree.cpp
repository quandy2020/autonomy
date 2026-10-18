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

#include "argus/map/ikd_tree.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  using argus::map::IkdTree;
  using argus::map::IkdTreeConfig;
  using argus::sensor::PointCloudXYZI;
  using argus::sensor::PointType;

  PointCloudXYZI cloud;
  for (int i = 0; i < 50; ++i) {
    PointType p;
    p.x = static_cast<float>(i);
    p.y = 0.f;
    p.z = 0.f;
    cloud.push_back(p);
  }

  IkdTreeConfig cfg;
  cfg.downsample_resolution = 0.0;
  IkdTree tree(cfg);
  tree.Build(cloud);
  if (tree.size() != 50) {
    return Fail("build size");
  }

  std::vector<Eigen::Vector3d> nn;
  std::vector<float> d2;
  const int k = tree.NearestSearch(Eigen::Vector3d(10.1, 0.0, 0.0), 3, &nn, &d2);
  if (k != 3 || nn.empty()) {
    return Fail("knn count");
  }
  if (std::abs(nn.front().x() - 10.0) > 0.5) {
    return Fail("knn nearest");
  }

  PointCloudXYZI extra;
  for (int i = 50; i < 60; ++i) {
    PointType p;
    p.x = static_cast<float>(i);
    extra.push_back(p);
  }
  tree.AddPoints(extra);
  if (tree.size() != 60) {
    return Fail("add size");
  }

  std::vector<Eigen::Vector3d> rad;
  const int nr =
      tree.RadiusSearch(Eigen::Vector3d(55.0, 0, 0), 2.5f, &rad, &d2);
  if (nr < 3) {
    return Fail("radius search");
  }

  std::cout << "OK ikd_tree (size=" << tree.size() << " knn0=" << nn.front().x()
            << " rad=" << nr << ")\n";
  return 0;
}
