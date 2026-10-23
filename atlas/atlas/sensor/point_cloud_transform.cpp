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

#include "sensor/point_cloud_transform.hpp"

#include <cmath>

namespace atlas {
namespace sensor {

PointCloudXYZI TransformPointCloud(const PointCloudXYZI& cloud,
                                   const Eigen::Isometry3f& transform) {
  PointCloudXYZI out;
  out.reserve(cloud.size());
  const Eigen::Matrix3f R = transform.linear();
  for (const auto& p : cloud.points) {
    PointXYZINormal q = p;
    const Eigen::Vector3f t = transform * Eigen::Vector3f(p.x, p.y, p.z);
    q.x = t.x();
    q.y = t.y();
    q.z = t.z();
    const Eigen::Vector3f n =
        R * Eigen::Vector3f(p.normal_x, p.normal_y, p.normal_z);
    q.normal_x = n.x();
    q.normal_y = n.y();
    q.normal_z = n.z();
    out.push_back(q);
  }
  return out;
}

PointCloudXYZI CropPointCloud(const PointCloudXYZI& cloud, float min_z,
                              float max_z) {
  PointCloudXYZI out;
  out.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    if (min_z <= p.z && p.z <= max_z) {
      out.push_back(p);
    }
  }
  return out;
}

PointCloudXYZI RemoveInvalidPoints(const PointCloudXYZI& cloud) {
  PointCloudXYZI out;
  out.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    if (p.x == 0.f && p.y == 0.f && p.z == 0.f) {
      continue;
    }
    out.push_back(p);
  }
  return out;
}

}  // namespace sensor
}  // namespace atlas
