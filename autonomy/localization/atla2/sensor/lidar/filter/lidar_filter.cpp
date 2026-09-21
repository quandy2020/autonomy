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

#include "autonomy/localization/atla2/sensor/lidar/filter/lidar_filter.hpp"

#include <cmath>

namespace autonomy::localization::atla2 {

LidarFilter::LidarFilter(LidarFilterOptions opt) : opt_(opt) {}

LidarScan LidarFilter::Process(const LidarScan& in) const {
  LidarScan out;
  out.t = in.t;
  out.t_begin = in.t_begin;
  out.frame_id = in.frame_id;
  out.model = in.model;
  out.points.reserve(in.points.size());

  const float r2_th = opt_.radius * opt_.radius;
  for (size_t i = 0; i < in.points.size(); ++i) {
    const auto& p = in.points[i];
    if (opt_.remove_ground && p.z < opt_.ground_z_max) {
      continue;
    }
    int neighbors = 0;
    for (size_t j = 0; j < in.points.size(); ++j) {
      if (i == j) {
        continue;
      }
      const auto& q = in.points[j];
      const float dx = p.x - q.x;
      const float dy = p.y - q.y;
      const float dz = p.z - q.z;
      if (dx * dx + dy * dy + dz * dz <= r2_th) {
        ++neighbors;
        if (neighbors >= opt_.min_neighbors) {
          break;
        }
      }
    }
    if (neighbors >= opt_.min_neighbors) {
      out.points.push_back(p);
    }
  }
  return out;
}

}  // namespace autonomy::localization::atla2
