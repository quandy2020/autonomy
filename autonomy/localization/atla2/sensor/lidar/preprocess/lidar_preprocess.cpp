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

#include "autonomy/localization/atla2/sensor/lidar/preprocess/lidar_preprocess.hpp"

#include <cmath>
#include <unordered_set>

namespace autonomy::localization::atla2 {

namespace {

int64_t VoxelKey(float x, float y, float z, float vs) {
  const auto qx = static_cast<int32_t>(std::floor(x / vs));
  const auto qy = static_cast<int32_t>(std::floor(y / vs));
  const auto qz = static_cast<int32_t>(std::floor(z / vs));
  return (static_cast<int64_t>(qx) & 0x1FFFFF) |
         ((static_cast<int64_t>(qy) & 0x1FFFFF) << 21) |
         ((static_cast<int64_t>(qz) & 0x1FFFFF) << 42);
}

}  // namespace

LidarPreprocessor::LidarPreprocessor(LidarPreprocessOptions opt) : opt_(opt) {}

LidarScan LidarPreprocessor::Process(const LidarScan& in) const {
  LidarScan out;
  out.t = in.t;
  out.t_begin = in.t_begin;
  out.frame_id = in.frame_id;
  out.model = in.model;
  out.points.reserve(in.points.size() / 2);

  const float vs = opt_.voxel_size > 1e-4f ? opt_.voxel_size : 0.2f;
  std::unordered_set<int64_t> occupied;
  occupied.reserve(in.points.size());

  for (const auto& p : in.points) {
    const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
    const float rmin2 = opt_.min_range * opt_.min_range;
    const float rmax2 = opt_.max_range * opt_.max_range;
    if (r2 < rmin2 || r2 > rmax2) {
      continue;
    }
    if (opt_.use_roi) {
      if (p.x < opt_.roi_xmin || p.x > opt_.roi_xmax || p.y < opt_.roi_ymin ||
          p.y > opt_.roi_ymax || p.z < opt_.roi_zmin || p.z > opt_.roi_zmax) {
        continue;
      }
    }
    const auto key = VoxelKey(p.x, p.y, p.z, vs);
    if (!occupied.insert(key).second) {
      continue;
    }
    out.points.push_back(p);
  }
  return out;
}

}  // namespace autonomy::localization::atla2
