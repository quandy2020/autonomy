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

#include "autonomy/localization/atla2/frontend/lio/lio_frontend.hpp"

#include <cmath>
#include <unordered_set>

#include "autonomy/localization/atla2/frontend/imu_preintegration.hpp"

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

bool LioFrontend::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  Reset();
  return true;
}

void LioFrontend::Reset() {
  result_ = OdometryResult{};
  local_map_.clear();
  initialized_ = false;
}

void LioFrontend::AccumulateMap(const LidarScan& scan) {
  const float vs = static_cast<float>(cfg_.voxel_size > 0 ? cfg_.voxel_size : 0.2);
  std::unordered_set<int64_t> occupied;
  occupied.reserve(local_map_.size() + scan.points.size());
  for (const auto& p : local_map_) {
    occupied.insert(VoxelKey(p.x, p.y, p.z, vs));
  }

  const Mat33 R = Se3RotationMatrix(result_.pose);
  const Vec3 t = Se3Translation(result_.pose);
  for (const auto& pt : scan.points) {
    const Vec3 pw = R * Vec3(pt.x, pt.y, pt.z) + t;
    const auto key = VoxelKey(static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                              static_cast<float>(pw.z()), vs);
    if (occupied.count(key)) {
      continue;
    }
    occupied.insert(key);
    PointXYZI w;
    w.x = static_cast<float>(pw.x());
    w.y = static_cast<float>(pw.y());
    w.z = static_cast<float>(pw.z());
    w.intensity = pt.intensity;
    w.timestamp = pt.timestamp;
    local_map_.push_back(w);
  }
  if (static_cast<int>(local_map_.size()) > cfg_.max_local_map_points) {
    local_map_.erase(local_map_.begin(),
                     local_map_.begin() + (local_map_.size() - cfg_.max_local_map_points));
  }
  result_.local_map = local_map_;
}

bool LioFrontend::Process(const SensorData& data) {
  if (!data.has_imu && !data.has_lidar) {
    return false;
  }
  result_.t = data.t;
  if (!initialized_) {
    result_.pose = Se3Identity();
    result_.velocity = Vec3::Zero();
    result_.valid = true;
    initialized_ = true;
    if (data.has_lidar) {
      AccumulateMap(data.lidar);
    }
    return true;
  }
  if (data.has_imu && data.imu.size() >= 2) {
    ImuPropagate(data.imu, result_.accel_bias, result_.gyro_bias, gravity_,
                 &result_.pose, &result_.velocity);
  }
  if (data.has_lidar) {
    // Registration placeholder: accumulate transformed scan into local map.
    AccumulateMap(data.lidar);
  }
  result_.valid = true;
  result_.cov.matrix = Mat66::Identity() * 0.02;
  return true;
}

bool LioFrontend::GetResult(OdometryResult* out) {
  if (!out || !result_.valid) {
    return false;
  }
  *out = result_;
  return true;
}

}  // namespace autonomy::localization::atla2
