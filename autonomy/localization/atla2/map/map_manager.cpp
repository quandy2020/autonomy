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

#include "autonomy/localization/atla2/map/map_manager.hpp"

#include <fstream>

#include "autonomy/localization/atla2/common/time.hpp"

namespace autonomy::localization::atla2 {

bool MapManager::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  Reset();
  return true;
}

void MapManager::Reset() {
  std::lock_guard<std::mutex> lock(mu_);
  keyframes_.clear();
  local_cloud_.clear();
  landmarks_.clear();
  next_kf_id_ = 0;
}

void MapManager::AddKeyframe(const Keyframe& kf) {
  std::lock_guard<std::mutex> lock(mu_);
  Keyframe copy = kf;
  if (copy.id < 0) {
    copy.id = next_kf_id_++;
  }
  keyframes_.push_back(std::move(copy));
}

void MapManager::UpdateFromOdometry(const OdometryResult& odom) {
  if (!odom.valid) {
    return;
  }
  std::lock_guard<std::mutex> lock(mu_);
  if (!odom.local_map.empty()) {
    local_cloud_ = odom.local_map;
  }
  if (!odom.landmarks.empty()) {
    landmarks_ = odom.landmarks;
  }
  // Insert keyframe every ~0.5 s of motion
  const bool need_kf =
      keyframes_.empty() ||
      (odom.t - keyframes_.back().t) > SecToStamp(0.5);
  if (need_kf) {
    Keyframe kf;
    kf.id = next_kf_id_++;
    kf.t = odom.t;
    kf.pose = odom.pose;
    kf.landmarks = odom.landmarks;
    kf.cloud = odom.local_map;
    keyframes_.push_back(std::move(kf));
  }
}

bool MapManager::SavePcd(const std::string& path) const {
  std::lock_guard<std::mutex> lock(mu_);
  std::ofstream ofs(path);
  if (!ofs) {
    return false;
  }
  ofs << "# .PCD v0.7 - Point Cloud Data file format\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z intensity\n"
      << "SIZE 4 4 4 4\n"
      << "TYPE F F F F\n"
      << "COUNT 1 1 1 1\n"
      << "WIDTH " << local_cloud_.size() << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << local_cloud_.size() << "\n"
      << "DATA ascii\n";
  for (const auto& p : local_cloud_) {
    ofs << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << '\n';
  }
  return true;
}

}  // namespace autonomy::localization::atla2
