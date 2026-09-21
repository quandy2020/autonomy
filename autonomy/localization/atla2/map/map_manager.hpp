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

#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "autonomy/localization/atla2/common/config.hpp"
#include "autonomy/localization/atla2/frontend/frontend_base.hpp"

namespace autonomy::localization::atla2 {

struct Keyframe {
  int id = -1;
  TimeStamp t = kInvalidTime;
  SE3 pose = Se3Identity();
  std::vector<Landmark> landmarks;
  PointCloud cloud;
};

//! Unified map: landmarks (VIO) + local/global point cloud (LIO).
class MapManager {
 public:
  bool Init(const Atla2Config& cfg);
  void Reset();

  void AddKeyframe(const Keyframe& kf);
  void UpdateFromOdometry(const OdometryResult& odom);

  const std::vector<Keyframe>& Keyframes() const { return keyframes_; }
  const PointCloud& LocalCloud() const { return local_cloud_; }
  const std::vector<Landmark>& Landmarks() const { return landmarks_; }

  bool SavePcd(const std::string& path) const;

 private:
  Atla2Config cfg_;
  mutable std::mutex mu_;
  std::vector<Keyframe> keyframes_;
  PointCloud local_cloud_;
  std::vector<Landmark> landmarks_;
  int next_kf_id_ = 0;
};

}  // namespace autonomy::localization::atla2
