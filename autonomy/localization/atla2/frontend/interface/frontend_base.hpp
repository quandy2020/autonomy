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

#include <vector>

#include "autonomy/localization/atla2/common/config.hpp"
#include "autonomy/localization/atla2/common/types.hpp"
#include "autonomy/localization/atla2/sensor/types.hpp"

namespace autonomy::localization::atla2 {

struct OdometryResult {
  TimeStamp t = kInvalidTime;
  SE3 pose = Se3Identity();
  Vec3 velocity = Vec3::Zero();
  Vec3 gyro_bias = Vec3::Zero();
  Vec3 accel_bias = Vec3::Zero();
  Covariance6 cov;
  std::vector<Landmark> landmarks;
  PointCloud local_map;
  bool valid = false;
};

class FrontendBase {
 public:
  virtual ~FrontendBase() = default;
  virtual bool Init(const Atla2Config& cfg) = 0;
  virtual bool Process(const SensorData& data) = 0;
  virtual bool GetResult(OdometryResult* out) = 0;
  virtual void Reset() = 0;
  virtual FrontendMode Mode() const = 0;
};

}  // namespace autonomy::localization::atla2
