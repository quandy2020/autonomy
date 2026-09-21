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

#include "autonomy/localization/atla2/frontend/imu_preintegration.hpp"
#include "autonomy/localization/atla2/sensor/interface/types.hpp"

namespace autonomy::localization::atla2 {

//! Motion compensation using IMU mid-point poses between t_begin and t_end.
class LidarDeskew {
 public:
  //! deskew each point to scan end time using per-point timestamp (seconds).
  //! If point.timestamp is 0, treat as end-of-scan (no motion).
  LidarScan Compensate(const LidarScan& in, const std::vector<ImuSample>& imu,
                       const Vec3& ba, const Vec3& bg) const;
};

}  // namespace autonomy::localization::atla2
