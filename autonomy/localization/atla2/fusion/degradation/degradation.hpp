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

#include "autonomy/localization/atla2/common/types.hpp"
#include "autonomy/localization/atla2/frontend/frontend_base.hpp"
#include "autonomy/localization/atla2/sensor/types.hpp"

namespace autonomy::localization::atla2 {

//! Aligned with proto DegradationLevel (incl. LO / VO only).
enum class DegradationLevel {
  kFull = 0,
  kLioOnly,
  kVioOnly,
  kLoOnly,
  kVoOnly,
  kImuCoast,
};

struct HealthReport {
  bool image_ok = true;
  bool lidar_ok = true;
  bool imu_ok = true;
  bool gps_ok = true;
  bool baro_ok = true;
  DegradationLevel level = DegradationLevel::kFull;
  FrontendMode recommended_mode = FrontendMode::kVio;
  double covariance_trace = 0.0;
};

//! Detect sensor faults and recommend degradation path: LIVO→LIO/VIO/LO/VO→IMU.
class DegradationManager {
 public:
  HealthReport Evaluate(const SensorData& data, const OdometryResult& odom) const;
  FrontendMode RecommendedMode(FrontendMode configured, const HealthReport& h) const;
};

}  // namespace autonomy::localization::atla2
