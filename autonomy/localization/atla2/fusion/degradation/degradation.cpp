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

#include "autonomy/localization/atla2/fusion/degradation/degradation.hpp"

namespace autonomy::localization::atla2 {

HealthReport DegradationManager::Evaluate(const SensorData& data,
                                          const OdometryResult& odom) const {
  HealthReport h;
  h.imu_ok = data.has_imu && !data.imu.empty();
  h.image_ok = data.has_image && data.image.width > 0;
  h.lidar_ok = data.has_lidar && !data.lidar.points.empty();
  h.gps_ok = data.has_gps && data.gps.valid;
  h.baro_ok = data.has_baro && data.baro.valid;
  h.covariance_trace = odom.valid ? odom.cov.matrix.trace() : 0.0;

  if (odom.valid && h.covariance_trace > 100.0) {
    h.image_ok = false;
    h.lidar_ok = false;
  }

  if (h.image_ok && h.lidar_ok) {
    h.level = DegradationLevel::kFull;
  } else if (h.lidar_ok && h.imu_ok) {
    h.level = DegradationLevel::kLioOnly;
  } else if (h.lidar_ok) {
    h.level = DegradationLevel::kLoOnly;
  } else if (h.image_ok && h.imu_ok) {
    h.level = DegradationLevel::kVioOnly;
  } else if (h.image_ok) {
    h.level = DegradationLevel::kVoOnly;
  } else {
    h.level = DegradationLevel::kImuCoast;
  }
  return h;
}

FrontendMode DegradationManager::RecommendedMode(FrontendMode configured,
                                                 const HealthReport& h) const {
  auto pick = [&](FrontendMode preferred) {
    return preferred;
  };

  if (configured == FrontendMode::kLivo) {
    if (h.level == DegradationLevel::kFull) {
      return pick(FrontendMode::kLivo);
    }
    if (h.level == DegradationLevel::kLioOnly) {
      return pick(FrontendMode::kLio);
    }
    if (h.level == DegradationLevel::kLoOnly) {
      return pick(FrontendMode::kLo);
    }
    if (h.level == DegradationLevel::kVioOnly) {
      return pick(FrontendMode::kVio);
    }
    if (h.level == DegradationLevel::kVoOnly) {
      return pick(FrontendMode::kVo);
    }
  }
  if (configured == FrontendMode::kLio) {
    if (h.level == DegradationLevel::kLioOnly || h.level == DegradationLevel::kFull) {
      return FrontendMode::kLio;
    }
    if (h.level == DegradationLevel::kLoOnly) {
      return FrontendMode::kLo;
    }
  }
  if (configured == FrontendMode::kLo &&
      (h.level == DegradationLevel::kLoOnly || h.level == DegradationLevel::kLioOnly ||
       h.level == DegradationLevel::kFull)) {
    return FrontendMode::kLo;
  }
  if (configured == FrontendMode::kVio) {
    if (h.level == DegradationLevel::kVioOnly || h.level == DegradationLevel::kFull) {
      return FrontendMode::kVio;
    }
    if (h.level == DegradationLevel::kVoOnly) {
      return FrontendMode::kVo;
    }
  }
  if (configured == FrontendMode::kVo) {
    return FrontendMode::kVo;
  }
  return configured;
}

}  // namespace autonomy::localization::atla2
