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

#include <memory>

#include "autonomy/localization/atla2/common/config.hpp"
#include "autonomy/localization/atla2/frontend/frontend_base.hpp"
#include "autonomy/localization/atla2/sensor/types.hpp"

namespace autonomy::localization::atla2 {

class OptimizerBase {
 public:
  virtual ~OptimizerBase() = default;
  virtual bool Init(const Atla2Config& cfg) = 0;
  //! Fuse frontend odometry into filter / graph state.
  virtual bool Update(const OdometryResult& odom) = 0;
  //! VIO path: also consume IMU / image packet for factors.
  virtual bool UpdateWithSensors(const OdometryResult& odom, const SensorData& data) {
    (void)data;
    return Update(odom);
  }
  virtual bool GetState(OdometryResult* out) const = 0;
  virtual void Reset() = 0;
};

std::unique_ptr<OptimizerBase> CreateOptimizer(const Atla2Config& cfg);

}  // namespace autonomy::localization::atla2
