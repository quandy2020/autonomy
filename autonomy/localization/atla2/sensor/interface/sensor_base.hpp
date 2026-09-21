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

#include <string>

#include "autonomy/localization/atla2/common/config.hpp"
#include "autonomy/localization/atla2/common/types.hpp"
#include "autonomy/localization/atla2/sensor/interface/types.hpp"

namespace autonomy::localization::atla2 {

//! Sensor driver abstraction (core does not depend on ROS / vendor SDK).
class SensorBase {
 public:
  virtual ~SensorBase() = default;

  virtual SensorKind Kind() const = 0;
  virtual const std::string& Name() const = 0;

  virtual bool Init(const Atla2Config& cfg) = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  virtual bool IsRunning() const = 0;
  virtual TimeStamp LatestTime() const = 0;
};

}  // namespace autonomy::localization::atla2
