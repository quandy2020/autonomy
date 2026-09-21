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

#include <cmath>
#include <deque>
#include <mutex>
#include <string>

#include "autonomy/localization/atla2/sensor/interface/sensor_base.hpp"

namespace autonomy::localization::atla2 {

//! Barometer → relative altitude (ISA approximation).
class BarometerSensor : public SensorBase {
 public:
  explicit BarometerSensor(std::string name = "baro");

  SensorKind Kind() const override { return SensorKind::kBarometer; }
  const std::string& Name() const override { return name_; }

  bool Init(const Atla2Config& cfg) override;
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override { return running_; }
  TimeStamp LatestTime() const override;

  void SetPressureRef(double pa) {
    p0_ = pa;
    has_ref_ = true;
  }

  void Push(const BaroSample& s);
  bool Pop(BaroSample* out);

  static double PressureToAltitude(double pressure_pa, double p0_pa);

 private:
  std::string name_;
  bool running_ = false;
  bool has_ref_ = false;
  double p0_ = 101325.0;
  mutable std::mutex mu_;
  std::deque<BaroSample> q_;
  TimeStamp latest_ = kInvalidTime;
};

}  // namespace autonomy::localization::atla2
