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

#include "autonomy/localization/atla2/sensor/barometer/barometer_sensor.hpp"

namespace autonomy::localization::atla2 {

BarometerSensor::BarometerSensor(std::string name) : name_(std::move(name)) {}

bool BarometerSensor::Init(const Atla2Config& cfg) {
  (void)cfg;
  return true;
}

bool BarometerSensor::Start() {
  running_ = true;
  return true;
}

void BarometerSensor::Stop() { running_ = false; }

TimeStamp BarometerSensor::LatestTime() const {
  std::lock_guard<std::mutex> lock(mu_);
  return latest_;
}

double BarometerSensor::PressureToAltitude(double pressure_pa, double p0_pa) {
  // ISA troposphere approximation
  return 44330.0 * (1.0 - std::pow(pressure_pa / p0_pa, 0.1903));
}

void BarometerSensor::Push(const BaroSample& s) {
  BaroSample c = s;
  if (!has_ref_ && c.pressure_pa > 0.0) {
    SetPressureRef(c.pressure_pa);
  }
  if (c.pressure_pa > 0.0) {
    c.altitude_m = PressureToAltitude(c.pressure_pa, p0_);
    c.valid = true;
  }
  std::lock_guard<std::mutex> lock(mu_);
  q_.push_back(c);
  latest_ = c.t;
  while (q_.size() > 100) {
    q_.pop_front();
  }
}

bool BarometerSensor::Pop(BaroSample* out) {
  if (!out) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mu_);
  if (q_.empty()) {
    return false;
  }
  *out = q_.front();
  q_.pop_front();
  return true;
}

}  // namespace autonomy::localization::atla2
