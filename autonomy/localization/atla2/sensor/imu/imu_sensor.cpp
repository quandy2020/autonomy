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

#include "autonomy/localization/atla2/sensor/imu/imu_sensor.hpp"

namespace autonomy::localization::atla2 {

ImuSensor::ImuSensor(std::string name) : name_(std::move(name)) {}

bool ImuSensor::Init(const Atla2Config& cfg) {
  noise_.acc_n = cfg.imu_acc_noise;
  noise_.gyr_n = cfg.imu_gyr_noise;
  noise_.acc_w = cfg.imu_acc_bias_rw;
  noise_.gyr_w = cfg.imu_gyr_bias_rw;
  return true;
}

bool ImuSensor::Start() {
  running_ = true;
  return true;
}

void ImuSensor::Stop() { running_ = false; }

TimeStamp ImuSensor::LatestTime() const {
  std::lock_guard<std::mutex> lock(mu_);
  return latest_;
}

void ImuSensor::Push(const ImuSample& s) {
  std::lock_guard<std::mutex> lock(mu_);
  q_.push_back(s);
  latest_ = s.t;
  while (q_.size() > 4000) {
    q_.pop_front();
  }
}

std::vector<ImuSample> ImuSensor::PopRange(TimeStamp t0, TimeStamp t1) {
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<ImuSample> out;
  while (!q_.empty() && q_.front().t <= t0 && t0 != kInvalidTime) {
    q_.pop_front();
  }
  while (!q_.empty() && (t1 == kInvalidTime || q_.front().t <= t1)) {
    out.push_back(q_.front());
    q_.pop_front();
  }
  return out;
}

bool ImuSensor::Latest(ImuSample* out) const {
  if (!out) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mu_);
  if (q_.empty()) {
    return false;
  }
  *out = q_.back();
  return true;
}

}  // namespace autonomy::localization::atla2
