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

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/localization/atla2/sensor/interface/sensor_base.hpp"

namespace autonomy::localization::atla2 {

struct ImuNoiseModel {
  double acc_n = 0.1;
  double gyr_n = 0.01;
  double acc_w = 0.001;
  double gyr_w = 0.0001;
};

class ImuSensor : public SensorBase {
 public:
  explicit ImuSensor(std::string name = "imu");

  SensorKind Kind() const override { return SensorKind::kImu; }
  const std::string& Name() const override { return name_; }

  bool Init(const Atla2Config& cfg) override;
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override { return running_; }
  TimeStamp LatestTime() const override;

  void SetNoise(const ImuNoiseModel& n) { noise_ = n; }
  const ImuNoiseModel& Noise() const { return noise_; }

  void Push(const ImuSample& s);
  //! Samples in (t0, t1], ordered.
  std::vector<ImuSample> PopRange(TimeStamp t0, TimeStamp t1);
  bool Latest(ImuSample* out) const;

 private:
  std::string name_;
  ImuNoiseModel noise_;
  bool running_ = false;
  mutable std::mutex mu_;
  std::deque<ImuSample> q_;
  TimeStamp latest_ = kInvalidTime;
};

}  // namespace autonomy::localization::atla2
