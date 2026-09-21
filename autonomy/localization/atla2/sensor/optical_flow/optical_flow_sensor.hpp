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

#include "autonomy/localization/atla2/sensor/interface/sensor_base.hpp"

namespace autonomy::localization::atla2 {

//! Optical-flow / PX4-style flow sensor adapter.
class OpticalFlowSensor : public SensorBase {
 public:
  explicit OpticalFlowSensor(std::string name = "optical_flow");

  SensorKind Kind() const override { return SensorKind::kOpticalFlow; }
  const std::string& Name() const override { return name_; }

  bool Init(const Atla2Config& cfg) override;
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override { return running_; }
  TimeStamp LatestTime() const override;

  //! Scale flow_px → body velocity using height (m) and focal length (px).
  void SetScale(double fx_px, double fy_px) {
    fx_ = fx_px;
    fy_ = fy_px;
  }

  void Push(const OpticalFlowSample& s, double height_m = 0.0);
  bool Pop(OpticalFlowSample* out);

 private:
  std::string name_;
  bool running_ = false;
  double fx_ = 320.0;
  double fy_ = 320.0;
  mutable std::mutex mu_;
  std::deque<OpticalFlowSample> q_;
  TimeStamp latest_ = kInvalidTime;
};

}  // namespace autonomy::localization::atla2
