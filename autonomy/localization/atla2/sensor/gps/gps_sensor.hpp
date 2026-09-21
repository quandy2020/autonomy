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

//! GPS / GNSS adapter. Converts LLA → local ENU once origin is set.
class GpsSensor : public SensorBase {
 public:
  explicit GpsSensor(std::string name = "gps");

  SensorKind Kind() const override { return SensorKind::kGps; }
  const std::string& Name() const override { return name_; }

  bool Init(const Atla2Config& cfg) override;
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override { return running_; }
  TimeStamp LatestTime() const override;

  void SetOrigin(double lat_deg, double lon_deg, double alt_m);
  bool HasOrigin() const { return has_origin_; }

  void Push(const GpsSample& s);
  bool Pop(GpsSample* out);

  //! WGS84 LLA → ENU relative to origin (meters).
  static Vec3 LlaToEnu(double lat_deg, double lon_deg, double alt_m, double lat0_deg,
                       double lon0_deg, double alt0_m);

 private:
  std::string name_;
  bool running_ = false;
  bool has_origin_ = false;
  double lat0_ = 0, lon0_ = 0, alt0_ = 0;
  mutable std::mutex mu_;
  std::deque<GpsSample> q_;
  TimeStamp latest_ = kInvalidTime;
};

}  // namespace autonomy::localization::atla2
