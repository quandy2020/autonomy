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

#include "autonomy/localization/atla2/sensor/gps/gps_sensor.hpp"

namespace autonomy::localization::atla2 {

namespace {
constexpr double kDeg2Rad = 0.017453292519943295;
constexpr double kWgs84A = 6378137.0;
}  // namespace

GpsSensor::GpsSensor(std::string name) : name_(std::move(name)) {}

bool GpsSensor::Init(const Atla2Config& cfg) {
  (void)cfg;
  return true;
}

bool GpsSensor::Start() {
  running_ = true;
  return true;
}

void GpsSensor::Stop() { running_ = false; }

TimeStamp GpsSensor::LatestTime() const {
  std::lock_guard<std::mutex> lock(mu_);
  return latest_;
}

void GpsSensor::SetOrigin(double lat_deg, double lon_deg, double alt_m) {
  lat0_ = lat_deg;
  lon0_ = lon_deg;
  alt0_ = alt_m;
  has_origin_ = true;
}

Vec3 GpsSensor::LlaToEnu(double lat_deg, double lon_deg, double alt_m, double lat0_deg,
                         double lon0_deg, double alt0_m) {
  const double lat = lat_deg * kDeg2Rad;
  const double lon = lon_deg * kDeg2Rad;
  const double lat0 = lat0_deg * kDeg2Rad;
  const double lon0 = lon0_deg * kDeg2Rad;
  const double dlat = lat - lat0;
  const double dlon = lon - lon0;
  const double e = dlon * std::cos(lat0) * kWgs84A;
  const double n = dlat * kWgs84A;
  const double u = alt_m - alt0_m;
  return Vec3(e, n, u);
}

void GpsSensor::Push(const GpsSample& s) {
  GpsSample c = s;
  if (has_origin_ && c.valid) {
    c.position_enu = LlaToEnu(c.lla.x(), c.lla.y(), c.lla.z(), lat0_, lon0_, alt0_);
  } else if (c.valid && !has_origin_) {
    SetOrigin(c.lla.x(), c.lla.y(), c.lla.z());
    c.position_enu = Vec3::Zero();
  }
  std::lock_guard<std::mutex> lock(mu_);
  q_.push_back(c);
  latest_ = c.t;
  while (q_.size() > 50) {
    q_.pop_front();
  }
}

bool GpsSensor::Pop(GpsSample* out) {
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
