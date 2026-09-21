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

#include "autonomy/localization/atla2/sensor/sync/sensor_sync.hpp"

#include <cstdlib>

namespace autonomy::localization::atla2 {

SensorSync::SensorSync(TimeStamp tol_ns) : tol_ns_(tol_ns) {}

void SensorSync::PushImu(const ImuSample& s) {
  std::lock_guard<std::mutex> lock(mu_);
  imu_.push_back(s);
  while (imu_.size() > 2000) {
    imu_.pop_front();
  }
}

void SensorSync::PushImage(const ImageFrame& f) {
  std::lock_guard<std::mutex> lock(mu_);
  images_.push_back(f);
  while (images_.size() > 30) {
    images_.pop_front();
  }
}

void SensorSync::PushLidar(const LidarScan& s) {
  std::lock_guard<std::mutex> lock(mu_);
  lidars_.push_back(s);
  while (lidars_.size() > 20) {
    lidars_.pop_front();
  }
}

void SensorSync::PushGps(const GpsSample& s) {
  std::lock_guard<std::mutex> lock(mu_);
  gps_.push_back(s);
  while (gps_.size() > 50) {
    gps_.pop_front();
  }
}

void SensorSync::PushBaro(const BaroSample& s) {
  std::lock_guard<std::mutex> lock(mu_);
  baro_.push_back(s);
  while (baro_.size() > 100) {
    baro_.pop_front();
  }
}

void SensorSync::PushOpticalFlow(const OpticalFlowSample& s) {
  std::lock_guard<std::mutex> lock(mu_);
  flow_.push_back(s);
  while (flow_.size() > 100) {
    flow_.pop_front();
  }
}

void SensorSync::Clear() {
  std::lock_guard<std::mutex> lock(mu_);
  imu_.clear();
  images_.clear();
  lidars_.clear();
  gps_.clear();
  baro_.clear();
  flow_.clear();
}

bool SensorSync::TryPop(SensorData* out) {
  if (!out) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mu_);

  TimeStamp anchor = kInvalidTime;
  bool use_image = false;
  if (!images_.empty()) {
    anchor = images_.front().t;
    use_image = true;
  } else if (!lidars_.empty()) {
    anchor = lidars_.front().t;
  } else {
    return false;
  }

  SensorData pkt;
  pkt.t = anchor;

  if (use_image) {
    pkt.image = images_.front();
    pkt.has_image = true;
    images_.pop_front();
  } else {
    pkt.lidar = lidars_.front();
    pkt.has_lidar = true;
    lidars_.pop_front();
  }

  const TimeStamp t0 = anchor - SecToStamp(0.1);
  while (!imu_.empty() && imu_.front().t < t0) {
    imu_.pop_front();
  }
  for (const auto& s : imu_) {
    if (s.t > anchor) {
      break;
    }
    pkt.imu.push_back(s);
  }
  pkt.has_imu = !pkt.imu.empty();

  if (use_image && !lidars_.empty() && WithinTol(lidars_.front().t, anchor, tol_ns_)) {
    pkt.lidar = lidars_.front();
    pkt.has_lidar = true;
    lidars_.pop_front();
  }

  auto attach_nearest = [&](auto& q, auto* dst, bool* flag) {
    if (q.empty()) {
      return;
    }
    // pick closest in window
    size_t best = 0;
    TimeStamp best_d = std::llabs(q.front().t - anchor);
    for (size_t i = 1; i < q.size(); ++i) {
      const TimeStamp d = std::llabs(q[i].t - anchor);
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    if (best_d <= tol_ns_) {
      *dst = q[best];
      *flag = true;
      q.erase(q.begin() + static_cast<std::ptrdiff_t>(best));
    }
  };

  attach_nearest(gps_, &pkt.gps, &pkt.has_gps);
  attach_nearest(baro_, &pkt.baro, &pkt.has_baro);
  attach_nearest(flow_, &pkt.optical_flow, &pkt.has_optical_flow);

  *out = std::move(pkt);
  return true;
}

}  // namespace autonomy::localization::atla2
