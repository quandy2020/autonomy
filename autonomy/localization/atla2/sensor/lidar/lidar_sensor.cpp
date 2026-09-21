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

#include "autonomy/localization/atla2/sensor/lidar/lidar_sensor.hpp"

namespace autonomy::localization::atla2 {

LidarSensor::LidarSensor(std::string name, LidarModel model)
    : name_(std::move(name)), driver_(CreateLidarDriver(model)) {
  driver_->SetCallback([this](const LidarScan& s) { OnDriverScan(s); });
}

bool LidarSensor::Init(const Atla2Config& cfg) {
  LidarPreprocessOptions po;
  po.voxel_size = static_cast<float>(cfg.voxel_size);
  preprocess_.SetOptions(po);
  return driver_ != nullptr;
}

bool LidarSensor::Start() {
  running_ = true;
  return driver_ && driver_->Open("");
}

void LidarSensor::Stop() {
  running_ = false;
  if (driver_) {
    driver_->Close();
  }
}

TimeStamp LidarSensor::LatestTime() const {
  std::lock_guard<std::mutex> lock(mu_);
  return latest_;
}

void LidarSensor::FeedRaw(const LidarScan& raw, const std::vector<ImuSample>& imu,
                          const Vec3& ba, const Vec3& bg) {
  {
    std::lock_guard<std::mutex> lock(mu_);
    last_imu_ = imu;
    ba_ = ba;
    bg_ = bg;
  }
  if (driver_) {
    driver_->Feed(raw);
  } else {
    OnDriverScan(raw);
  }
}

void LidarSensor::OnDriverScan(const LidarScan& raw) {
  LidarScan s = preprocess_.Process(raw);
  std::vector<ImuSample> imu;
  Vec3 ba, bg;
  {
    std::lock_guard<std::mutex> lock(mu_);
    imu = last_imu_;
    ba = ba_;
    bg = bg_;
  }
  if (enable_deskew_ && !imu.empty()) {
    s = deskew_.Compensate(s, imu, ba, bg);
  }
  if (enable_filter_) {
    s = filter_.Process(s);
  }
  std::lock_guard<std::mutex> lock(mu_);
  q_.push_back(std::move(s));
  latest_ = q_.back().t;
  while (q_.size() > 10) {
    q_.pop_front();
  }
}

bool LidarSensor::Pop(LidarScan* out) {
  if (!out) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mu_);
  if (q_.empty()) {
    return false;
  }
  *out = std::move(q_.front());
  q_.pop_front();
  return true;
}

}  // namespace autonomy::localization::atla2
