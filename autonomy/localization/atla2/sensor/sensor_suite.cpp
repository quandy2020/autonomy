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

#include "autonomy/localization/atla2/sensor/sensor_suite.hpp"

#include "autonomy/localization/atla2/common/time.hpp"

namespace autonomy::localization::atla2 {

bool SensorSuite::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  sync_.SetTol(SecToStamp(cfg_.sync_tol_ms * 1e-3));
  sync_.SetExtrinsics(&tree_);
  tree_.Clear();
  tree_.Set("body", "imu", Se3Identity());
  tree_.Set("body", "cam", Se3Identity());
  tree_.Set("body", "lidar", Se3Identity());

  all_.clear();
  camera_.reset();
  imu_.reset();
  lidar_.reset();
  gps_.reset();
  baro_.reset();
  flow_.reset();

  if (cfg_.vo_enabled || cfg_.vio_enabled) {
    camera_ = std::make_unique<CameraSensor>("cam0");
    camera_->Init(cfg_);
    all_.push_back(camera_.get());
  }
  if (cfg_.vio_enabled || cfg_.lio_enabled) {
    imu_ = std::make_unique<ImuSensor>("imu0");
    imu_->Init(cfg_);
    all_.push_back(imu_.get());
  }
  if (cfg_.lio_enabled) {
    lidar_ = std::make_unique<LidarSensor>("lidar0", LidarModel::kLivox);
    lidar_->Init(cfg_);
    all_.push_back(lidar_.get());
  }
  if (cfg_.gps_enabled) {
    gps_ = std::make_unique<GpsSensor>("gps0");
    gps_->Init(cfg_);
    all_.push_back(gps_.get());
  }
  // Optional sensors: create stubs so callers can Push when available.
  baro_ = std::make_unique<BarometerSensor>("baro0");
  baro_->Init(cfg_);
  all_.push_back(baro_.get());
  flow_ = std::make_unique<OpticalFlowSensor>("flow0");
  flow_->Init(cfg_);
  all_.push_back(flow_.get());

  return true;
}

bool SensorSuite::Start() {
  bool ok = true;
  for (auto* s : all_) {
    ok = s->Start() && ok;
  }
  return ok;
}

void SensorSuite::Stop() {
  for (auto* s : all_) {
    s->Stop();
  }
}

bool SensorSuite::Poll(SensorData* out) {
  if (camera_) {
    ImageFrame img;
    while (camera_->Pop(&img)) {
      sync_.PushImage(img);
    }
  }
  if (imu_) {
    const TimeStamp t1 = imu_->LatestTime();
    if (t1 != kInvalidTime) {
      auto batch = imu_->PopRange(kInvalidTime, t1);
      for (const auto& sample : batch) {
        sync_.PushImu(sample);
      }
    }
  }
  if (lidar_) {
    LidarScan scan;
    while (lidar_->Pop(&scan)) {
      sync_.PushLidar(scan);
    }
  }
  if (gps_) {
    GpsSample g;
    while (gps_->Pop(&g)) {
      sync_.PushGps(g);
    }
  }
  if (baro_) {
    BaroSample b;
    while (baro_->Pop(&b)) {
      sync_.PushBaro(b);
    }
  }
  if (flow_) {
    OpticalFlowSample f;
    while (flow_->Pop(&f)) {
      sync_.PushOpticalFlow(f);
    }
  }
  return sync_.TryPop(out);
}

}  // namespace autonomy::localization::atla2
