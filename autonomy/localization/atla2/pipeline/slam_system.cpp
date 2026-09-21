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

#include "autonomy/localization/atla2/pipeline/slam_system.hpp"

#include "autonomy/localization/atla2/backend/optimizer_base.hpp"
#include "autonomy/localization/atla2/common/time.hpp"
#include "autonomy/localization/atla2/frontend/factory.hpp"

namespace autonomy::localization::atla2 {

bool SlamSystem::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  sync_.SetTol(SecToStamp(cfg_.sync_tol_ms * 1e-3));
  frontend_ = CreateFrontend(cfg_);
  if (!frontend_) {
    return false;
  }
  backend_ = CreateOptimizer(cfg_);
  if (!backend_) {
    return false;
  }
  if (!map_.Init(cfg_)) {
    return false;
  }
  tree_.Clear();
  tree_.Set("body", "imu", Se3Identity());
  tree_.Set("body", "cam", Se3Identity());
  tree_.Set("body", "lidar", Se3Identity());
  last_health_ = HealthReport{};
  sm_.OnInitStart();
  return true;
}

bool SlamSystem::InitFromFile(const std::string& platform_yaml) {
  Atla2Config cfg;
  if (!LoadConfig(platform_yaml, &cfg)) {
    return false;
  }
  return Init(cfg);
}

void SlamSystem::Reset() {
  if (frontend_) {
    frontend_->Reset();
  }
  if (backend_) {
    backend_->Reset();
  }
  map_.Reset();
  sync_.Clear();
  last_ = OdometryResult{};
  last_health_ = HealthReport{};
  sm_.Reset();
}

void SlamSystem::PushImu(const ImuSample& s) { sync_.PushImu(s); }
void SlamSystem::PushImage(const ImageFrame& f) { sync_.PushImage(f); }
void SlamSystem::PushLidar(const LidarScan& s) { sync_.PushLidar(s); }

bool SlamSystem::MaybeHotSwitchFrontend(FrontendMode recommended) {
  if (!cfg_.enable_mode_hot_switch || !frontend_) {
    return false;
  }
  if (recommended == frontend_->Mode()) {
    return false;
  }
  // IMU-coast: keep current frontend; state machine already marks Degraded.
  if (last_health_.level == DegradationLevel::kImuCoast) {
    return false;
  }

  Atla2Config switched = cfg_;
  switched.mode = recommended;
  auto fe = CreateFrontend(switched);
  if (!fe) {
    return false;
  }
  frontend_ = std::move(fe);
  cfg_.mode = recommended;
  sm_.OnDegraded();
  return true;
}

bool SlamSystem::Step() {
  SensorData data;
  if (!sync_.TryPop(&data)) {
    return false;
  }
  return Step(data);
}

bool SlamSystem::Step(const SensorData& data) {
  if (!frontend_ || !backend_) {
    return false;
  }
  if (!frontend_->Process(data)) {
    sm_.OnLost();
    return false;
  }
  OdometryResult odom;
  if (!frontend_->GetResult(&odom)) {
    sm_.OnLost();
    return false;
  }

  last_health_ = degradation_.Evaluate(data, odom);
  last_health_.recommended_mode =
      degradation_.RecommendedMode(cfg_.mode, last_health_);

  if (last_health_.level == DegradationLevel::kImuCoast) {
    sm_.OnDegraded();
  } else if (last_health_.level != DegradationLevel::kFull) {
    sm_.OnDegraded();
    MaybeHotSwitchFrontend(last_health_.recommended_mode);
  }

  if (!backend_->UpdateWithSensors(odom, data)) {
    return false;
  }
  backend_->GetState(&last_);
  map_.UpdateFromOdometry(last_);
  if (sm_.state() == SlamState::kInitializing ||
      sm_.state() == SlamState::kUninitialized) {
    sm_.OnInitSuccess();
  } else if (sm_.state() != SlamState::kDegraded) {
    sm_.OnTrackingOk();
  }
  return true;
}

bool SlamSystem::GetOdometry(OdometryResult* out) const {
  if (!out || !last_.valid) {
    return false;
  }
  *out = last_;
  return true;
}

}  // namespace autonomy::localization::atla2
