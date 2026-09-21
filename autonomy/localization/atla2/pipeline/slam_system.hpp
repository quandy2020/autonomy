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

#include <memory>
#include <string>

#include "autonomy/localization/atla2/backend/optimizer_base.hpp"
#include "autonomy/localization/atla2/common/config.hpp"
#include "autonomy/localization/atla2/common/transform_tree.hpp"
#include "autonomy/localization/atla2/frontend/frontend_base.hpp"
#include "autonomy/localization/atla2/fusion/degradation.hpp"
#include "autonomy/localization/atla2/map/map_manager.hpp"
#include "autonomy/localization/atla2/pipeline/state_machine.hpp"
#include "autonomy/localization/atla2/sensor/sync.hpp"

namespace autonomy::localization::atla2 {

//! Top-level SLAM orchestrator: sync → frontend → backend → map.
class SlamSystem {
 public:
  SlamSystem() = default;
  ~SlamSystem() = default;

  bool Init(const Atla2Config& cfg);
  bool InitFromFile(const std::string& platform_yaml);

  void PushImu(const ImuSample& s);
  void PushImage(const ImageFrame& f);
  void PushLidar(const LidarScan& s);

  //! Process one synced packet (or TryPop from internal sync).
  bool Step();
  bool Step(const SensorData& data);

  bool GetOdometry(OdometryResult* out) const;
  SlamState state() const { return sm_.state(); }
  FrontendMode active_mode() const {
    return frontend_ ? frontend_->Mode() : cfg_.mode;
  }
  const HealthReport& last_health() const { return last_health_; }
  MapManager* map() { return &map_; }
  TransformTree* extrinsics() { return &tree_; }
  const Atla2Config& config() const { return cfg_; }

  void Reset();

 private:
  //! Recreate frontend when recommended_mode differs (degradation hot-switch).
  bool MaybeHotSwitchFrontend(FrontendMode recommended);

  Atla2Config cfg_;
  SensorSync sync_;
  std::unique_ptr<FrontendBase> frontend_;
  std::unique_ptr<OptimizerBase> backend_;
  MapManager map_;
  DegradationManager degradation_;
  StateMachine sm_;
  TransformTree tree_;
  OdometryResult last_;
  HealthReport last_health_;
};

}  // namespace autonomy::localization::atla2
