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

#include "autonomy/localization/atla2/frontend/frontend_base.hpp"

namespace autonomy::localization::atla2 {

//! Visual odometry: image only, no IMU. Pose stays until a tracker updates it.
class VoFrontend : public FrontendBase {
 public:
  bool Init(const Atla2Config& cfg) override;
  bool Process(const SensorData& data) override;
  bool GetResult(OdometryResult* out) override;
  void Reset() override;
  FrontendMode Mode() const override { return FrontendMode::kVo; }

 private:
  Atla2Config cfg_;
  OdometryResult result_;
  bool initialized_ = false;
  int frame_count_ = 0;
};

}  // namespace autonomy::localization::atla2
