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

#include "autonomy/localization/atla2/frontend/vo/vo_frontend.hpp"

namespace autonomy::localization::atla2 {

bool VoFrontend::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  Reset();
  return true;
}

void VoFrontend::Reset() {
  result_ = OdometryResult{};
  initialized_ = false;
  frame_count_ = 0;
}

bool VoFrontend::Process(const SensorData& data) {
  if (!data.has_image || data.image.width <= 0 || data.image.height <= 0) {
    return false;
  }
  result_.t = data.t;
  if (!initialized_) {
    result_.pose = Se3Identity();
    result_.velocity = Vec3::Zero();
    initialized_ = true;
  }
  // Vision update placeholder: identity until the feature tracker lands.
  ++frame_count_;
  result_.valid = true;
  result_.cov.matrix = Mat66::Identity() * 0.05;
  return true;
}

bool VoFrontend::GetResult(OdometryResult* out) {
  if (!out || !result_.valid) {
    return false;
  }
  *out = result_;
  return true;
}

}  // namespace autonomy::localization::atla2
