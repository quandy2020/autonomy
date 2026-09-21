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

#include "autonomy/localization/atla2/frontend/livo/livo_frontend.hpp"

namespace autonomy::localization::atla2 {

bool LivoFrontend::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  return vio_.Init(cfg) && lio_.Init(cfg);
}

void LivoFrontend::Reset() {
  vio_.Reset();
  lio_.Reset();
  result_ = OdometryResult{};
  vio_ok_ = false;
  lio_ok_ = false;
}

bool LivoFrontend::Process(const SensorData& data) {
  vio_ok_ = vio_.Process(data);
  lio_ok_ = lio_.Process(data);

  OdometryResult v, l;
  const bool hv = vio_.GetResult(&v);
  const bool hl = lio_.GetResult(&l);

  if (!hv && !hl) {
    result_.valid = false;
    return false;
  }

  if (cfg_.fusion == FusionStyle::kLoose) {
    if (hl && hv) {
      // Prefer lidar translation, visual rotation (loose).
      result_.t = data.t;
      result_.pose = MakeSe3(Se3Translation(l.pose), Se3Rotation(v.pose));
      result_.velocity = 0.5 * (v.velocity + l.velocity);
      result_.gyro_bias = v.gyro_bias;
      result_.accel_bias = v.accel_bias;
      result_.landmarks = v.landmarks;
      result_.local_map = l.local_map;
      result_.cov.matrix = 0.5 * (v.cov.matrix + l.cov.matrix);
      result_.valid = true;
    } else if (hl) {
      result_ = l;
    } else {
      result_ = v;
    }
  } else {
    // Tight coupling placeholder: same as loose until joint residual lands.
    if (hl) {
      result_ = l;
      if (hv) {
        result_.landmarks = v.landmarks;
        result_.pose = MakeSe3(Se3Translation(l.pose), Se3Rotation(v.pose));
      }
    } else {
      result_ = v;
    }
  }
  return result_.valid;
}

bool LivoFrontend::GetResult(OdometryResult* out) {
  if (!out || !result_.valid) {
    return false;
  }
  *out = result_;
  return true;
}

}  // namespace autonomy::localization::atla2
