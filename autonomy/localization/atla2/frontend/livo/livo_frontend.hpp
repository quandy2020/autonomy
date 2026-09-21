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

#include "autonomy/localization/atla2/frontend/frontend_base.hpp"
#include "autonomy/localization/atla2/frontend/lio/lio_frontend.hpp"
#include "autonomy/localization/atla2/frontend/vio/vio_frontend.hpp"

namespace autonomy::localization::atla2 {

//! Loose-coupled LIVO: run VIO + LIO, fuse poses (prefer LIO translation, VIO attitude).
class LivoFrontend : public FrontendBase {
 public:
  bool Init(const Atla2Config& cfg) override;
  bool Process(const SensorData& data) override;
  bool GetResult(OdometryResult* out) override;
  void Reset() override;
  FrontendMode Mode() const override { return FrontendMode::kLivo; }

 private:
  Atla2Config cfg_;
  VioFrontend vio_;
  LioFrontend lio_;
  OdometryResult result_;
  bool vio_ok_ = false;
  bool lio_ok_ = false;
};

}  // namespace autonomy::localization::atla2
