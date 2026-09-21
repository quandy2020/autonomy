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

#include "autonomy/localization/atla2/backend/optimizer_base.hpp"

namespace autonomy::localization::atla2 {

//! Iterated EKF-style state smoother over OdometryResult (15-dof: p,v,R,ba,bg).
class IekfOptimizer : public OptimizerBase {
 public:
  bool Init(const Atla2Config& cfg) override;
  bool Update(const OdometryResult& odom) override;
  bool GetState(OdometryResult* out) const override;
  void Reset() override;

 private:
  Atla2Config cfg_;
  OdometryResult state_;
  bool has_state_ = false;
};

}  // namespace autonomy::localization::atla2
