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

#include "autonomy/localization/atla2/backend/graph/sliding_window.hpp"
#include "autonomy/localization/atla2/backend/optimizer_base.hpp"
#include "autonomy/localization/atla2/frontend/imu_preintegration.hpp"
#include "autonomy/localization/atla2/sensor/types.hpp"

namespace autonomy::localization::atla2 {

//! Ceres sliding-window VIO backend: IMU + relative pose + reprojection.
class GraphOptimizer : public OptimizerBase {
 public:
  bool Init(const Atla2Config& cfg) override;
  bool Update(const OdometryResult& odom) override;
  //! Preferred VIO entry: odometry + IMU (+ optional landmark uv).
  bool UpdateWithSensors(const OdometryResult& odom, const SensorData& data) override;
  bool GetState(OdometryResult* out) const override;
  void Reset() override;

 private:
  bool Optimize();
  WindowFrame MakeFrame(const OdometryResult& odom, const SensorData* data);

  Atla2Config cfg_;
  SlidingWindow window_;
  CameraModel cam_;
  Vec3 gravity_{0.0, 0.0, -9.81};
  OdometryResult state_;
  bool has_state_ = false;
  int max_iterations_ = 15;
  double pose_weight_ = 10.0;
  double imu_weight_ = 1.0;
  double visual_weight_ = 1.0;
};

}  // namespace autonomy::localization::atla2
