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

#include <cmath>
#include <iostream>

#include "argus/fusion/confidence_gating.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  using argus::fusion::ConfidenceGating;
  using argus::fusion::ConfidenceGatingConfig;
  using argus::perception::DegeneracyResult;
  using argus::perception::SensorHealth;

  ConfidenceGatingConfig config;
  config.min_lidar_weight = 0.2;
  config.min_inv_condition = 0.3;  // skip if 1/κ < 0.3
  config.max_dynamic_ratio_lidar = 0.5;
  ConfidenceGating gate(config);

  SensorHealth ok;
  ok.lidar_healthy = true;
  ok.camera_healthy = true;

  DegeneracyResult healthy;
  healthy.is_degenerated = false;
  healthy.condition_number = 2.0;
  auto healthy_obs = gate.Evaluate(healthy, ok, 0.0, 0.0);
  if (!healthy_obs.lidar_valid ||
      std::abs(healthy_obs.lidar_weight - 1.0) > 1e-9) {
    return Fail("healthy case should keep full lidar weight");
  }

  DegeneracyResult mild;
  mild.is_degenerated = true;
  mild.condition_number = 2.5;  // inv_condition=0.4 > 0.3 → valid, weighted
  mild.weak_directions = Eigen::Vector3d::UnitX();
  auto mild_obs = gate.Evaluate(mild, ok, 0.0, 0.0);
  if (!mild_obs.lidar_valid) {
    return Fail("mild degeneracy should stay valid");
  }
  if (mild_obs.lidar_weight < config.min_lidar_weight - 1e-9) {
    return Fail("weight floor");
  }

  DegeneracyResult severe;
  severe.is_degenerated = true;
  severe.condition_number = 100.0;  // inv_condition=0.01 < 0.3 → skip
  severe.weak_directions = Eigen::Vector3d::UnitY();
  auto severe_obs = gate.Evaluate(severe, ok, 0.0, 0.0);
  if (severe_obs.lidar_valid || severe_obs.lidar_weight > 0.0) {
    return Fail("severe degeneracy should skip lidar");
  }

  SensorHealth bad_lidar = ok;
  bad_lidar.lidar_healthy = false;
  auto unhealthy_obs = gate.Evaluate(healthy, bad_lidar, 0.0, 0.0);
  if (unhealthy_obs.lidar_valid) {
    return Fail("unhealthy lidar must be invalid");
  }

  auto dynamic_obs = gate.Evaluate(healthy, ok, 0.9, 0.0);
  if (dynamic_obs.lidar_weight > config.min_lidar_weight + 1e-9) {
    return Fail("high dynamic ratio should clamp lidar weight");
  }

  std::cout << "OK confidence_gating\n";
  return 0;
}
