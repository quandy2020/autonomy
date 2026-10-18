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
#include <cstdlib>
#include <iostream>

#include <Eigen/Core>

#include "argus/fusion/anisotropic_covariance.hpp"
#include "argus/perception/degeneracy_detector.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  using argus::fusion::AnisotropicCovariance;
  using argus::perception::DegeneracyConfig;
  using argus::perception::DegeneracyDetector;

  // Corridor-like FIM: strong Y/Z, weak X.
  Eigen::Matrix3d fim = Eigen::Matrix3d::Zero();
  fim(0, 0) = 0.01;
  fim(1, 1) = 10.0;
  fim(2, 2) = 10.0;

  DegeneracyConfig cfg;
  cfg.eigenvalue_threshold = 0.05;
  cfg.condition_number_threshold = 100.0;
  DegeneracyDetector detector(cfg);
  const auto r = detector.Analyze(fim);
  if (!r.is_degenerated) {
    return Fail("expected is_degenerated=true for weak-X FIM");
  }
  if (std::abs(r.weak_directions.x()) < 0.9) {
    return Fail("weak direction should align with +X");
  }

  Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
  AnisotropicCovariance::InflatePositionBlock(P, Eigen::Vector3d::UnitX(),
                                              5.0);
  if (std::abs(P(0, 0) - 6.0) > 1e-9) {
    return Fail("Pxx should be 1+5");
  }
  if (std::abs(P(1, 1) - 1.0) > 1e-9 || std::abs(P(2, 2) - 1.0) > 1e-9) {
    return Fail("orthogonal variances must stay ~1");
  }

  // Well-conditioned FIM.
  fim.setIdentity();
  fim *= 5.0;
  const auto ok = detector.Analyze(fim);
  if (ok.is_degenerated) {
    return Fail("isotropic FIM should not be is_degenerated");
  }

  std::cout << "OK degeneracy_detector + anisotropic_covariance\n";
  return 0;
}
