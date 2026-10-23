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

#include "optimization/voxel/loop_factor.hpp"

#include <Eigen/Dense>

#include "common/math/so3.hpp"

namespace atlas {
namespace vxslam {
namespace backend {
namespace {

Eigen::Matrix<double, 6, 1> Se3Log(const transform::Rigid3d& T) {
  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = T.translation();
  xi.tail<3>() = math::Log(T.rotation().toRotationMatrix());
  return xi;
}

}  // namespace

double LoopFactorResidual(const transform::Rigid3d& T_w_from,
                          const transform::Rigid3d& T_w_to,
                          const LoopFactor& factor) {
  const transform::Rigid3d T_est = T_w_from.inverse() * T_w_to;
  return Se3Log(factor.T_i_j.inverse() * T_est).norm();
}

}  // namespace backend
}  // namespace vxslam
}  // namespace atlas
