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

/**
 * @file loop_factor.hpp
 * @brief Loop-closure relative-pose factor and SE(3) residual helper.
 */

#ifndef ARGUS_POSE_GRAPH_OPTIMIZATION_LOOP_FACTOR_HPP_
#define ARGUS_POSE_GRAPH_OPTIMIZATION_LOOP_FACTOR_HPP_

#include "common/transform/rigid_transform.hpp"

namespace argus {
namespace vxslam {
namespace backend {

/**
 * @brief Loop-closure constraint between two pose nodes.
 * @var from_id     Source node identifier.
 * @var to_id       Target node identifier.
 * @var T_i_j       Measured pose of @c to_id in the frame of @c from_id.
 * @var information Scalar information weight (default 100).
 */
struct LoopFactor {
  int from_id = 0;
  int to_id = 0;
  transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
  double information = 100.0;
};

/**
 * @brief SE(3) log residual ‖Log(T_meas⁻¹ · T_est)‖ for one loop factor.
 * @param[in] T_w_from World pose of the @c from_id node.
 * @param[in] T_w_to   World pose of the @c to_id node.
 * @param[in] factor   Measured relative pose @c T_from_to and weight.
 * @return Scalar residual norm (translation metres mixed with rotation rad).
 */
double LoopFactorResidual(const transform::Rigid3d& T_w_from,
                          const transform::Rigid3d& T_w_to,
                          const LoopFactor& factor);

}  // namespace backend
}  // namespace vxslam
}  // namespace argus

#endif  // ARGUS_POSE_GRAPH_OPTIMIZATION_LOOP_FACTOR_HPP_
