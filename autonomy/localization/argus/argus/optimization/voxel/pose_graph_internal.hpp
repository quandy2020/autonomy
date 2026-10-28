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
 * @file pose_graph_internal.hpp
 * @brief Shared relative-edge type for GTSAM / Gauss–Newton solvers.
 */

#ifndef ARGUS_BACKEND_POSE_GRAPH_POSE_GRAPH_INTERNAL_HPP_
#define ARGUS_BACKEND_POSE_GRAPH_POSE_GRAPH_INTERNAL_HPP_

#include <Eigen/Core>
#include <unordered_map>
#include <vector>

#include "common/transform/rigid_transform.hpp"

namespace argus {
namespace vxslam {
namespace backend {
namespace pose_graph_detail {

/**
 * @brief Relative pose edge consumed by GTSAM free-function solvers.
 * @var from_id     Source node identifier.
 * @var to_id       Target node identifier.
 * @var T_i_j       Measured pose of @c to_id in the frame of @c from_id.
 * @var information Scalar information weight.
 */
struct RelativeEdge {
  int from_id = 0;
  int to_id = 0;
  transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
  double information = 1.0;
  Eigen::Matrix<double, 6, 1> variances = Eigen::Matrix<double, 6, 1>::Ones();
  bool use_variances = false;
};

}  // namespace pose_graph_detail
}  // namespace backend
}  // namespace vxslam
}  // namespace argus

#endif  // ARGUS_BACKEND_POSE_GRAPH_POSE_GRAPH_INTERNAL_HPP_
