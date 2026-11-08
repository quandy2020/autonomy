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

#ifndef ATLAS_BACKEND_POSE_GRAPH_BETWEEN_FACTOR_HPP_
#define ATLAS_BACKEND_POSE_GRAPH_BETWEEN_FACTOR_HPP_

#include <array>
#include <cstdint>

#include "common/transform/rigid_transform.hpp"

namespace atlas {
namespace backend {
namespace pose_graph {

struct BetweenFactor {
  std::int64_t from_id = -1;
  std::int64_t to_id = -1;
  transform::Rigid3d relative_pose = transform::Rigid3d::Identity();
  double rotation_weight = 1.0;
  double translation_weight = 1.0;
  double scale = 1.0;
  bool similarity_constraint = false;
  // Optional diagonal information in [rotation xyz, translation xyz] order.
  // Entries are inverse variances in the tangent units used by the residual.
  std::array<double, 6> diagonal_information{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  bool diagonal_information_provided = false;
};

}  // namespace pose_graph
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_POSE_GRAPH_BETWEEN_FACTOR_HPP_
