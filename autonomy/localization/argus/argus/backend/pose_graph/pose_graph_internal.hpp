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

#ifndef ARGUS_BACKEND_POSE_GRAPH_POSE_GRAPH_INTERNAL_HPP_
#define ARGUS_BACKEND_POSE_GRAPH_POSE_GRAPH_INTERNAL_HPP_

#include <array>
#include <unordered_map>
#include <vector>

#include "argus/common/math/transform/rigid_transform.hpp"

namespace argus {
namespace backend {
namespace pose_graph_detail {

struct CeresEdge {
    int from_id = 0;
    int to_id = 0;
    transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
    double information = 1.0;
};

#if defined(ARGUS_HAS_CERES) && ARGUS_HAS_CERES
bool OptimizeWithCeres(
    const std::unordered_map<int, transform::Rigid3d>& poses_in,
    const std::vector<CeresEdge>& edges, int max_iterations,
    std::unordered_map<int, transform::Rigid3d>* poses_out, double* cost_rms,
    int* iterations);
#endif

#if defined(ARGUS_HAS_GTSAM) && ARGUS_HAS_GTSAM
/** @brief Batch LM on NonlinearFactorGraph (FPR-friendly; not incremental iSAM2). */
bool OptimizeWithGtsam(
    const std::unordered_map<int, transform::Rigid3d>& poses_in,
    const std::vector<CeresEdge>& edges, int max_iterations,
    std::unordered_map<int, transform::Rigid3d>* poses_out, double* cost_rms,
    int* iterations);
#endif

}  // namespace pose_graph_detail
}  // namespace backend
}  // namespace argus

#endif  // ARGUS_BACKEND_POSE_GRAPH_POSE_GRAPH_INTERNAL_HPP_
