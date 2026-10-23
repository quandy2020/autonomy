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
 * @file gtsam_optimizer.hpp
 * @brief GTSAM ISAM2 batch pose-graph optimizer and free-function entry.
 */

#ifndef ATLAS_POSE_GRAPH_OPTIMIZATION_GTSAM_OPTIMIZER_HPP_
#define ATLAS_POSE_GRAPH_OPTIMIZATION_GTSAM_OPTIMIZER_HPP_

#include "optimization/voxel/optimization.hpp"
#include "optimization/voxel/pose_graph_internal.hpp"

namespace atlas {
namespace vxslam {
namespace backend {

/**
 * @brief GTSAM ISAM2 batch solve (loop-gate friendly; not incremental across
 *        calls).
 * Built only when @c ATLAS_HAS_GTSAM=1. Without GTSAM, @ref Solve returns
 * @c ok=false.
 */
class GtsamOptimizer : public Optimizer {
 public:
  /**
   * @brief Solve one batch problem via ISAM2 (or fail if GTSAM is absent).
   * @param[in] problem Initial poses, edges, and iteration limit.
   * @return Filled @ref OptimizationResult; @c ok is false when GTSAM is
   *         unavailable or the graph is too small.
   */
  OptimizationResult Solve(const OptimizationProblem& problem) override;
};

namespace pose_graph_detail {

#if defined(ATLAS_HAS_GTSAM) && ATLAS_HAS_GTSAM
/**
 * @brief Free-function GTSAM entry used by @ref PoseGraph::OptimizeUnlocked.
 * @param[in]  poses_in        Initial world-frame body poses.
 * @param[in]  edges           Relative constraints.
 * @param[in]  max_iterations  Extra ISAM2 update passes after the first.
 * @param[out] poses_out       Optimized poses; must be non-null.
 * @param[out] root_mean_cost  Optional root-mean residual cost; may be null.
 * @param[out] iterations      Optional iteration count written; may be null.
 * @return True when the solve finished with a finite cost.
 */
bool OptimizeWithGtsam(
    const std::unordered_map<int, transform::Rigid3d>& poses_in,
    const std::vector<RelativeEdge>& edges, int max_iterations,
    std::unordered_map<int, transform::Rigid3d>* poses_out,
    double* root_mean_cost, int* iterations);
#endif

}  // namespace pose_graph_detail
}  // namespace backend
}  // namespace vxslam
}  // namespace atlas

#endif  // ATLAS_POSE_GRAPH_OPTIMIZATION_GTSAM_OPTIMIZER_HPP_
