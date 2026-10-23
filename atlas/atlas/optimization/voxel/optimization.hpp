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
 * @file optimization.hpp
 * @brief Batch pose-graph optimizer interface and problem / result types.
 */

#ifndef ATLAS_POSE_GRAPH_OPTIMIZATION_OPTIMIZATION_HPP_
#define ATLAS_POSE_GRAPH_OPTIMIZATION_OPTIMIZATION_HPP_

#include <Eigen/Core>
#include <unordered_map>
#include <vector>

#include "common/transform/rigid_transform.hpp"

namespace atlas {
namespace vxslam {
namespace backend {

/**
 * @brief Relative pose constraint for one batch solve.
 * @var from_id     Source node identifier.
 * @var to_id       Target node identifier.
 * @var T_i_j       Measured pose of @c to_id in the frame of @c from_id.
 * @var information Scalar information weight (larger = tighter).
 */
struct PoseEdge {
  int from_id = 0;
  int to_id = 0;
  transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
  double information = 1.0;
  /** @brief Optional diagonal variances in GTSAM Pose3 tangent ordering. */
  Eigen::Matrix<double, 6, 1> variances = Eigen::Matrix<double, 6, 1>::Ones();
  bool use_variances = false;
};

/**
 * @brief Input graph snapshot for one @ref Optimizer::Solve call.
 * @var poses          Initial world-frame body poses keyed by node id.
 * @var edges          Relative constraints (odometry and / or loop).
 * @var max_iterations Solver iteration cap.
 */
struct OptimizationProblem {
  std::unordered_map<int, transform::Rigid3d> poses;
  std::vector<PoseEdge> edges;
  int max_iterations = 10;
};

/**
 * @brief Output of @ref Optimizer::Solve.
 * @var ok             True when the solve finished with a finite cost.
 * @var poses          Optimized world-frame body poses keyed by node id.
 * @var root_mean_cost Root-mean residual cost over active edges.
 * @var iterations     Number of solver iterations performed.
 */
struct OptimizationResult {
  bool ok = false;
  std::unordered_map<int, transform::Rigid3d> poses;
  double root_mean_cost = 0.0;
  int iterations = 0;
};

/**
 * @brief Batch pose-graph optimizer interface (GTSAM / Gauss–Newton).
 * @ref PoseGraph owns the live graph and loop gate; solvers only consume a
 * snapshot @ref OptimizationProblem and return updated poses.
 */
class Optimizer {
 public:
  virtual ~Optimizer() = default;

  /**
   * @brief Solve one batch pose-graph problem.
   * @param[in] problem Initial poses, edges, and iteration limit.
   * @return Filled @ref OptimizationResult; @c ok is false on failure or
   *         when the backend is unavailable.
   */
  virtual OptimizationResult Solve(const OptimizationProblem& problem) = 0;
};

}  // namespace backend
}  // namespace vxslam
}  // namespace atlas

#endif  // ATLAS_POSE_GRAPH_OPTIMIZATION_OPTIMIZATION_HPP_
