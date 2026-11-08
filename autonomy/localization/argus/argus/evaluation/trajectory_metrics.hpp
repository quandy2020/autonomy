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
 * @file trajectory_metrics.hpp
 * @brief Convenience alias for trajectory ATE / rotation metrics.
 */

#ifndef ARGUS_EVALUATION_TRAJECTORY_METRICS_HPP_
#define ARGUS_EVALUATION_TRAJECTORY_METRICS_HPP_

#include <vector>

#include "argus/evaluation/trajectory_evaluator.hpp"

namespace argus {
namespace evaluation {

/**
 * @brief Preferred name for a trajectory ATE / rotation summary.
 * Identical to @ref TrajectoryEvalSummary.
 */
using TrajectoryMetrics = TrajectoryEvalSummary;

/**
 * @brief Compute trajectory metrics (ATE / rotation) versus ground truth.
 * Thin wrapper around @ref EvaluateTrajectories for apps / benchmark.
 * @param[in] estimate      Estimated trajectory.
 * @param[in] ground_truth  Ground-truth trajectory.
 * @param[in] max_dt_sec    Maximum |t_est - t_gt| in seconds for pairing
 *                          (default 0.05).
 * @return Same result as @ref EvaluateTrajectories for the given arguments.
 */
TrajectoryMetrics ComputeTrajectoryMetrics(
    const std::vector<TimedPose>& estimate,
    const std::vector<TimedPose>& ground_truth, double max_dt_sec = 0.05);

}  // namespace evaluation
}  // namespace argus

#endif  // ARGUS_EVALUATION_TRAJECTORY_METRICS_HPP_
