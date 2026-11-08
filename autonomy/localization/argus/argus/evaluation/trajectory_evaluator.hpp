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
 * @file trajectory_evaluator.hpp
 * @brief Offline absolute trajectory error (ATE) versus ground truth.
 */

#ifndef ARGUS_EVALUATION_TRAJECTORY_EVALUATOR_HPP_
#define ARGUS_EVALUATION_TRAJECTORY_EVALUATOR_HPP_

#include <string>
#include <vector>

#include "argus/evaluation/tum_format_writer.hpp"

namespace argus {
namespace evaluation {

/**
 * @brief Aggregate ATE / rotation errors after nearest-neighbor time
 *        association.
 * @var matched_pairs Number of estimate poses paired within @c max_dt_sec.
 * @var ate_rmse_m    Root-mean-square translation error (metres).
 * @var ate_mean_m    Mean translation error (metres).
 * @var ate_max_m     Maximum translation error (metres).
 * @var rot_rmse_rad  Root-mean-square rotation angle error (radians).
 * @var rot_mean_rad  Mean rotation angle error (radians).
 * @var ok            True when metrics were computed successfully.
 * @var error         Human-readable failure reason when @c ok is false.
 */
struct TrajectoryEvalSummary {
    int matched_pairs = 0;
    double ate_rmse_m = 0.0;
    double ate_mean_m = 0.0;
    double ate_max_m = 0.0;
    double rot_rmse_rad = 0.0;
    double rot_mean_rad = 0.0;
    bool ok = false;
    std::string error;
};

/**
 * @brief Nearest-neighbor time association then ATE / rotation RMSE.
 * For each estimate pose, picks the closest ground-truth stamp and rejects
 * pairs with absolute time difference larger than @p max_dt_sec. Translation
 * error is `‖t‖` of `T_gt^{-1} T_est`; rotation error is the angle of that
 * relative transform.
 * @param[in] estimate      Estimated trajectory (need not be sorted).
 * @param[in] ground_truth  Ground-truth trajectory (scanned forward in time).
 * @param[in] max_dt_sec    Maximum |t_est - t_gt| in seconds to accept a pair
 *                          (default 0.05).
 * @return Filled @ref TrajectoryEvalSummary. @c ok is false and @c error is
 *         set when either trajectory is empty or no pairs survive the time
 *         gate.
 */
TrajectoryEvalSummary EvaluateTrajectories(
    const std::vector<TimedPose>& estimate,
    const std::vector<TimedPose>& ground_truth, double max_dt_sec = 0.05);

}  // namespace evaluation

/** @brief Compatibility aliases for apps / matching docs. */
namespace relocalization {
using TrajectoryEvalSummary = ::argus::evaluation::TrajectoryEvalSummary;
using ::argus::evaluation::EvaluateTrajectories;
}  // namespace relocalization

}  // namespace argus

#endif  // ARGUS_EVALUATION_TRAJECTORY_EVALUATOR_HPP_
