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
 * @file pose_trajectory_eval.hpp
 * @brief Offline trajectory ATE vs ground truth (TUM format; no bag).
 */

#ifndef ARGUS_RELOCALIZATION_POSE_TRAJECTORY_EVAL_HPP_
#define ARGUS_RELOCALIZATION_POSE_TRAJECTORY_EVAL_HPP_

#include <string>
#include <vector>

#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/relocalization/relocalization_base.hpp"

namespace argus {
namespace relocalization {

struct TimedPose {
    double stamp_sec = 0.0;
    transform::Rigid3d T_w_b = transform::Rigid3d::Identity();
};

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
 * @brief Load TUM trajectory: `t tx ty tz qx qy qz qw` (one pose per line).
 * @return false on empty / unreadable file.
 */
bool LoadTumTrajectory(const std::string& path, std::vector<TimedPose>* out);

/**
 * @brief Nearest-neighbor time association + ATE / rotation RMSE.
 * @param max_dt_sec  Reject pairs with |t_est - t_gt| larger than this.
 */
TrajectoryEvalSummary EvaluateTrajectories(const std::vector<TimedPose>& estimate,
                                     const std::vector<TimedPose>& ground_truth,
                                     double max_dt_sec = 0.05);

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_POSE_TRAJECTORY_EVAL_HPP_
