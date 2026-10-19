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
 * @file eval_pose_traj.cpp
 * @brief Offline ATE vs TUM ground-truth trajectories (no bag).
 *
 * Usage:
 *   eval_pose_traj <estimate.tum> <groundtruth.tum> [max_dt_sec]
 */

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "argus/relocalization/pose_eval.hpp"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: eval_pose_traj <estimate.tum> <gt.tum> [max_dt_sec]\n"
              << "  TUM format: t tx ty tz qx qy qz qw\n";
    return 2;
  }
  const std::string est_path = argv[1];
  const std::string gt_path = argv[2];
  const double max_dt =
      (argc >= 4) ? std::atof(argv[3]) : 0.05;

  std::vector<argus::relocalization::TimedPose> est;
  std::vector<argus::relocalization::TimedPose> gt;
  if (!argus::relocalization::LoadTumTrajectory(est_path, &est)) {
    std::cerr << "failed to load estimate: " << est_path << "\n";
    return 1;
  }
  if (!argus::relocalization::LoadTumTrajectory(gt_path, &gt)) {
    std::cerr << "failed to load ground truth: " << gt_path << "\n";
    return 1;
  }
  const auto s =
      argus::relocalization::EvaluateTrajectories(est, gt, max_dt);
  if (!s.ok) {
    std::cerr << "eval failed: " << s.error << "\n";
    return 1;
  }
  std::cout << "matched_pairs=" << s.matched_pairs << "\n"
            << "ate_rmse_m=" << s.ate_rmse_m << "\n"
            << "ate_mean_m=" << s.ate_mean_m << "\n"
            << "ate_max_m=" << s.ate_max_m << "\n"
            << "rot_rmse_rad=" << s.rot_rmse_rad << "\n"
            << "rot_mean_rad=" << s.rot_mean_rad << "\n";
  return 0;
}
