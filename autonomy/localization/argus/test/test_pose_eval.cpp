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

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "argus/relocalization/pose_trajectory_eval.hpp"
#include "argus/relocalization/prior_map_loader.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

void WriteTum(const std::string& path, double t, double x, double y, double z) {
  std::ofstream os(path, std::ios::app);
  os << t << ' ' << x << ' ' << y << ' ' << z << " 0 0 0 1\n";
}

}  // namespace

int main() {
  using argus::relocalization::EvaluateTrajectories;
  using argus::relocalization::LoadTumTrajectory;
  using argus::relocalization::PriorMapLoader;
  using argus::relocalization::TimedPose;

  const std::string dir = "argus_test_pose_eval_tmp";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const std::string gt = dir + "/gt.txt";
  const std::string est = dir + "/est.txt";
  std::filesystem::remove(gt);
  std::filesystem::remove(est);
  for (int i = 0; i < 10; ++i) {
    WriteTum(gt, 0.1 * i, i * 1.0, 0.0, 0.0);
    WriteTum(est, 0.1 * i + 0.01, i * 1.0 + 0.1, 0.0, 0.0);
  }

  std::vector<TimedPose> gt_traj;
  std::vector<TimedPose> est_traj;
  if (!LoadTumTrajectory(gt, &gt_traj) || gt_traj.size() != 10) {
    return Fail("load gt");
  }
  if (!LoadTumTrajectory(est, &est_traj) || est_traj.size() != 10) {
    return Fail("load est");
  }
  const auto summary = EvaluateTrajectories(est_traj, gt_traj, 0.05);
  if (!summary.ok || summary.matched_pairs != 10) {
    std::cerr << summary.error << std::endl;
    return Fail("eval pairs");
  }
  if (std::abs(summary.ate_mean_m - 0.1) > 1e-6) {
    std::cerr << "mean=" << summary.ate_mean_m << std::endl;
    return Fail("ate mean");
  }
  if (summary.ate_rmse_m < 0.099 || summary.ate_rmse_m > 0.101) {
    std::cerr << "rmse=" << summary.ate_rmse_m << std::endl;
    return Fail("ate rmse");
  }

  // Prior LoadPath: directory with map.xyz
  {
    std::ofstream xyz(dir + "/map.xyz");
    xyz << "0 0 0\n1 0 0\n2 0 0\n";
  }
  auto prior = PriorMapLoader::LoadPath(dir, 0.5);
  if (!prior || prior->size() == 0) {
    return Fail("LoadPath dir");
  }
  auto prior2 = PriorMapLoader::LoadPath(dir + "/map.xyz", 0.5);
  if (!prior2 || prior2->size() == 0) {
    return Fail("LoadPath file");
  }

  std::filesystem::remove_all(dir);
  std::cout << "OK pose_eval (ate_rmse=" << summary.ate_rmse_m
            << " pairs=" << summary.matched_pairs << ")\n";
  return 0;
}
