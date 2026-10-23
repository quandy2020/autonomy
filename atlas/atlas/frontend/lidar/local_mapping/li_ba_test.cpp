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

#include <gtest/gtest.h>

#include <deque>
#include <vector>

#include "frontend/lidar/imu/imu_preintegration.hpp"
#include "frontend/lidar/local_mapping/li_ba_optimizer.hpp"
#include "frontend/lidar/local_mapping/lidar_factor.hpp"
#include "map/voxel/sliding_octree.hpp"

namespace atlas {
namespace vxslam {
namespace {

TEST(LiBaOptimizer, BuildsDampedHessianFromOnePlane) {
  const int window = 3;
  LidarFactor factor(window);
  std::vector<PointCluster> original(static_cast<std::size_t>(window));
  PointCluster sum;
  for (int i = 0; i < window; ++i) {
    for (int k = 0; k < 30; ++k) {
      original[static_cast<std::size_t>(i)].push(
          Eigen::Vector3d(0.1 * k, 0.05 * (k % 5), 0.0));
    }
    sum += original[static_cast<std::size_t>(i)];
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(sum.cov());
  PointCluster fixed;
  double coefficient = 1.0;
  Eigen::Vector3d eigenvalues = solver.eigenvalues();
  Eigen::Matrix3d eigenvectors = solver.eigenvectors();
  factor.push_voxel(original, fixed, coefficient, eigenvalues, eigenvectors,
                    sum);

  std::vector<ImuState> states(static_cast<std::size_t>(window));
  std::deque<ImuPreintegration*> imus;
  for (int i = 0; i < window - 1; ++i) {
    imus.push_back(new ImuPreintegration());
    imus.back()->AddImu(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.8),
                        0.1);
    imus.back()->cov.setIdentity();
    imus.back()->cov *= 1e-3;
  }
  imu_coef = 1e-6;
  LiBaOptimizer optimizer;
  Eigen::MatrixXd hessian;
  optimizer.DampingIter(states, factor, imus, &hessian);
  for (ImuPreintegration* pre : imus) {
    delete pre;
  }
  EXPECT_GE(hessian.rows(), window);
}

}  // namespace
}  // namespace vxslam
}  // namespace atlas
