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

#include "frontend/lidar/imu/imu_preintegration.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <deque>

namespace argus {
namespace vxslam {
namespace {

TEST(ImuPreintegration, MidPointIntegrationProducesFiniteCost) {
  g_noise_meas.setIdentity();
  g_noise_meas *= 1e-4;
  g_noise_walk.setIdentity();
  g_noise_walk *= 1e-6;

  ImuPreintegration pre;
  std::deque<sensor::ImuData> imus;
  for (int i = 0; i < 20; ++i) {
    sensor::ImuData imu;
    imu.time = 0.01 * i;
    imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.8);
    imu.angular_velocity = Eigen::Vector3d(0.01, 0.0, 0.0);
    imus.push_back(imu);
  }
  pre.PushImu(imus);
  EXPECT_GT(pre.dtime, 0.0);

  ImuState a;
  ImuState b = a;
  b.t = pre.dtime;
  b.p = pre.p_delta;
  b.v = pre.v_delta;
  b.R = pre.R_delta;
  Eigen::MatrixXd jtj;
  Eigen::VectorXd gg;
  const double cost = pre.Evaluate(a, b, jtj, gg, true);
  EXPECT_TRUE(std::isfinite(cost));
  EXPECT_EQ(jtj.rows(), 30);
  EXPECT_EQ(jtj.cols(), 30);
}

TEST(ImuPreintegration, MergeCombinesDeltas) {
  ImuPreintegration first;
  ImuPreintegration second;
  first.dtime = 0.5;
  second.dtime = 0.25;
  first.p_delta = Eigen::Vector3d(1.0, 0.0, 0.0);
  second.R_delta = math::Exp(Eigen::Vector3d(0.0, 0.0, 0.1));
  first.Merge(second);
  EXPECT_NEAR(first.dtime, 0.75, 1e-12);
}

}  // namespace
}  // namespace vxslam
}  // namespace argus
