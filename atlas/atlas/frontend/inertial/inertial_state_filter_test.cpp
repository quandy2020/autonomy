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

#include "frontend/inertial/inertial_state_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <deque>

#include "frontend/inertial/inertial_state.hpp"
#include "map/point_cluster.hpp"

namespace atlas {
namespace frontend {
namespace inertial {
namespace {

using ::atlas::map::PointCluster;

sensor::ImuData MakeImu(double time, const Eigen::Vector3d& accel) {
  sensor::ImuData imu;
  imu.time = time;
  imu.linear_acceleration = accel;
  imu.angular_velocity = Eigen::Vector3d::Zero();
  return imu;
}

TEST(InertialStateFilter, InitializesGravityThenUndistorts) {
  InertialStateFilter ekf;
  ekf.min_init_num = 5;
  InertialState state;
  sensor::PointCloudXYZI cloud;
  sensor::PointType point;
  point.x = 1.f;
  point.y = 0.f;
  point.z = 0.f;
  point.curvature = 0.05f;
  cloud.push_back(point);

  std::deque<sensor::ImuData> imus;
  for (int i = 0; i < 8; ++i) {
    imus.push_back(MakeImu(0.01 * i, Eigen::Vector3d(0.0, 0.0, 9.8)));
  }
  ekf.pcl_beg_time = 0.0;
  ekf.pcl_end_time = 0.07;
  EXPECT_EQ(ekf.Process(&state, &cloud, &imus), 0);

  for (int i = 0; i < 10; ++i) {
    imus.push_back(MakeImu(0.1 + 0.01 * i, Eigen::Vector3d(0.0, 0.0, 9.8)));
  }
  ekf.pcl_beg_time = 0.1;
  ekf.pcl_end_time = 0.2;
  ekf.Process(&state, &cloud, &imus);
  EXPECT_TRUE(ekf.init_flag);
  EXPECT_NEAR(state.g.z(), -9.8, 1e-6);
}

TEST(InertialStateFilter, ExpressesReinitializedGravityInWorldFrame) {
  InertialStateFilter filter;
  filter.min_init_num = 1;
  InertialState state;
  state.R = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Vector3d expected_world_gravity(0.0, 0.0, -9.81);
  const Eigen::Vector3d body_acceleration =
      -state.R.transpose() * expected_world_gravity;

  std::deque<sensor::ImuData> imus;
  imus.push_back(MakeImu(0.0, body_acceleration));
  imus.push_back(MakeImu(0.01, body_acceleration));
  sensor::PointCloudXYZI cloud;
  filter.pcl_beg_time = 0.0;
  filter.pcl_end_time = 0.01;

  EXPECT_EQ(filter.Process(&state, &cloud, &imus), 0);
  EXPECT_TRUE(state.g.isApprox(expected_world_gravity, 1e-9));
}

TEST(InertialState, ResetMotionPreservingPoseKeepsWorldAnchor) {
  InertialState state;
  state.t = 42.5;
  state.R = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  state.p = Eigen::Vector3d(4.0, -2.0, 1.5);
  state.v = Eigen::Vector3d(1.0, 2.0, 3.0);
  state.bg = Eigen::Vector3d(0.1, 0.2, 0.3);
  state.ba = Eigen::Vector3d(-0.1, -0.2, -0.3);
  state.g = Eigen::Vector3d(0.0, 0.0, -9.81);
  const double expected_time = state.t;
  const Eigen::Matrix3d expected_rotation = state.R;
  const Eigen::Vector3d expected_position = state.p;
  const Eigen::Vector3d expected_gravity = state.g;

  state.ResetMotionPreservingPose();

  EXPECT_DOUBLE_EQ(state.t, expected_time);
  EXPECT_TRUE(state.R.isApprox(expected_rotation));
  EXPECT_TRUE(state.p.isApprox(expected_position));
  EXPECT_TRUE(state.g.isApprox(expected_gravity));
  EXPECT_TRUE(state.v.isZero());
  EXPECT_TRUE(state.bg.isZero());
  EXPECT_TRUE(state.ba.isZero());
  EXPECT_NEAR(state.cov(0, 0), 0.0001, 1e-12);
}

TEST(PointCluster, AccumulatesMoments) {
  PointCluster cluster;
  cluster.Push(Eigen::Vector3d(1, 0, 0));
  cluster.Push(Eigen::Vector3d(0, 1, 0));
  cluster.Push(Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(cluster.N, 3);
  EXPECT_NEAR(cluster.v.x(), 1.0, 1e-12);
  EXPECT_NEAR(cluster.v.y(), 1.0, 1e-12);
}

}  // namespace
}  // namespace inertial
}  // namespace frontend
}  // namespace atlas
