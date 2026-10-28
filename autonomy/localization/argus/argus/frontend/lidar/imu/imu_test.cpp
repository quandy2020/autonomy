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

#include <cmath>
#include <deque>

#include "frontend/lidar/imu/imu_ekf.hpp"
#include "frontend/lidar/imu/imu_state.hpp"
#include "map/voxel/point_cluster.hpp"

namespace argus {
namespace vxslam {
namespace {

sensor::ImuData MakeImu(double time, const Eigen::Vector3d& accel) {
  sensor::ImuData imu;
  imu.time = time;
  imu.linear_acceleration = accel;
  imu.angular_velocity = Eigen::Vector3d::Zero();
  return imu;
}

TEST(ImuEkf, InitializesGravityThenUndistorts) {
  ImuEkf ekf;
  ekf.min_init_num = 5;
  ImuState state;
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
}  // namespace vxslam
}  // namespace argus
