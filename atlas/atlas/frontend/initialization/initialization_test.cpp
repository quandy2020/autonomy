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

#include "frontend/initialization/initialization.hpp"

#include <gtest/gtest.h>

#include <deque>

#include "frontend/initialization/state_initializer.hpp"

namespace atlas {
namespace frontend {
namespace initialization {

using ::atlas::frontend::inertial::InertialState;
namespace {

TEST(MotionInitializationInput, DiscardsOldestScanFromSynchronizedBuffers) {
  MotionInitializationInput input;
  input.lidar_scans = {
      std::make_shared<sensor::PointCloudXYZI>(),
      std::make_shared<sensor::PointCloudXYZI>(),
      std::make_shared<sensor::PointCloudXYZI>()};
  input.imu_samples_by_scan.resize(3);
  input.scan_start_times = {1.0, 2.0, 3.0};

  ASSERT_TRUE(DiscardOldestMotionInitializationScan(&input));

  ASSERT_EQ(input.lidar_scans.size(), 2U);
  ASSERT_EQ(input.imu_samples_by_scan.size(), 2U);
  ASSERT_EQ(input.scan_start_times.size(), 2U);
  EXPECT_DOUBLE_EQ(input.scan_start_times[0], 2.0);
  EXPECT_DOUBLE_EQ(input.scan_start_times[1], 3.0);
  EXPECT_TRUE(DiscardOldestMotionInitializationScan(&input));
  EXPECT_TRUE(DiscardOldestMotionInitializationScan(&input));
  EXPECT_FALSE(DiscardOldestMotionInitializationScan(&input));
}

TEST(MotionInitializationWindow, RebasesLatestStateToWorldPoseAnchor) {
  std::vector<InertialState> state_window(2);
  state_window[0].R = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())
                          .toRotationMatrix();
  state_window[0].p = Eigen::Vector3d(2.0, 1.0, 0.0);
  state_window[1].R = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ())
                          .toRotationMatrix();
  state_window[1].p = Eigen::Vector3d(5.0, 3.0, 1.0);
  const Eigen::Vector3d relative_position =
      state_window[1].R.transpose() *
      (state_window[0].p - state_window[1].p);
  InertialState anchor;
  anchor.R = Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ())
                 .toRotationMatrix();
  anchor.p = Eigen::Vector3d(20.0, -8.0, 4.0);

  ASSERT_TRUE(
      RebaseMotionInitializationWindow(&state_window, anchor));

  EXPECT_TRUE(state_window.back().R.isApprox(anchor.R));
  EXPECT_TRUE(state_window.back().p.isApprox(anchor.p));
  EXPECT_TRUE((anchor.R.transpose() *
               (state_window[0].p - state_window[1].p))
                  .isApprox(relative_position));
}

TEST(Initialization, AlignsGravityFromMeanAcceleration) {
  std::deque<sensor::ImuData> imus;
  for (int i = 0; i < 20; ++i) {
    sensor::ImuData imu;
    imu.time = 0.01 * i;
    imu.linear_acceleration = Eigen::Vector3d(0.1, -0.2, 9.7);
    imus.push_back(imu);
  }
  InertialState state;
  ASSERT_TRUE(AlignGravity(imus, &state));
  EXPECT_LT(state.g.z(), 0.0);
  EXPECT_NEAR(state.g.norm(), Eigen::Vector3d(0.1, -0.2, 9.7).norm(), 1e-9);
  EXPECT_TRUE(HasEnoughMotionInitializationScans(5, 5));
  EXPECT_FALSE(HasEnoughMotionInitializationScans(4, 5));
}

TEST(Initializer, RejectsInsufficientImuSamplesWithoutChangingState) {
  frontend::InitializationOptions configuration;
  configuration.minimum_imu_sample_count = 3;
  frontend::StateInitializer initializer(configuration);
  std::vector<sensor::ImuData> imu(2);
  for (std::size_t index = 0; index < imu.size(); ++index) {
    imu[index].time = 0.01 * static_cast<double>(index);
    imu[index].linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
  }
  frontend::FrontendState state;
  state.time = 42.0;
  EXPECT_FALSE(initializer.TryInitialize(imu, &state));
  EXPECT_DOUBLE_EQ(state.time, 42.0);
}

TEST(Initializer, RejectsNonFiniteImuSamples) {
  frontend::InitializationOptions configuration;
  configuration.minimum_imu_sample_count = 2;
  frontend::StateInitializer initializer(configuration);
  std::vector<sensor::ImuData> imu(2);
  imu[0].time = 0.0;
  imu[1].time = 0.01;
  imu[0].linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
  imu[1].linear_acceleration =
      Eigen::Vector3d(0.0, 0.0, std::numeric_limits<double>::quiet_NaN());
  frontend::FrontendState state;
  EXPECT_FALSE(initializer.TryInitialize(imu, &state));
}

TEST(Initializer, RejectsNonIncreasingImuTimestamps) {
  frontend::InitializationOptions configuration;
  configuration.minimum_imu_sample_count = 2;
  frontend::StateInitializer initializer(configuration);
  std::vector<sensor::ImuData> imu(2);
  imu[0].time = 0.01;
  imu[1].time = 0.01;
  for (sensor::ImuData& sample : imu) {
    sample.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
  }
  frontend::FrontendState state;
  EXPECT_FALSE(initializer.TryInitialize(imu, &state));
}

}  // namespace
}  // namespace initialization
}  // namespace frontend
}  // namespace atlas
