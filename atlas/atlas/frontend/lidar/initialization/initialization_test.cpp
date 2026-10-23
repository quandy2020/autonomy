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

#include "frontend/lidar/initialization/initialization.hpp"

#include <gtest/gtest.h>

#include <deque>

namespace atlas {
namespace vxslam {
namespace {

TEST(Initialization, AlignsGravityFromMeanAcceleration) {
  std::deque<sensor::ImuData> imus;
  for (int i = 0; i < 20; ++i) {
    sensor::ImuData imu;
    imu.time = 0.01 * i;
    imu.linear_acceleration = Eigen::Vector3d(0.1, -0.2, 9.7);
    imus.push_back(imu);
  }
  ImuState state;
  ASSERT_TRUE(AlignGravity(imus, &state));
  EXPECT_LT(state.g.z(), 0.0);
  EXPECT_NEAR(state.g.norm(), Eigen::Vector3d(0.1, -0.2, 9.7).norm(), 1e-9);
  EXPECT_TRUE(MotionInitReady(5, 5));
  EXPECT_FALSE(MotionInitReady(4, 5));
}

}  // namespace
}  // namespace vxslam
}  // namespace atlas
