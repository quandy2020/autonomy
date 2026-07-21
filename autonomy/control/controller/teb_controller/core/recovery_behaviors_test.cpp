/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/control/controller/teb_controller/core/recovery_behaviors.hpp"

#include "gtest/gtest.h"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {
namespace {

Twist MakeTwist(double vx, double omega) {
  Twist t;
  t.linear.x = vx;
  t.angular.z = omega;
  return t;
}

TEST(FailureDetectorTest, DetectsOscillationOnAlternatingOmega) {
  FailureDetector detector;
  detector.setBufferLength(10);
  for (int i = 0; i < 10; ++i) {
    const double omega = (i % 2 == 0) ? 0.05 : -0.05;
    detector.update(MakeTwist(0.02, omega), /*v_max=*/1.0,
                    /*v_backwards_max=*/1.0, /*omega_max=*/1.0,
                    /*v_eps=*/0.1, /*omega_eps=*/0.1);
  }
  EXPECT_TRUE(detector.isOscillating());
}

TEST(FailureDetectorTest, ClearResetsState) {
  FailureDetector detector;
  detector.setBufferLength(8);
  for (int i = 0; i < 8; ++i) {
    detector.update(MakeTwist(0.01, (i % 2) ? 0.05 : -0.05), 1, 1, 1, 0.1,
                    0.1);
  }
  ASSERT_TRUE(detector.isOscillating());
  detector.clear();
  EXPECT_FALSE(detector.isOscillating());
}

}  // namespace
}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
