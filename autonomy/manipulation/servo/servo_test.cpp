/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <cmath>

#include "autonomy/manipulation/kinematics/stub_kinematics.hpp"
#include "autonomy/manipulation/servo/servo.hpp"

namespace autonomy {
namespace manipulation {
namespace servo {
namespace {

TEST(ServoTest, BoundedJointIncrement) {
  auto kin = std::make_shared<kinematics::StubKinematics>();
  ASSERT_TRUE(kin->Init("arm", "base", "tool0"));

  DampedLeastSquaresServo servo;
  servo.SetKinematics(kin);
  core::JointState q;
  q.names = {"j1", "j2"};
  q.positions = {0.0, 0.0};
  servo.SetState(q);
  servo.SetDt(0.01);
  servo.SetMaxJointVelocity(0.5);
  ASSERT_TRUE(servo.Init());

  TwistCommand twist;
  twist.vx = 0.1;
  core::JointState out;
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(servo.Update(twist, &out));
  }
  ASSERT_EQ(out.positions.size(), 2u);
  EXPECT_LE(std::abs(out.positions[0]), 0.5);
  EXPECT_LE(std::abs(out.positions[1]), 0.5);
}

}  // namespace
}  // namespace servo
}  // namespace manipulation
}  // namespace autonomy
