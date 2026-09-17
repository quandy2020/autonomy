/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/motion/execution/effort_tracking_controller.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {
namespace {

TEST(EffortTrackingTest, PassThroughWhenNoActual) {
  const auto cmd = CorrectEffortCommand({1.0, -2.0}, {}, 0.5, 0.0);
  ASSERT_EQ(cmd.size(), 2u);
  EXPECT_NEAR(cmd[0], 1.0, 1e-9);
  EXPECT_NEAR(cmd[1], -2.0, 1e-9);
}

TEST(EffortTrackingTest, PCorrectionAndClamp) {
  // desired=10, actual=8, kp=0.5 → 10 + 0.5*2 = 11 → clamp 10
  const auto cmd = CorrectEffortCommand({10.0}, {8.0}, 0.5, 10.0);
  ASSERT_EQ(cmd.size(), 1u);
  EXPECT_NEAR(cmd[0], 10.0, 1e-9);
}

TEST(EffortTrackingTest, ControllerPublishesCorrected) {
  EffortTrackingController ctl;
  ctl.SetKp(1.0);
  ctl.SetMaxAbsEffort(0.0);
  std::vector<double> published;
  ctl.SetCommandPublisher([&](const std::vector<double>& c) { published = c; });
  ctl.SetStateProvider([]() {
    core::JointState s;
    s.add_effort(1.0);
    return s;
  });
  core::JointState des;
  des.add_effort(3.0);
  ctl.SetDesired(des);
  ASSERT_TRUE(ctl.Publish());
  ASSERT_EQ(published.size(), 1u);
  EXPECT_NEAR(published[0], 5.0, 1e-9);  // 3 + 1*(3-1)
}

}  // namespace
}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
