/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/planning/time_parameterization.hpp"

namespace autonomy {
namespace manipulation {
namespace trajectory {
namespace {

TEST(TotgTest, MonotonicTimeAndVelocityBound) {
  core::RobotTrajectory traj;
  core::JointState a;
  a.positions = {0.0, 0.0};
  core::JointState b;
  b.positions = {1.0, -1.0};
  traj.waypoints = {a, b};
  traj.time_from_start = {0.0, 0.0};

  TimeParamOptions opts;
  opts.max_velocity = 0.5;
  ASSERT_TRUE(ApplyTotg(&traj, opts));
  ASSERT_EQ(traj.time_from_start.size(), 2u);
  EXPECT_GE(traj.time_from_start[1], traj.time_from_start[0]);
  EXPECT_NEAR(traj.time_from_start[1], 2.0, 1e-6);  // max_dq=1 / vmax=0.5
}

}  // namespace
}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
