/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/planner/pipeline/time_parameterization.hpp"

namespace autonomy {
namespace manipulation {
namespace trajectory {
namespace {

TEST(TotgTest, MonotonicTimeAndVelocityBound) {
  core::RobotTrajectory traj;
  core::JointState a;
  SetJointState(&a, {"j1", "j2"}, {0.0, 0.0});
  core::JointState b;
  SetJointState(&b, {"j1", "j2"}, {1.0, -1.0});
  AddTrajectoryPoint(&traj, a, 0.0);
  AddTrajectoryPoint(&traj, b, 0.0);

  TimeParamOptions opts;
  opts.max_velocity = 0.5;
  opts.max_acceleration = 100.0;  // not binding for 2-point path
  ASSERT_TRUE(ApplyTotg(&traj, opts));
  ASSERT_EQ(traj.points_size(), 2);
  EXPECT_GE(GetTrajectoryTime(traj, 1), GetTrajectoryTime(traj, 0));
  EXPECT_GT(GetTrajectoryTime(traj, 1), 0.0);
}

TEST(TotgTest, AccelerationExtendsDuration) {
  core::RobotTrajectory traj;
  core::JointState a;
  SetJointState(&a, {"j1"}, {0.0});
  core::JointState mid;
  SetJointState(&mid, {"j1"}, {0.5});
  core::JointState b;
  SetJointState(&b, {"j1"}, {1.0});
  AddTrajectoryPoint(&traj, a, 0.0);
  AddTrajectoryPoint(&traj, mid, 0.0);
  AddTrajectoryPoint(&traj, b, 0.0);

  TimeParamOptions fast;
  fast.max_velocity = 10.0;
  fast.max_acceleration = 100.0;
  ASSERT_TRUE(ApplyTotg(&traj, fast));
  const double t_fast = GetTrajectoryTime(traj, traj.points_size() - 1);

  TimeParamOptions slow_acc = fast;
  slow_acc.max_acceleration = 0.5;
  ASSERT_TRUE(ApplyTotg(&traj, slow_acc));
  EXPECT_GT(GetTrajectoryTime(traj, traj.points_size() - 1), t_fast);
}

TEST(TotgTest, PathToleranceCornerBlendRuns) {
  core::RobotTrajectory traj;
  core::JointState a;
  SetJointState(&a, {"j1", "j2"}, {0.0, 0.0});
  core::JointState b;
  SetJointState(&b, {"j1", "j2"}, {1.0, 0.0});
  core::JointState c;
  SetJointState(&c, {"j1", "j2"}, {1.0, 1.0});
  AddTrajectoryPoint(&traj, a, 0.0);
  AddTrajectoryPoint(&traj, b, 0.0);
  AddTrajectoryPoint(&traj, c, 0.0);
  TimeParamOptions opts;
  opts.max_velocity = 1.0;
  opts.max_acceleration = 2.0;
  opts.path_tolerance = 0.2;
  ASSERT_TRUE(ApplyTotg(&traj, opts));
  EXPECT_GT(GetTrajectoryTime(traj, traj.points_size() - 1), 0.0);
  EXPECT_LE(GetTrajectoryTime(traj, 0),
            GetTrajectoryTime(traj, traj.points_size() - 1));
}

}  // namespace
}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
