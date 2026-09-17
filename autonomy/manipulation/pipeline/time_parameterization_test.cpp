/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/model/joint_state_utilities.hpp"
#include "autonomy/manipulation/pipeline/time_parameterization.hpp"

namespace autonomy {
namespace manipulation {
namespace trajectory {
namespace {

TEST(TimeOptimalTrajectoryGenerationTest, MonotonicTimeAndVelocityBound) {
  automsgs::msgs::trajectory_msgs::JointTrajectory traj;
  automsgs::msgs::sensor_msgs::JointState a;
  SetJointState(&a, {"j1", "j2"}, {0.0, 0.0});
  automsgs::msgs::sensor_msgs::JointState b;
  SetJointState(&b, {"j1", "j2"}, {1.0, -1.0});
  AddTrajectoryPoint(&traj, a, 0.0);
  AddTrajectoryPoint(&traj, b, 0.0);

  TimeParameterizationOptions options;
  options.set_max_velocity(0.5);
  options.set_max_acceleration(100.0);  // not binding for 2-point path
  ASSERT_TRUE(ApplyTimeOptimalTrajectoryGeneration(&traj, options));
  ASSERT_EQ(traj.points_size(), 2);
  EXPECT_GE(GetTrajectoryPointTimeSeconds(traj, 1), GetTrajectoryPointTimeSeconds(traj, 0));
  EXPECT_GT(GetTrajectoryPointTimeSeconds(traj, 1), 0.0);
}

TEST(TimeOptimalTrajectoryGenerationTest, AccelerationExtendsDuration) {
  automsgs::msgs::trajectory_msgs::JointTrajectory traj;
  automsgs::msgs::sensor_msgs::JointState a;
  SetJointState(&a, {"j1"}, {0.0});
  automsgs::msgs::sensor_msgs::JointState mid;
  SetJointState(&mid, {"j1"}, {0.5});
  automsgs::msgs::sensor_msgs::JointState b;
  SetJointState(&b, {"j1"}, {1.0});
  AddTrajectoryPoint(&traj, a, 0.0);
  AddTrajectoryPoint(&traj, mid, 0.0);
  AddTrajectoryPoint(&traj, b, 0.0);

  TimeParameterizationOptions fast;
  fast.set_max_velocity(10.0);
  fast.set_max_acceleration(100.0);
  ASSERT_TRUE(ApplyTimeOptimalTrajectoryGeneration(&traj, fast));
  const double t_fast = GetTrajectoryPointTimeSeconds(traj, traj.points_size() - 1);

  TimeParameterizationOptions slow_acc = fast;
  slow_acc.set_max_acceleration(0.5);
  ASSERT_TRUE(ApplyTimeOptimalTrajectoryGeneration(&traj, slow_acc));
  EXPECT_GT(GetTrajectoryPointTimeSeconds(traj, traj.points_size() - 1), t_fast);
}

TEST(TimeOptimalTrajectoryGenerationTest, PathToleranceCornerBlendRuns) {
  automsgs::msgs::trajectory_msgs::JointTrajectory traj;
  automsgs::msgs::sensor_msgs::JointState a;
  SetJointState(&a, {"j1", "j2"}, {0.0, 0.0});
  automsgs::msgs::sensor_msgs::JointState b;
  SetJointState(&b, {"j1", "j2"}, {1.0, 0.0});
  automsgs::msgs::sensor_msgs::JointState c;
  SetJointState(&c, {"j1", "j2"}, {1.0, 1.0});
  AddTrajectoryPoint(&traj, a, 0.0);
  AddTrajectoryPoint(&traj, b, 0.0);
  AddTrajectoryPoint(&traj, c, 0.0);
  TimeParameterizationOptions options;
  options.set_max_velocity(1.0);
  options.set_max_acceleration(2.0);
  options.set_path_tolerance(0.2);
  ASSERT_TRUE(ApplyTimeOptimalTrajectoryGeneration(&traj, options));
  EXPECT_GT(GetTrajectoryPointTimeSeconds(traj, traj.points_size() - 1), 0.0);
  EXPECT_LE(GetTrajectoryPointTimeSeconds(traj, 0),
            GetTrajectoryPointTimeSeconds(traj, traj.points_size() - 1));
}

}  // namespace
}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
