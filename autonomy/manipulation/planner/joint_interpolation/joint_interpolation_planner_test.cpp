/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/planner/joint_interpolation/joint_interpolation_planner.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

TEST(JointInterpolationPlannerTest, InterpolatesJointGoal) {
  JointInterpolationPlanner planner;
  ASSERT_TRUE(planner.Init("joint_interpolation"));
  planner.SetNumSteps(5);

  MotionPlanRequest request;
  SetJointState(&request.start_state, {"j1", "j2"}, {0.0, 0.0});
  SetJointState(&request.goal_state, {"j1", "j2"}, {1.0, -1.0});
  request.planning_time = 1.0;

  const MotionPlanResponse response = planner.Plan(request);
  ASSERT_TRUE(response.success);
  ASSERT_EQ(response.trajectory.points_size(), 5);
  EXPECT_DOUBLE_EQ(response.trajectory.points(0).positions(0), 0.0);
  EXPECT_DOUBLE_EQ(
      response.trajectory.points(response.trajectory.points_size() - 1)
          .positions(0),
      1.0);
  EXPECT_DOUBLE_EQ(
      response.trajectory.points(response.trajectory.points_size() - 1)
          .positions(1),
      -1.0);
}

TEST(JointInterpolationPlannerTest, RejectsPoseGoalWithoutIk) {
  JointInterpolationPlanner planner;
  ASSERT_TRUE(planner.Init("joint_interpolation"));

  MotionPlanRequest request;
  request.has_goal_pose = true;
  SetJointState(&request.start_state, {"j1"}, {0.0});
  SetJointState(&request.goal_state, {"j1"}, {1.0});

  const MotionPlanResponse response = planner.Plan(request);
  EXPECT_FALSE(response.success);
  EXPECT_FALSE(response.error.empty());
}

}  // namespace
}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
