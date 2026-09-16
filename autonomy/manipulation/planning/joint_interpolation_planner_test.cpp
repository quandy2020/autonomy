/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/planning/joint_interpolation_planner.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

TEST(JointInterpolationPlannerTest, InterpolatesJointGoal) {
  JointInterpolationPlanner planner;
  ASSERT_TRUE(planner.Init("joint_interpolation"));
  planner.SetNumSteps(5);

  MotionPlanRequest request;
  request.start_state.positions = {0.0, 0.0};
  request.goal_state.positions = {1.0, -1.0};
  request.planning_time = 1.0;

  const MotionPlanResponse response = planner.Plan(request);
  ASSERT_TRUE(response.success);
  ASSERT_EQ(response.trajectory.waypoints.size(), 5u);
  EXPECT_DOUBLE_EQ(response.trajectory.waypoints.front().positions[0], 0.0);
  EXPECT_DOUBLE_EQ(response.trajectory.waypoints.back().positions[0], 1.0);
  EXPECT_DOUBLE_EQ(response.trajectory.waypoints.back().positions[1], -1.0);
}

TEST(JointInterpolationPlannerTest, RejectsPoseGoalWithoutIk) {
  JointInterpolationPlanner planner;
  ASSERT_TRUE(planner.Init("joint_interpolation"));

  MotionPlanRequest request;
  request.has_goal_pose = true;
  request.start_state.positions = {0.0};
  request.goal_state.positions = {1.0};

  const MotionPlanResponse response = planner.Plan(request);
  EXPECT_FALSE(response.success);
  EXPECT_FALSE(response.error.empty());
}

}  // namespace
}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
