/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/planning/joint_interpolation_planner.hpp"
#include "autonomy/manipulation/planning/rrt_connect_planner.hpp"
#include "autonomy/manipulation/scene/simple_planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace {

TEST(PlanBenchmarkTest, JointInterpolationSuccessRate) {
  planning::JointInterpolationPlanner planner;
  ASSERT_TRUE(planner.Init("joint_interpolation"));
  int ok = 0;
  for (int i = 0; i < 20; ++i) {
    planning::MotionPlanRequest req;
    req.start_state.positions = {0.0, 0.0};
    req.goal_state.positions = {0.1 * i, -0.05 * i};
    if (planner.Plan(req).success) {
      ++ok;
    }
  }
  EXPECT_EQ(ok, 20);
}

TEST(PlanBenchmarkTest, RrtConnectWithoutObstacle) {
  planning::RrtConnectPlanner planner;
  ASSERT_TRUE(planner.Init("rrt_connect"));
  planning::MotionPlanRequest req;
  req.start_state.names = {"j1", "j2"};
  req.start_state.positions = {0.0, 0.0};
  req.goal_state.names = {"j1", "j2"};
  req.goal_state.positions = {0.5, -0.5};
  req.scene = std::make_shared<scene::SimplePlanningScene>();
  EXPECT_TRUE(planner.Plan(req).success);
}

}  // namespace
}  // namespace manipulation
}  // namespace autonomy
