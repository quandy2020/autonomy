/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/planner/joint_interpolation/joint_interpolation_planner.hpp"
#include "autonomy/manipulation/planner/rrt_connect/rrt_connect_planner.hpp"
#include "autonomy/manipulation/motion/scene/simple_planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace {

TEST(PlanBenchmarkTest, JointInterpolationSuccessRate) {
  planning::JointInterpolationPlanner planner;
  ASSERT_TRUE(planner.Init("joint_interpolation"));
  int ok = 0;
  for (int i = 0; i < 20; ++i) {
    planning::MotionPlanRequest req;
    SetJointState(&req.start_state, {}, {0.0, 0.0});
    SetJointState(&req.goal_state, {}, {0.1 * i, -0.05 * i});
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
  SetJointState(&req.start_state, {"j1", "j2"}, {0.0, 0.0});
  SetJointState(&req.goal_state, {"j1", "j2"}, {0.5, -0.5});
  req.scene = std::make_shared<scene::SimplePlanningScene>();
  EXPECT_TRUE(planner.Plan(req).success);
}

}  // namespace
}  // namespace manipulation
}  // namespace autonomy
