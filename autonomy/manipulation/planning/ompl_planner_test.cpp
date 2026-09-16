/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/planning/ompl_planner.hpp"
#include "autonomy/manipulation/scene/simple_planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

TEST(OmplPlannerTest, PlansJointSpace) {
  OmplPlanner planner;
  ASSERT_TRUE(planner.Init("ompl"));

  MotionPlanRequest req;
  req.start_state.names = {"j1", "j2"};
  req.start_state.positions = {0.0, 0.0};
  req.goal_state.names = {"j1", "j2"};
  req.goal_state.positions = {0.4, -0.3};
  req.planning_time = 1.0;
  req.scene = std::make_shared<scene::SimplePlanningScene>();

  const auto resp = planner.Plan(req);
  ASSERT_TRUE(resp.success) << resp.error;
  EXPECT_GE(resp.trajectory.waypoints.size(), 2u);
}

}  // namespace
}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
