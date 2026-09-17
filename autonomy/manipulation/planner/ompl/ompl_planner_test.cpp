/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/planner/ompl/ompl_planner.hpp"
#include "autonomy/manipulation/motion/scene/simple_planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

TEST(OmplPlannerTest, PlansJointSpace) {
  OmplPlanner planner;
  ASSERT_TRUE(planner.Init("ompl"));

  MotionPlanRequest req;
  SetJointState(&req.start_state, {"j1", "j2"}, {0.0, 0.0});
  SetJointState(&req.goal_state, {"j1", "j2"}, {0.4, -0.3});
  req.planning_time = 1.0;
  req.scene = std::make_shared<scene::SimplePlanningScene>();

  const auto resp = planner.Plan(req);
  ASSERT_TRUE(resp.success) << resp.error;
  EXPECT_GE(resp.trajectory.points_size(), 2);
}

}  // namespace
}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
