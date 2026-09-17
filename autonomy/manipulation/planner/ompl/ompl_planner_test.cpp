/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/model/joint_state_utilities.hpp"
#include "autonomy/manipulation/planner/ompl/ompl_planner.hpp"
#include "autonomy/manipulation/motion/scene/simple_planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {
namespace {

TEST(OmplPlannerTest, PlansJointSpace) {
  OmplPlanner planner;
  ASSERT_TRUE(planner.Init("ompl"));

  MotionPlanRequest req;
  SetJointState(req.pb.mutable_start_state(), {"j1", "j2"}, {0.0, 0.0});
  SetJointState(req.pb.mutable_goal_state(), {"j1", "j2"}, {0.4, -0.3});
  req.pb.set_planning_time(1.0);
  req.scene = std::make_shared<scene::SimplePlanningScene>();

  const auto resp = planner.Plan(req);
  ASSERT_TRUE(resp.success()) << resp.error();
  EXPECT_GE(resp.trajectory().points_size(), 2);
}

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
