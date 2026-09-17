/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/motion/collision/aabb_collision_detector.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/planner/joint_interpolation/joint_interpolation_planner.hpp"
#include "autonomy/manipulation/motion/scene/collision_object_util.hpp"
#include "autonomy/manipulation/motion/scene/simple_planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace {

TEST(CollisionSceneTest, PathInvalidWithObstacle) {
  auto scene = std::make_shared<scene::SimplePlanningScene>();
  auto detector = std::make_shared<collision::AabbCollisionDetector>();
  ASSERT_TRUE(detector->Init("aabb"));
  detector->SetLinkLength(0.3);
  detector->SetEeRadius(0.05);
  scene->SetCollisionDetector(detector);

  scene->AddCollisionObject(
      scene::MakeBoxObject("wall", 0.3, 0.0, 0.0, 0.2, 0.2, 0.2));

  planning::JointInterpolationPlanner planner;
  ASSERT_TRUE(planner.Init("joint_interpolation"));
  planner.SetNumSteps(10);

  planning::MotionPlanRequest req;
  SetJointState(&req.start_state, {}, {0.0});
  SetJointState(&req.goal_state, {}, {1.57});
  req.scene = scene;
  const auto resp = planner.Plan(req);
  EXPECT_FALSE(resp.success);
}

}  // namespace
}  // namespace manipulation
}  // namespace autonomy
