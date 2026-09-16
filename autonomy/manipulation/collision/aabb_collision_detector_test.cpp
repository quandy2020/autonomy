/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/collision/aabb_collision_detector.hpp"
#include "autonomy/manipulation/planning/joint_interpolation_planner.hpp"
#include "autonomy/manipulation/scene/simple_planning_scene.hpp"

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

  scene::CollisionObject box;
  box.id = "wall";
  box.type = scene::ShapeType::kBox;
  box.x = 0.3;
  box.y = 0.0;
  box.z = 0.0;
  box.size_x = 0.2;
  box.size_y = 0.2;
  box.size_z = 0.2;
  scene->AddCollisionObject(box);

  planning::JointInterpolationPlanner planner;
  ASSERT_TRUE(planner.Init("joint_interpolation"));
  planner.SetNumSteps(10);

  planning::MotionPlanRequest req;
  req.start_state.positions = {0.0};
  req.goal_state.positions = {1.57};
  req.scene = scene;
  const auto resp = planner.Plan(req);
  EXPECT_FALSE(resp.success);
}

}  // namespace
}  // namespace manipulation
}  // namespace autonomy
