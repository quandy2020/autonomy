/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/collision/fcl_collision_detector.hpp"
#include "autonomy/manipulation/scene/simple_planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {
namespace {

TEST(FclCollisionDetectorTest, DetectsSphereObstacle) {
  auto detector = CreateFclCollisionDetector();
  ASSERT_NE(detector, nullptr);
  ASSERT_TRUE(detector->Init("fcl"));

  auto scene = std::make_shared<scene::SimplePlanningScene>();
  scene::CollisionObject obj;
  obj.id = "ball";
  obj.type = scene::ShapeType::kSphere;
  obj.x = 0.3;
  obj.y = 0.0;
  obj.z = 0.0;
  obj.size_x = 0.1;
  scene->AddCollisionObject(obj);

  core::JointState clear;
  clear.positions = {-1.0};
  EXPECT_FALSE(detector->CheckRobotWorld(clear, *scene).collision);

  core::JointState hit;
  hit.positions = {0.0};
  EXPECT_TRUE(detector->CheckRobotWorld(hit, *scene).collision);
}

}  // namespace
}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
