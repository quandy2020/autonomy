/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/motion/collision/fcl_collision_detector.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/model/link_fk.hpp"
#include "autonomy/manipulation/motion/scene/collision_object_util.hpp"
#include "autonomy/manipulation/motion/scene/simple_planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {
namespace {

TEST(FclCollisionDetectorTest, DetectsSphereObstacleEeProxy) {
  auto detector = CreateFclCollisionDetector();
  ASSERT_NE(detector, nullptr);
  ASSERT_TRUE(detector->Init("fcl"));

  auto scene = std::make_shared<scene::SimplePlanningScene>();
  scene->AddCollisionObject(
      scene::MakeSphereObject("ball", 0.3, 0.0, 0.0, 0.1));

  core::JointState clear;
  SetJointState(&clear, {}, {-1.0});
  EXPECT_FALSE(detector->CheckRobotWorld(clear, *scene).collision);

  core::JointState hit;
  SetJointState(&hit, {}, {0.0});
  EXPECT_TRUE(detector->CheckRobotWorld(hit, *scene).collision);
}

TEST(FclCollisionDetectorTest, AcmSkipsAllowedPairWithLinkTree) {
  auto detector = std::make_shared<FclCollisionDetector>();
  ASSERT_TRUE(detector->Init("fcl"));
  detector->SetLinkRadius(0.05);

  auto scene = std::make_shared<scene::SimplePlanningScene>();
  scene->AddCollisionObject(
      scene::MakeBoxObject("wall", 10.0, 0.0, 0.0, 0.2, 0.2, 0.2));
  scene->SetAllowedCollision("link1", "wall", true);

  core::JointState st;
  SetJointState(&st, {}, {0.0, 0.0});
  EXPECT_FALSE(detector->CheckRobotWorld(st, *scene).collision);
}

}  // namespace
}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
