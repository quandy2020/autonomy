/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <automsgs/msgs/trajectory_msgs/joint_trajectory.pb.h>

#include "autonomy/manipulation/motion/collision/aabb_collision_detector.hpp"
#include "autonomy/manipulation/motion/scene/collision_object_helpers.hpp"
#include "autonomy/manipulation/motion/scene/simple_planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace {

TEST(CollisionSceneTest, PathInvalidWithObstacle) {
  auto scene = std::make_shared<scene::SimplePlanningScene>();
  auto detector = std::make_shared<collision::AabbCollisionDetector>();
  ASSERT_TRUE(detector->Init("aabb"));
  detector->SetLinkLength(0.3);
  detector->SetEndEffectorRadius(0.05);
  scene->SetCollisionDetector(detector);

  scene->AddCollisionObject(
      scene::MakeBoxObject("wall", 0.3, 0.0, 0.0, 0.2, 0.2, 0.2));

  automsgs::msgs::trajectory_msgs::JointTrajectory traj;
  for (int i = 0; i <= 10; ++i) {
    auto* pt = traj.add_points();
    pt->add_positions(0.157 * i);
  }
  EXPECT_FALSE(scene->IsPathValid(traj));
}

}  // namespace
}  // namespace manipulation
}  // namespace autonomy
