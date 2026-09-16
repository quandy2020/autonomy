/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <filesystem>

#include "autonomy/manipulation/core/simple_robot_model.hpp"
#include "autonomy/manipulation/core/link_fk.hpp"
#include "autonomy/manipulation/core/urdf_joints.hpp"

namespace autonomy {
namespace manipulation {
namespace core {
namespace {

std::string TestUrdfPath() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "test_data" / "test_arm.urdf";
}

TEST(UrdfJointsTest, LoadsMovableJoints) {
  std::vector<UrdfJointInfo> joints;
  std::string error;
  ASSERT_TRUE(LoadUrdfJoints(TestUrdfPath(), &joints, &error)) << error;
  ASSERT_EQ(joints.size(), 2u);
  EXPECT_EQ(joints[0].name, "joint1");
  EXPECT_EQ(joints[1].name, "joint2");
  EXPECT_TRUE(joints[0].has_position_limits);
  EXPECT_NEAR(joints[0].lower, -3.14, 1e-6);
  EXPECT_NEAR(joints[0].upper, 3.14, 1e-6);
}

TEST(SimpleRobotModelTest, LoadUrdfPopulatesAllGroup) {
  SimpleRobotModel model;
  ASSERT_TRUE(model.Load(TestUrdfPath(), ""));
  const auto all = model.GetJointNames("all");
  ASSERT_EQ(all.size(), 2u);
  model.SetGroupJoints("arm", all);
  EXPECT_EQ(model.GetJointNames("arm").size(), 2u);
  ASSERT_NE(model.GetJointLimits("joint1"), nullptr);
  EXPECT_TRUE(model.GetJointLimits("joint1")->has_position_limits);
}

TEST(SimpleRobotModelTest, LoadSrdfDisableCollisions) {
  const auto srdf = std::filesystem::path(__FILE__).parent_path().parent_path() /
                    "test_data" / "test_arm.srdf";
  SimpleRobotModel model;
  ASSERT_TRUE(model.Load(TestUrdfPath(), srdf.string()));
  EXPECT_TRUE(model.HasGroup("arm"));
  EXPECT_EQ(model.GetJointNames("arm").size(), 2u);
  ASSERT_EQ(model.DisabledCollisions().size(), 2u);
}

TEST(SimpleRobotStateTest, EnforceBounds) {
  auto model = std::make_shared<SimpleRobotModel>();
  ASSERT_TRUE(model->Load(TestUrdfPath(), ""));
  SimpleRobotState state(model);
  JointState js;
  js.names = {"joint1", "joint2"};
  js.positions = {10.0, -10.0};
  state.SetJointState(js);
  EXPECT_FALSE(state.SatisfiesBounds());
  state.EnforceBounds();
  EXPECT_TRUE(state.SatisfiesBounds());
  const auto clamped = state.GetJointState();
  EXPECT_NEAR(clamped.positions[0], 3.14, 1e-6);
  EXPECT_NEAR(clamped.positions[1], -3.14, 1e-6);
}

TEST(LinkFkTest, ComputesTipPose) {
  LinkFkTree tree;
  ASSERT_TRUE(tree.LoadUrdf(TestUrdfPath()));
  EXPECT_EQ(tree.RootLink(), "base_link");
  JointState js;
  js.names = {"joint1", "joint2"};
  js.positions = {0.0, 0.0};
  Transform tip;
  ASSERT_TRUE(tree.GetLinkPose(js, "tool0", &tip));
  EXPECT_NEAR(tip.x, 0.3, 1e-6);  // 0.2 + 0.1 from URDF origins
  EXPECT_NEAR(tip.y, 0.0, 1e-6);
  EXPECT_NEAR(tip.z, 0.1, 1e-6);
}

}  // namespace
}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
