/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <filesystem>

#include "autonomy/manipulation/model/simple_robot_model.hpp"
#include "autonomy/manipulation/model/simple_robot_state.hpp"
#include "autonomy/manipulation/model/link_forward_kinematics.hpp"
#include "autonomy/manipulation/model/urdf_joint_loader.hpp"

namespace autonomy {
namespace manipulation {
namespace model {
namespace {

std::string TestUrdfPath() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "test_data" / "test_arm.urdf";
}

TEST(UrdfJointsTest, LoadsMovableJoints) {
  std::vector<JointModel> joints;
  std::string error;
  ASSERT_TRUE(LoadJointsFromUrdfFile(TestUrdfPath(), &joints, &error)) << error;
  ASSERT_EQ(joints.size(), 2u);
  EXPECT_EQ(joints[0].name(), "joint1");
  EXPECT_EQ(joints[1].name(), "joint2");
  EXPECT_TRUE(joints[0].limits().has_position_limits());
  EXPECT_NEAR(joints[0].limits().min_position(), -3.14, 1e-6);
  EXPECT_NEAR(joints[0].limits().max_position(), 3.14, 1e-6);
}

TEST(SimpleRobotModelTest, LoadUrdfPopulatesAllGroup) {
  SimpleRobotModel model;
  ASSERT_TRUE(model.Load(TestUrdfPath(), ""));
  const auto all = model.GetJointNames("all");
  ASSERT_EQ(all.size(), 2u);
  model.SetGroupJoints("arm", all);
  EXPECT_EQ(model.GetJointNames("arm").size(), 2u);
  ASSERT_NE(model.GetJointLimits("joint1"), nullptr);
  EXPECT_TRUE(model.GetJointLimits("joint1")->has_position_limits());
}

TEST(SimpleRobotModelTest, LoadDisabledCollisionsFromSrdf) {
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
  automsgs::msgs::sensor_msgs::JointState js;
  js.add_name("joint1");
  js.add_name("joint2");
  js.add_position(10.0);
  js.add_position(-10.0);
  state.SetJointState(js);
  EXPECT_FALSE(state.SatisfiesBounds());
  state.EnforceBounds();
  EXPECT_TRUE(state.SatisfiesBounds());
  const auto clamped = state.GetJointState();
  EXPECT_NEAR(clamped.position(0), 3.14, 1e-6);
  EXPECT_NEAR(clamped.position(1), -3.14, 1e-6);
}

TEST(LinkForwardKinematicsTreeTest, ComputesTipPose) {
  LinkForwardKinematicsTree tree;
  ASSERT_TRUE(tree.LoadFromUrdfFile(TestUrdfPath()));
  EXPECT_EQ(tree.RootLinkName(), "base_link");
  automsgs::msgs::sensor_msgs::JointState js;
  js.add_name("joint1");
  js.add_name("joint2");
  js.add_position(0.0);
  js.add_position(0.0);
  automsgs::msgs::geometry_msgs::Pose tip;
  ASSERT_TRUE(tree.GetLinkPose(js, "tool0", &tip));
  EXPECT_NEAR(tip.position().x(), 0.3, 1e-6);  // 0.2 + 0.1 from URDF origins
  EXPECT_NEAR(tip.position().y(), 0.0, 1e-6);
  EXPECT_NEAR(tip.position().z(), 0.1, 1e-6);
}

}  // namespace
}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
