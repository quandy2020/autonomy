/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <cmath>
#include <filesystem>

#include "autonomy/manipulation/model/urdf_kdl_chain.hpp"
#include "autonomy/manipulation/motion/kinematics/kdl_kinematics.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {
namespace {

std::string TestUrdfPath() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "test_data" / "test_arm.urdf";
}

TEST(UrdfKdlTest, BuildsChain) {
  model::KdlChainDescription model;
  std::string error;
  ASSERT_TRUE(model::BuildKdlChainFromUrdfFile(TestUrdfPath(), "base_link", "tool0",
                                          &model, &error))
      << error;
  ASSERT_EQ(model.joint_names.size(), 2u);
  EXPECT_EQ(model.joint_names[0], "joint1");
  EXPECT_EQ(model.joint_names[1], "joint2");
  EXPECT_EQ(model.chain.getNrOfJoints(), 2u);
}

TEST(KdlKinematicsTest, FkIkRoundTrip) {
  KdlKinematics kin;
  ASSERT_TRUE(kin.Init("arm", "base_link", "tool0"));
  ASSERT_TRUE(kin.LoadFromUrdfFile(TestUrdfPath()));

  automsgs::msgs::sensor_msgs::JointState joints;
  SetJointState(&joints, kin.JointNames(), {0.4, -0.3});

  automsgs::msgs::geometry_msgs::Pose tip;
  ASSERT_TRUE(kin.GetPositionFK(joints, &tip));
  EXPECT_GT(std::hypot(tip.position().x(), tip.position().y(),
                       tip.position().z()),
            0.1);

  automsgs::msgs::sensor_msgs::JointState seed;
  SetJointState(&seed, kin.JointNames(), {0.0, 0.0});
  automsgs::msgs::sensor_msgs::JointState solution;
  common::InverseKinematicsOptions opts;
  opts.set_max_attempts(8);
  ASSERT_EQ(kin.GetPositionIK(tip, seed, opts, &solution), ErrorCode::SUCCESS);
  ASSERT_EQ(solution.position_size(), 2);

  automsgs::msgs::geometry_msgs::Pose tip2;
  ASSERT_TRUE(kin.GetPositionFK(solution, &tip2));
  EXPECT_NEAR(tip2.position().x(), tip.position().x(), 1e-3);
  EXPECT_NEAR(tip2.position().y(), tip.position().y(), 1e-3);
  EXPECT_NEAR(tip2.position().z(), tip.position().z(), 1e-3);
}

TEST(KdlKinematicsTest, PositionOnlyIk) {
  KdlKinematics kin;
  ASSERT_TRUE(kin.Init("arm", "base_link", "tool0"));
  ASSERT_TRUE(kin.LoadFromUrdfFile(TestUrdfPath()));

  automsgs::msgs::sensor_msgs::JointState joints;
  SetJointState(&joints, kin.JointNames(), {0.2, 0.1});
  automsgs::msgs::geometry_msgs::Pose tip;
  ASSERT_TRUE(kin.GetPositionFK(joints, &tip));

  common::InverseKinematicsOptions opts;
  opts.set_position_only(true);
  opts.set_max_attempts(8);
  // Seed near the solution so NR_JL converges under position-only.
  automsgs::msgs::sensor_msgs::JointState seed;
  SetJointState(&seed, kin.JointNames(), {0.15, 0.08});
  automsgs::msgs::sensor_msgs::JointState solution;
  ASSERT_EQ(kin.GetPositionIK(tip, seed, opts, &solution), ErrorCode::SUCCESS);
  automsgs::msgs::geometry_msgs::Pose tip2;
  ASSERT_TRUE(kin.GetPositionFK(solution, &tip2));
  EXPECT_NEAR(tip2.position().x(), tip.position().x(), 5e-3);
  EXPECT_NEAR(tip2.position().y(), tip.position().y(), 5e-3);
  EXPECT_NEAR(tip2.position().z(), tip.position().z(), 5e-3);
}

}  // namespace
}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
