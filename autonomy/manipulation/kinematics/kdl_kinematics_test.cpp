/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <cmath>
#include <filesystem>

#include "autonomy/manipulation/core/urdf_kdl.hpp"
#include "autonomy/manipulation/kinematics/kdl_kinematics.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {
namespace {

std::string TestUrdfPath() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "test_data" / "test_arm.urdf";
}

TEST(UrdfKdlTest, BuildsChain) {
  core::KdlChainModel model;
  std::string error;
  ASSERT_TRUE(core::BuildKdlChainFromUrdf(TestUrdfPath(), "base_link", "tool0",
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
  ASSERT_TRUE(kin.LoadUrdf(TestUrdfPath()));

  core::JointState joints;
  joints.names = kin.JointNames();
  joints.positions = {0.4, -0.3};

  Pose tip;
  ASSERT_TRUE(kin.GetPositionFK(joints, &tip));
  EXPECT_GT(std::hypot(tip.x, tip.y, tip.z), 0.1);

  core::JointState seed;
  seed.names = joints.names;
  seed.positions = {0.0, 0.0};
  core::JointState solution;
  IkOptions opts;
  opts.max_attempts = 8;
  ASSERT_EQ(kin.GetPositionIK(tip, seed, opts, &solution), ErrorCode::kSuccess);
  ASSERT_EQ(solution.positions.size(), 2u);

  Pose tip2;
  ASSERT_TRUE(kin.GetPositionFK(solution, &tip2));
  EXPECT_NEAR(tip2.x, tip.x, 1e-3);
  EXPECT_NEAR(tip2.y, tip.y, 1e-3);
  EXPECT_NEAR(tip2.z, tip.z, 1e-3);
}

TEST(KdlKinematicsTest, PositionOnlyIk) {
  KdlKinematics kin;
  ASSERT_TRUE(kin.Init("arm", "base_link", "tool0"));
  ASSERT_TRUE(kin.LoadUrdf(TestUrdfPath()));

  core::JointState joints;
  joints.names = kin.JointNames();
  joints.positions = {0.2, 0.1};
  Pose tip;
  ASSERT_TRUE(kin.GetPositionFK(joints, &tip));

  IkOptions opts;
  opts.position_only = true;
  opts.max_attempts = 8;
  // Seed near the solution so NR_JL converges under position-only.
  core::JointState seed = joints;
  seed.positions = {0.15, 0.08};
  core::JointState solution;
  ASSERT_EQ(kin.GetPositionIK(tip, seed, opts, &solution), ErrorCode::kSuccess);
  Pose tip2;
  ASSERT_TRUE(kin.GetPositionFK(solution, &tip2));
  EXPECT_NEAR(tip2.x, tip.x, 5e-3);
  EXPECT_NEAR(tip2.y, tip.y, 5e-3);
  EXPECT_NEAR(tip2.z, tip.z, 5e-3);
}

}  // namespace
}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
