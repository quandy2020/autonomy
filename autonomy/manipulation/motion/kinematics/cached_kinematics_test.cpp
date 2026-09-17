/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <memory>

#include "autonomy/manipulation/model/joint_state_utilities.hpp"
#include "autonomy/manipulation/motion/kinematics/cached_kinematics.hpp"
#include "autonomy/manipulation/motion/kinematics/null_kinematics.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {
namespace {

TEST(CachedKinematicsTest, LruEvictsOldest) {
  auto stub = std::make_shared<NullKinematics>();
  ASSERT_TRUE(stub->Init("arm", "base", "tool0"));
  CachedKinematics cached(stub, /*cache_size=*/2);

  automsgs::msgs::sensor_msgs::JointState seed;
  SetJointState(&seed, {"j1"}, {0.0});
  automsgs::msgs::sensor_msgs::JointState sol;

  automsgs::msgs::geometry_msgs::Pose a;
  SetPose(&a, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  automsgs::msgs::geometry_msgs::Pose b;
  SetPose(&b, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  automsgs::msgs::geometry_msgs::Pose c;
  SetPose(&c, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

  EXPECT_EQ(cached.GetPositionIK(a, seed, common::InverseKinematicsOptions{}, &sol),
      ErrorCode::SUCCESS);
  EXPECT_EQ(cached.GetPositionIK(b, seed, common::InverseKinematicsOptions{}, &sol),
      ErrorCode::SUCCESS);
  EXPECT_EQ(cached.CacheSize(), 2u);
  EXPECT_EQ(cached.GetPositionIK(c, seed, common::InverseKinematicsOptions{}, &sol),
            ErrorCode::SUCCESS);
  EXPECT_EQ(cached.CacheSize(), 2u);

  cached.ClearCache();
  EXPECT_EQ(cached.CacheSize(), 0u);
}

}  // namespace
}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
