/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <memory>

#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/motion/kinematics/cached_kinematics.hpp"
#include "autonomy/manipulation/motion/kinematics/stub_kinematics.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {
namespace {

TEST(CachedKinematicsTest, LruEvictsOldest) {
  auto stub = std::make_shared<StubKinematics>();
  ASSERT_TRUE(stub->Init("arm", "base", "tool0"));
  CachedKinematics cached(stub, /*cache_size=*/2);

  core::JointState seed;
  SetJointState(&seed, {"j1"}, {0.0});
  core::JointState sol;

  Pose a;
  SetPose(&a, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  Pose b;
  SetPose(&b, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  Pose c;
  SetPose(&c, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

  EXPECT_EQ(cached.GetPositionIK(a, seed, IkOptions{}, &sol),
            ErrorCode::kSuccess);
  EXPECT_EQ(cached.GetPositionIK(b, seed, IkOptions{}, &sol),
            ErrorCode::kSuccess);
  EXPECT_EQ(cached.CacheSize(), 2u);
  EXPECT_EQ(cached.GetPositionIK(c, seed, IkOptions{}, &sol),
            ErrorCode::kSuccess);
  EXPECT_EQ(cached.CacheSize(), 2u);

  cached.ClearCache();
  EXPECT_EQ(cached.CacheSize(), 0u);
}

}  // namespace
}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
