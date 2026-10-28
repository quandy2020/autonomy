#include "common/keyframe.hpp"

#include <gtest/gtest.h>

namespace argus {
namespace common {
namespace {

TEST(Keyframe, StoresIdTimeAndPose) {
  const transform::Rigid3d pose =
      transform::Rigid3d::Translation(Eigen::Vector3d(0.5, 0.0, 1.0));
  Keyframe frame(7, 2.0, pose);
  EXPECT_EQ(frame.id(), 7);
  EXPECT_DOUBLE_EQ(frame.time(), 2.0);
  EXPECT_TRUE(frame.pose().translation().isApprox(pose.translation()));

  const transform::Rigid3d optimized =
      transform::Rigid3d::Translation(Eigen::Vector3d(1.0, 0.0, 0.0));
  frame.SetPose(optimized);
  EXPECT_TRUE(frame.pose().translation().isApprox(optimized.translation()));
  EXPECT_DOUBLE_EQ(frame.time(), 2.0);
}

}  // namespace
}  // namespace common
}  // namespace argus
