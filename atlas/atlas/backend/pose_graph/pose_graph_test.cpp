#include <gtest/gtest.h>

#include "backend/backend_runner.hpp"

namespace atlas {
namespace backend {
namespace {

TEST(BackendRunner, OptimizesOdometryFactorsWithGtsam) {
  BackendRunner backend;
  EXPECT_STREQ(backend.solver_name(), "gtsam");
  ASSERT_TRUE(backend.AddNode(0, transform::Rigid3d::Identity(), true));
  ASSERT_TRUE(backend.AddNode(
      1, transform::Rigid3d::Translation(Eigen::Vector3d(1.3, 0.2, 0.0)),
      false));
  RelativePoseFactor factor;
  factor.from_id = 0;
  factor.to_id = 1;
  factor.relative_pose =
      transform::Rigid3d::Translation(Eigen::Vector3d(1.0, 0.0, 0.0));
  factor.kind = PoseFactorKind::kPose3;
  backend.AddFactor(factor);
  ASSERT_TRUE(backend.Optimize());
  transform::Rigid3d pose;
  ASSERT_TRUE(backend.Pose(1, &pose));
  EXPECT_NEAR(pose.translation().x(), 1.0, 1e-5);
  EXPECT_NEAR(pose.translation().y(), 0.0, 1e-5);
  ASSERT_TRUE(backend.Pose(0, &pose));
  EXPECT_NEAR(pose.translation().norm(), 0.0, 1e-8);

  ASSERT_TRUE(backend.AddNode(
      2, transform::Rigid3d::Translation(Eigen::Vector3d(2.4, -0.3, 0.0)),
      false));
  factor.from_id = 1;
  factor.to_id = 2;
  backend.AddFactor(factor);
  ASSERT_TRUE(backend.Optimize());
  ASSERT_TRUE(backend.Pose(2, &pose));
  EXPECT_NEAR(pose.translation().x(), 2.0, 1e-5);
  EXPECT_NEAR(pose.translation().y(), 0.0, 1e-5);
}

}  // namespace
}  // namespace backend
}  // namespace atlas
