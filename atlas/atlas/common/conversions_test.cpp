#include "common/conversions.hpp"

#include <gtest/gtest.h>

namespace atlas {
namespace common {
namespace {

TEST(Conversions, VectorRoundTrip) {
  const Eigen::Vector3d gravity(0.0, 0.0, -9.8);
  const Eigen::Vector3d back = ToEigen(ToProto(gravity));
  EXPECT_TRUE(back.isApprox(gravity));
}

TEST(Conversions, ZeroQuaternionBecomesIdentity) {
  const proto::Quaterniond zero;
  EXPECT_TRUE(ToEigen(zero).isApprox(Eigen::Quaterniond::Identity()));
}

TEST(Conversions, Rigid3dNormalizesRotation) {
  proto::Rigid3d pose;
  pose.mutable_translation()->set_x(1.5);
  pose.mutable_rotation()->set_w(2.0);
  const Eigen::Isometry3d eigen = ToEigen(pose);
  EXPECT_DOUBLE_EQ(eigen.translation().x(), 1.5);
  EXPECT_TRUE(eigen.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-12));

  const proto::Rigid3d restored = ToProto(eigen);
  EXPECT_NEAR(restored.rotation().w(), 1.0, 1e-12);
  EXPECT_NEAR(restored.translation().x(), 1.5, 1e-12);
}

}  // namespace
}  // namespace common
}  // namespace atlas
