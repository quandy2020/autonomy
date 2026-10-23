#include "common/math/math.hpp"

#include <gtest/gtest.h>

namespace atlas {
namespace common {
namespace {

TEST(Math, SkewIsCrossProduct) {
  const Eigen::Vector3d v(1.0, 2.0, 3.0);
  const Eigen::Vector3d w(-1.0, 0.5, 4.0);
  EXPECT_TRUE(v.cross(w).isApprox(SkewSymmetric(v) * w));
}

TEST(Math, ExpLogSO3RoundTrip) {
  const Eigen::Vector3d angle(0.3, -0.2, 0.4);
  EXPECT_TRUE(ExpSO3(Eigen::Vector3d::Zero()).isIdentity(1e-12));
  EXPECT_TRUE(LogSO3(ExpSO3(angle)).isApprox(angle, 1e-9));
}

TEST(Math, ExpLogSE3RoundTrip) {
  Eigen::Matrix<double, 6, 1> twist;
  twist << 0.2, -0.1, 0.3, 1.0, 0.5, -0.25;
  const Eigen::Matrix<double, 6, 1> recovered = LogSE3(ExpSE3(twist));
  EXPECT_TRUE(recovered.isApprox(twist, 1e-9));
  EXPECT_TRUE(ExpSE3(Eigen::Matrix<double, 6, 1>::Zero())
                  .isApprox(Eigen::Isometry3d::Identity(), 1e-12));
}

}  // namespace
}  // namespace common
}  // namespace atlas
