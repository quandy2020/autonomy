#include "atlas/common/optimization/optimizer.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "atlas/map/map.hpp"

namespace autonomy::localization::atlas {
namespace {

TEST(OptimizerTest, FullInertialBaRejectsInvalidMapsWithoutMutation) {
    EXPECT_FALSE(Optimizer::FullInertialBA(nullptr, 1));

    Map map;
    const int change_index = map.GetMapChangeIndex();
    EXPECT_FALSE(Optimizer::FullInertialBA(&map, 1));
    EXPECT_EQ(map.GetMapChangeIndex(), change_index);
    EXPECT_EQ(map.KeyFramesInMap(), 0U);
    EXPECT_EQ(map.MapPointsInMap(), 0U);
}

TEST(OptimizerTest, RelativeGeometryAcceptsLargeCoordinatesAndRigidMotion) {
    const std::vector<Eigen::Vector3d> current = {
        Eigen::Vector3d(1.0e9, -2.0e9, 3.0e9),
        Eigen::Vector3d(1.0e9 + 2.0, -2.0e9 + 1.0, 3.0e9 - 1.0),
        Eigen::Vector3d(1.0e9 - 3.0, -2.0e9 + 4.0, 3.0e9 + 2.0)};
    constexpr double kAngle = 0.7;
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(kAngle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d translation(-4.0e8, 7.0e8, 2.0e8);
    std::vector<Eigen::Vector3d> optimized;
    optimized.reserve(current.size());
    for (const Eigen::Vector3d& point : current) {
        optimized.push_back(rotation * point + translation);
    }

    EXPECT_TRUE(
        Optimizer::IsRelativeGeometryUpdateReasonable(current, optimized));
}

TEST(OptimizerTest, RelativeGeometryRejectsFiniteCatastrophicScaleChange) {
    const std::vector<Eigen::Vector3d> current = {
        Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 2.0, 0.0)};
    std::vector<Eigen::Vector3d> optimized;
    optimized.reserve(current.size());
    for (const Eigen::Vector3d& point : current) {
        optimized.push_back(1000.0 * point + Eigen::Vector3d(5.0, 6.0, 7.0));
    }

    EXPECT_FALSE(
        Optimizer::IsRelativeGeometryUpdateReasonable(current, optimized));
}

TEST(OptimizerTest, RelativeMagnitudeRejectsFiniteVelocityExplosion) {
    const std::vector<Eigen::Vector3d> current = {
        Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 2.0, 0.0)};
    const std::vector<Eigen::Vector3d> optimized = {
        1000.0 * current[0], 1000.0 * current[1]};

    EXPECT_FALSE(
        Optimizer::IsRelativeMagnitudeUpdateReasonable(current, optimized));
}

}  // namespace
}  // namespace autonomy::localization::atlas
