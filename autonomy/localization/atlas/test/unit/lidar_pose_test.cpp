/*
 * Copyright 2026 The Openbot Authors
 *
 * Ceres point-to-plane pose optimization.
 */

#include "autonomy/localization/atlas/backend/optimizer.hpp"

#include "gtest/gtest.h"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {
namespace {

TEST(LidarPoseTest, PointToPlanePullsHeight) {
    std::vector<LidarPlaneFactor> factors;
    for (int x = -2; x <= 2; ++x) {
        for (int y = -2; y <= 2; ++y) {
            LidarPlaneFactor factor;
            factor.point_body = Vec3(0.4 * x, 0.4 * y, 0.0);
            factor.plane_point = Vec3::Zero();
            factor.plane_normal = Vec3::UnitZ();
            factor.sqrt_info = 1.0;
            factors.push_back(factor);
        }
    }
    SE3 pose = SE3Identity();
    pose.translation() = Vec3(0.0, 0.0, 0.3);
    const int inliers =
        Optimizer::OptimizeLidarPose(&pose, factors, nullptr, 0.0, 15);
    EXPECT_GE(inliers, 5);
    EXPECT_NEAR(pose.translation().z(), 0.0, 1e-2);
}

}  // namespace
}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
