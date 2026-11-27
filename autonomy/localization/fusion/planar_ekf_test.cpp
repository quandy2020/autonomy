// Copyright 2026 The Openbot Authors
#include <cmath>
#include <limits>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
#include "autonomy/localization/fusion/planar_ekf.hpp"

namespace autonomy::localization::fusion {
TEST(PlanarEkfTest, StraightMotionAndCovariance) {
    PlanarEkf filter({});
    const Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * .001;
    ASSERT_TRUE(filter.AddWheel(0., {1, 0, 0}, r));
    for (int i = 1; i <= 1000; ++i) {
        const double time = i * .01;
        if (i % 10 == 0) ASSERT_TRUE(filter.AddWheel(time, {1, 0, 0}, r));
        ASSERT_TRUE(filter.AddGyro(time, 0., .0001));
    }
    EXPECT_NEAR(filter.state()(0), 10., 1e-6);
    EXPECT_NEAR(filter.state()(1), 0., 1e-6);
    EXPECT_NEAR(filter.state()(2), 0., 1e-6);
    const auto p = filter.covariance();
    EXPECT_TRUE(p.allFinite());
    EXPECT_TRUE(p.isApprox(p.transpose(), 1e-10));
    EXPECT_GT((Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>>(p).eigenvalues().minCoeff()), -1e-9);
}

TEST(PlanarEkfTest, GyroCorrectsWheelYawAndFollowsCircle) {
    PlanarEkf filter({});
    Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * .001;
    r(2, 2) = 10.;
    ASSERT_TRUE(filter.AddWheel(0., {1, 0, .2}, r));
    ASSERT_TRUE(filter.AddGyro(0., 1., 1e-6));
    for (int i = 1; i <= 1000; ++i) {
        const double time = i * .01;
        if (i % 10 == 0) ASSERT_TRUE(filter.AddWheel(time, {1, 0, .2}, r));
        ASSERT_TRUE(filter.AddGyro(time, 1., 1e-6));
    }
    EXPECT_NEAR(filter.state()(0), std::sin(10.), .01);
    EXPECT_NEAR(filter.state()(1), 1. - std::cos(10.), .01);
    EXPECT_NEAR(filter.state()(2), std::atan2(std::sin(10.), std::cos(10.)), .001);
    EXPECT_NEAR(filter.state()(5), 1., .001);
}

TEST(PlanarEkfTest, InvalidDuplicateAndOutOfOrderMeasurementsDoNotChangeState) {
    PlanarEkf filter({});
    EXPECT_FALSE(filter.AddGyro(0., 0., .01));
    EXPECT_THROW(filter.state(), std::logic_error);
    const Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * .01;
    ASSERT_TRUE(filter.AddWheel(1., {1, 0, 0}, r));
    ASSERT_TRUE(filter.AddGyro(1.1, .2, .01));
    const auto original = filter.state();
    const auto p = filter.covariance();
    EXPECT_FALSE(filter.AddWheel(1., {9, 0, 0}, r));
    EXPECT_FALSE(filter.AddGyro(1.1, 9., .01));
    EXPECT_FALSE(filter.AddGyro(.9, 9., .01));
    EXPECT_FALSE(filter.AddGyro(1.2, std::numeric_limits<double>::quiet_NaN(), .01));
    EXPECT_FALSE(filter.AddGyro(1.2, 0., 0.));
    EXPECT_FALSE(filter.AddWheel(1.2, {1, 0, 0}, Eigen::Matrix3d::Zero()));
    Eigen::Matrix3d bad = r; bad(0, 1) = 1.;
    EXPECT_FALSE(filter.AddWheel(1.2, {1, 0, 0}, bad));
    EXPECT_TRUE(filter.state().isApprox(original, 0.));
    EXPECT_TRUE(filter.covariance().isApprox(p, 0.));
}

TEST(PlanarEkfTest, WheelOutageStopsPredictionAndResumePreservesPose) {
    PlanarEkf filter({});
    const Eigen::Matrix3d r = Eigen::Matrix3d::Identity() * .01;
    ASSERT_TRUE(filter.AddWheel(0., {1, 0, 0}, r));
    ASSERT_TRUE(filter.AddGyro(.4, 0., .01));
    const auto original = filter.state();
    EXPECT_FALSE(filter.AddGyro(.6, 0., .01));
    ASSERT_TRUE(filter.AddWheel(2., {0, 0, 0}, r));
    EXPECT_NEAR(filter.state()(0), original(0), 1e-9);
    EXPECT_GT(filter.covariance()(0, 0), 1.);
    ASSERT_TRUE(filter.AddGyro(2.1, 0., .01));
    EXPECT_NEAR(filter.state()(0), original(0), 1e-9);
}
}  // namespace autonomy::localization::fusion
