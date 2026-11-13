/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "autonomy/common/math/bspline.hpp"

#include "autonomy/common/math/bezier.hpp"

#include <Eigen/Core>

#include "gtest/gtest.h"

namespace autonomy {
namespace common {
namespace math {

TEST(ClampedCubicBSpline, InterpolatesEndpoints) {
    std::vector<Eigen::Vector3d> controls = {
        Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(2.0, 1.0, 0.0), Eigen::Vector3d(3.0, 1.0, 0.0)};
    ClampedCubicBSpline<Eigen::Vector3d> curve(controls);
    const Eigen::Vector3d start = curve.Evaluate(0.0);
    const Eigen::Vector3d finish = curve.Evaluate(1.0);
    EXPECT_NEAR(start.x(), 0.0, 1e-9);
    EXPECT_NEAR(start.y(), 0.0, 1e-9);
    EXPECT_NEAR(finish.x(), 3.0, 1e-9);
    EXPECT_NEAR(finish.y(), 1.0, 1e-9);
}

TEST(ClampedCubicBSpline, FourPointsMatchCubicBezier) {
    const Eigen::Vector2d p0(0.0, 0.0);
    const Eigen::Vector2d p1(1.0, 2.0);
    const Eigen::Vector2d p2(3.0, 2.0);
    const Eigen::Vector2d p3(4.0, 0.0);
    const auto bezier =
        CubicBezier<Eigen::Vector2d>::FromControlPoints(p0, p1, p2, p3);
    ClampedCubicBSpline<Eigen::Vector2d> spline({p0, p1, p2, p3});
    for (int i = 0; i <= 20; ++i) {
        const double u = 0.05 * i;
        const Eigen::Vector2d a = bezier.Evaluate(u);
        const Eigen::Vector2d b = spline.Evaluate(u);
        EXPECT_NEAR(a.x(), b.x(), 1e-9) << "u=" << u;
        EXPECT_NEAR(a.y(), b.y(), 1e-9) << "u=" << u;
    }
}

TEST(ClampedCubicBSpline, ClampsParameter) {
    ClampedCubicBSpline<double> curve({0.0, 1.0, 2.0, 3.0});
    EXPECT_DOUBLE_EQ(curve.Evaluate(-1.0), curve.Evaluate(0.0));
    EXPECT_DOUBLE_EQ(curve.Evaluate(2.0), curve.Evaluate(1.0));
}

TEST(ClampedCubicBSpline, EmptyAndLinearFallback) {
    ClampedCubicBSpline<double> empty;
    EXPECT_NEAR(empty.Evaluate(0.3), 0.0, 1e-12);

    ClampedCubicBSpline<double> line({0.0, 4.0});
    EXPECT_NEAR(line.Evaluate(0.25), 1.0, 1e-12);
    EXPECT_NEAR(line.EvaluateDerivative(1, 0.5), 4.0, 1e-12);
}

TEST(ClampedCubicBSpline, SampleInclusive) {
    ClampedCubicBSpline<double> curve({0.0, 0.0, 1.0, 1.0});
    const auto samples = curve.Sample(5);
    ASSERT_EQ(samples.size(), 5u);
    EXPECT_NEAR(samples.front(), 0.0, 1e-12);
    EXPECT_NEAR(samples.back(), 1.0, 1e-12);
}

TEST(ClampedCubicBSpline, FirstDerivativeMatchesFiniteDifference) {
    ClampedCubicBSpline<double> curve({0.0, 2.0, 1.0, 4.0, 3.0, 5.0, 4.0, 6.0});
    const double u = 0.41;
    const double step = 1e-6;
    const double numerical =
        (curve.Evaluate(u + step) - curve.Evaluate(u - step)) / (2.0 * step);
    EXPECT_NEAR(curve.EvaluateDerivative(1, u), numerical, 5e-5);
}

TEST(ClampedCubicBSpline, SecondDerivativeContinuousAtInteriorKnot) {
    ClampedCubicBSpline<double> curve({0.0, 1.0, 4.0, 2.0, 5.0, 3.0, 6.0, 4.0});
    const double knot = 0.2;
    const double left = curve.EvaluateDerivative(2, knot - 1e-8);
    const double right = curve.EvaluateDerivative(2, knot + 1e-8);
    EXPECT_NEAR(left, right, 1e-4);
}

TEST(ClampedCubicBSpline, CachesKnots) {
    ClampedCubicBSpline<Eigen::Vector3d> curve({
        Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::Ones()});
    EXPECT_EQ(curve.control_point_count(), 5);
    EXPECT_FALSE(curve.knots().empty());
    EXPECT_NEAR(curve.knots().front(), 0.0, 1e-12);
    EXPECT_NEAR(curve.knots().back(), 1.0, 1e-12);
}

TEST(ResamplePolylineArcLength, EqualSpacing) {
    std::vector<Eigen::Vector3d> points = {
        Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 1.0, 0.0)};
    const auto samples = ResamplePolylineArcLength(points, 5);
    ASSERT_EQ(samples.size(), 5u);
    EXPECT_NEAR(samples.front().x(), 0.0, 1e-9);
    EXPECT_NEAR(samples.back().x(), 1.0, 1e-9);
    EXPECT_NEAR(samples.back().y(), 1.0, 1e-9);
}

TEST(ResamplePolylineArcLength, EmptyAndDegenerate) {
    EXPECT_TRUE(ResamplePolylineArcLength(std::vector<double>{}, 4).empty());
    const auto same = ResamplePolylineArcLength(std::vector<double>{1.0, 1.0}, 3);
    ASSERT_EQ(same.size(), 3u);
    EXPECT_NEAR(same[1], 1.0, 1e-12);
}

}  // namespace math
}  // namespace common
}  // namespace autonomy
