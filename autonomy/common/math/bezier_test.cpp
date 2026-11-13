/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "autonomy/common/math/bezier.hpp"

#include <Eigen/Core>

#include "gtest/gtest.h"

namespace autonomy {
namespace common {
namespace math {
namespace {

double Horner(const std::array<double, 4>& coefficients, double u) {
    return coefficients[0] + u * (coefficients[1] +
                                  u * (coefficients[2] + u * coefficients[3]));
}

}  // namespace

TEST(CubicBezier, InterpolatesEndpoints) {
    const auto curve = CubicBezier<Eigen::Vector2d>::FromControlPoints(
        Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 2.0),
        Eigen::Vector2d(3.0, 2.0), Eigen::Vector2d(4.0, 0.0));
    const Eigen::Vector2d start = curve.Evaluate(0.0);
    const Eigen::Vector2d finish = curve.Evaluate(1.0);
    EXPECT_NEAR(start.x(), 0.0, 1e-12);
    EXPECT_NEAR(start.y(), 0.0, 1e-12);
    EXPECT_NEAR(finish.x(), 4.0, 1e-12);
    EXPECT_NEAR(finish.y(), 0.0, 1e-12);
}

TEST(CubicBezier, ClampsParameter) {
    const auto curve = CubicBezier<double>::FromControlPoints(0.0, 1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(curve.Evaluate(-2.0), curve.Evaluate(0.0));
    EXPECT_DOUBLE_EQ(curve.Evaluate(5.0), curve.Evaluate(1.0));
}

TEST(CubicBezier, StraightLineWhenHandlesCollinear) {
    const auto curve = CubicBezier<double>::FromControlPoints(0.0, 1.0, 2.0, 3.0);
    for (int i = 0; i <= 10; ++i) {
        const double u = 0.1 * i;
        EXPECT_NEAR(curve.Evaluate(u), 3.0 * u, 1e-12);
    }
}

TEST(CubicBezier, SamplesInclusiveCount) {
    const auto curve = CubicBezier<double>::FromControlPoints(0.0, 0.0, 1.0, 1.0);
    const auto samples = curve.Sample(5);
    ASSERT_EQ(samples.size(), 5u);
    EXPECT_NEAR(samples.front(), 0.0, 1e-12);
    EXPECT_NEAR(samples.back(), 1.0, 1e-12);
}

TEST(CubicBezier, MonomialRoundTripMatchesHorner) {
    const std::array<double, 4> coefficients = {1.0, -0.5, 0.25, 0.125};
    const auto curve = CubicBezier<double>::FromMonomial(coefficients);
    const auto recovered = curve.ToMonomial();
    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(recovered[static_cast<std::size_t>(i)],
                    coefficients[static_cast<std::size_t>(i)], 1e-12);
    }
    for (int i = 0; i <= 20; ++i) {
        const double u = 0.05 * i;
        EXPECT_NEAR(curve.Evaluate(u), Horner(coefficients, u), 1e-12);
    }
}

TEST(CubicBezier, VectorMonomialMatchesComponentwise) {
    std::array<Eigen::Vector2d, 4> coefficients;
    coefficients[0] = Eigen::Vector2d(0.0, 1.0);
    coefficients[1] = Eigen::Vector2d(3.0, 0.0);
    coefficients[2] = Eigen::Vector2d(0.0, -3.0);
    coefficients[3] = Eigen::Vector2d(0.0, 0.0);
    const auto curve = CubicBezier<Eigen::Vector2d>::FromMonomial(coefficients);
    const Eigen::Vector2d mid = curve.Evaluate(0.5);
    EXPECT_NEAR(mid.x(), 0.0 + 3.0 * 0.5, 1e-12);
    EXPECT_NEAR(mid.y(), 1.0 - 3.0 * 0.25, 1e-12);
}

TEST(CubicBezier, FirstDerivativeMatchesFiniteDifference) {
    const auto curve = CubicBezier<double>::FromControlPoints(0.0, 2.0, 1.0, 4.0);
    const double u = 0.37;
    const double step = 1e-6;
    const double numerical =
        (curve.Evaluate(u + step) - curve.Evaluate(u - step)) / (2.0 * step);
    EXPECT_NEAR(curve.EvaluateDerivative(1, u), numerical, 1e-6);
}

TEST(CubicBezier, ThirdDerivativeIsConstantJerk) {
    const auto curve = CubicBezier<double>::FromControlPoints(0.0, 1.0, 0.0, 1.0);
    const double jerk = curve.EvaluateDerivative(3, 0.0);
    EXPECT_NEAR(curve.EvaluateDerivative(3, 0.5), jerk, 1e-12);
    EXPECT_NEAR(curve.EvaluateDerivative(3, 1.0), jerk, 1e-12);
    EXPECT_NEAR(curve.EvaluateDerivative(4, 0.3), 0.0, 1e-12);
}

TEST(CubicBezier, DurationScalesVelocity) {
    const auto curve = CubicBezier<double>::FromControlPoints(0.0, 0.0, 1.0, 1.0);
    const double duration = 2.0;
    const double u_velocity = curve.EvaluateDerivative(1, 0.5);
    const double time_velocity =
        curve.EvaluateWithDuration(1, 0.5 * duration, duration);
    EXPECT_NEAR(time_velocity, u_velocity / duration, 1e-12);
}

TEST(CubicBezier, SamplesStayInControlBoundingInterval) {
    const auto curve = CubicBezier<double>::FromControlPoints(-1.0, 4.0, -2.0, 1.0);
    const double low = -2.0;
    const double high = 4.0;
    for (const double value : curve.Sample(32)) {
        EXPECT_GE(value, low - 1e-12);
        EXPECT_LE(value, high + 1e-12);
    }
}

TEST(PiecewiseCubicBezier, JoinsTwoSegments) {
    CubicBezier<double> first =
        CubicBezier<double>::FromControlPoints(0.0, 0.0, 1.0, 1.0);
    CubicBezier<double> second =
        CubicBezier<double>::FromControlPoints(1.0, 1.0, 2.0, 2.0);
    PiecewiseCubicBezier<double> path({first, second});
    EXPECT_EQ(path.segment_count(), 2);
    EXPECT_NEAR(path.Evaluate(0.0), 0.0, 1e-12);
    EXPECT_NEAR(path.Evaluate(0.5), 1.0, 1e-12);
    EXPECT_NEAR(path.Evaluate(1.0), 2.0, 1e-12);
}

TEST(PiecewiseCubicBezier, EmptyReturnsZero) {
    PiecewiseCubicBezier<Eigen::Vector3d> empty;
    const Eigen::Vector3d value = empty.Evaluate(0.3);
    EXPECT_NEAR(value.norm(), 0.0, 1e-12);
}

}  // namespace math
}  // namespace common
}  // namespace autonomy
