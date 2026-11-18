/* Copyright 2026 The Openbot Authors. */
/** @file trajectory_time_scaler.cpp
 * @brief Physical-boundary-preserving retiming with final convex-hull checks.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/trajectory_time_scaler.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

namespace autonomy::control::controller::policy_controller::sand_planner {
namespace {
/** @brief Finite strictly positive quantities are safe denominators. */
bool Positive(double value) { return std::isfinite(value) && value > 0.0; }
/** @brief XYZ norm without mixing meters and radians. */
double LinearNorm(const TrajectoryVector& value) {
    return std::hypot(std::hypot(value[0], value[1]), value[2]);
}
/** @brief A finite bound is nonnegative and no larger than its physical limit. */
bool Within(double bound, double limit) {
    return std::isfinite(bound) && bound >= 0.0 && bound <= limit;
}
}  // namespace

bool TrajectoryLimits::Valid() const {
    return Positive(linear_velocity) && Positive(linear_acceleration) && Positive(linear_jerk) &&
           Positive(yaw_velocity) && Positive(yaw_acceleration) && Positive(yaw_jerk);
}

bool TrajectoryLimits::Contains(const TrajectoryDerivativeBounds& b) const {
    return Valid() && Within(b.linear_velocity, linear_velocity) &&
           Within(b.linear_acceleration, linear_acceleration) && Within(b.linear_jerk, linear_jerk) &&
           Within(b.yaw_velocity, yaw_velocity) && Within(b.yaw_acceleration, yaw_acceleration) &&
           Within(b.yaw_jerk, yaw_jerk);
}

TimeScalingResult ScaleTrajectory(const TimedBSpline& proposal, const TrajectoryState& boundary,
                                 const TrajectoryLimits& limits, const TimeScalingOptions& options) {
    TimeScalingResult result;
    if (!boundary.Finite() || !limits.Valid() || !options.maximum_iterations ||
        options.maximum_iterations > 1024 || !Positive(options.maximum_duration) ||
        !std::isfinite(options.growth_margin) || options.growth_margin <= 1.0) {
        result.reason = "invalid boundary, limits or time-scaling budget";
        return result;
    }
    if (LinearNorm(boundary.velocity) > limits.linear_velocity ||
        LinearNorm(boundary.acceleration) > limits.linear_acceleration ||
        std::abs(boundary.velocity[3]) > limits.yaw_velocity ||
        std::abs(boundary.acceleration[3]) > limits.yaw_acceleration) {
        result.reason = "fixed physical boundary exceeds derivative limits";
        return result;
    }
    const auto& original = proposal.data().timing;
    const double maximum = std::min(options.maximum_duration, original.valid_until - original.start_stamp);
    double duration = original.duration;
    try {
        for (unsigned i = 0; i < options.maximum_iterations; ++i) {
            if (!std::isfinite(duration) || duration > maximum ||
                original.start_stamp + duration > original.valid_until) {
                result.reason = "retiming exceeds duration or authorized validity";
                return result;
            }
            auto timing = original;
            timing.duration = duration;
            auto trial = proposal.WithTiming(timing).WithStartState(boundary);
            ++result.iterations;
            const auto bounds = trial.DerivativeBounds();
            if (limits.Contains(bounds)) {
                result.trajectory = std::move(trial);
                return result;
            }
            // These ratios propose a duration; the next anchored trial is always checked anew.
            const double ratio = std::max({bounds.linear_velocity / limits.linear_velocity,
                std::sqrt(bounds.linear_acceleration / limits.linear_acceleration),
                std::cbrt(bounds.linear_jerk / limits.linear_jerk),
                bounds.yaw_velocity / limits.yaw_velocity,
                std::sqrt(bounds.yaw_acceleration / limits.yaw_acceleration),
                std::cbrt(bounds.yaw_jerk / limits.yaw_jerk), 1.0});
            duration *= ratio * options.growth_margin;
        }
        result.reason = "retiming exhausted its bounded iteration budget";
    } catch (const std::exception& exception) {
        result.reason = exception.what();
    }
    return result;
}

}  // namespace autonomy::control::controller::policy_controller::sand_planner
