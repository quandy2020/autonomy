/* Copyright 2026 The Openbot Authors. */
/** @file timed_bspline.cpp
 * @brief Execution metadata and physical-time scaling over common/math splines.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/timed_bspline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace sand_planner {
namespace {

/** @brief All four coordinates must be finite before algebra or indexing. */
bool Finite(const TrajectoryVector& value) {
    return std::all_of(value.begin(), value.end(), [](double x) { return std::isfinite(x); });
}

/** @brief Stable XYZ norm, deliberately independent of angular units. */
double LinearNorm(const TrajectoryVector& value) {
    return std::hypot(std::hypot(value[0], value[1]), value[2]);
}

/** @brief Reject malformed cubic input before any recursive numerical work. */
void Validate(const BSplineData& data) {
    const auto count = data.control_points.size();
    if (count < 4 || count > TimedBSpline::kMaximumControlPoints || data.knots.size() != count + 4)
        throw std::invalid_argument("cubic spline needs 4..64 controls and N+4 knots");
    for (const auto& control : data.control_points)
        if (!Finite(control)) throw std::invalid_argument("nonfinite control point");
    for (unsigned i = 0; i < 4; ++i)
        if (data.knots[i] != 0.0 || data.knots[count + i] != 1.0)
            throw std::invalid_argument("cubic endpoint knots must be clamped at zero and one");
    for (std::size_t i = 4; i <= count; ++i)
        if (!std::isfinite(data.knots[i]) ||
            data.knots[i] - data.knots[i - 1] < TimedBSpline::kMinimumKnotSpan)
            throw std::invalid_argument("interior knots must be simple with positive bounded spans");
    const auto& t = data.timing;
    if (t.frame_id.empty() || t.execution_id.empty() || !t.alignment_epoch || !t.reference_version ||
        !std::isfinite(t.start_stamp) || t.start_stamp < 0.0 || !std::isfinite(t.duration) || t.duration <= 0.0 ||
        !std::isfinite(t.valid_until) || t.valid_until <= t.start_stamp ||
        !std::isfinite(t.start_stamp + t.duration) || t.start_stamp + t.duration <= t.start_stamp)
        throw std::invalid_argument("invalid trajectory identity, physical timing or validity");
}

}  // namespace

bool TrajectoryState::Finite() const {
    return sand_planner::Finite(position) && sand_planner::Finite(velocity) &&
        sand_planner::Finite(acceleration) && sand_planner::Finite(jerk);
}

TimedBSpline::TimedBSpline(BSplineData data) : data_(std::move(data)) {
    Validate(data_);
    for (unsigned axis = 0; axis < 4; ++axis) {
        std::vector<double> controls;
        controls.reserve(data_.control_points.size());
        for (const auto& point : data_.control_points) controls.push_back(point[axis]);
        curves_[axis] = ::autonomy::common::math::ClampedCubicBSpline<double>(
            std::move(controls), data_.knots);
    }
}

double TimedBSpline::EndStamp() const {
    return std::min(data_.timing.start_stamp + data_.timing.duration, data_.timing.valid_until);
}

TrajectoryState TimedBSpline::Evaluate(double stamp) const {
    const auto& time = data_.timing;
    if (!std::isfinite(stamp) || stamp < time.start_stamp || stamp > EndStamp())
        throw std::out_of_range("reference evaluation outside its physical validity");
    const double u = std::clamp((stamp - time.start_stamp) / time.duration, 0.0, 1.0);
    TrajectoryState result;
    for (unsigned axis = 0; axis < 4; ++axis) {
        result.position[axis] = curves_[axis].Evaluate(u);
        result.velocity[axis] = curves_[axis].EvaluateDerivative(1, u) / time.duration;
        result.acceleration[axis] = curves_[axis].EvaluateDerivative(2, u) / time.duration / time.duration;
        result.jerk[axis] = curves_[axis].EvaluateDerivative(3, u) / time.duration / time.duration / time.duration;
    }
    if (!result.Finite()) throw std::overflow_error("nonfinite physical spline evaluation");
    return result;
}

std::vector<TrajectoryVector> TimedBSpline::DerivativeControlPoints(unsigned order) const {
    if (!order || order > 3) throw std::invalid_argument("derivative order must be 1..3");
    std::vector<TrajectoryVector> controls(data_.control_points.size() - order);
    for (unsigned axis = 0; axis < 4; ++axis) {
        const auto derived = curves_[axis].DerivativeControlPoints(static_cast<int>(order));
        for (std::size_t i = 0; i < derived.size(); ++i) {
            double value = derived[i];
            for (unsigned j = 0; j < order; ++j) value /= data_.timing.duration;
            controls[i][axis] = value;
        }
    }
    for (const auto& control : controls)
        if (!Finite(control)) throw std::overflow_error("nonfinite physical spline derivative");
    return controls;
}

TrajectoryDerivativeBounds TimedBSpline::DerivativeBounds() const {
    TrajectoryDerivativeBounds result;
    for (unsigned order = 1; order <= 3; ++order) {
        double linear = 0.0, angular = 0.0;
        for (const auto& control : DerivativeControlPoints(order)) {
            linear = std::max(linear, LinearNorm(control));
            angular = std::max(angular, std::abs(control[3]));
        }
        if (!std::isfinite(linear)) throw std::overflow_error("derivative norm overflow");
        if (order == 1) { result.linear_velocity = linear; result.yaw_velocity = angular; }
        else if (order == 2) { result.linear_acceleration = linear; result.yaw_acceleration = angular; }
        else { result.linear_jerk = linear; result.yaw_jerk = angular; }
    }
    return result;
}

TimedBSpline TimedBSpline::WithTiming(TrajectoryTiming timing) const {
    auto data = data_;
    data.timing = std::move(timing);
    return TimedBSpline(std::move(data));
}

TimedBSpline TimedBSpline::WithStartState(const TrajectoryState& state) const {
    if (!state.Finite()) throw std::invalid_argument("nonfinite trajectory boundary state");
    auto data = data_;
    const double duration = data.timing.duration;
    for (unsigned axis = 0; axis < 4; ++axis) {
        const double first = state.velocity[axis] * duration;
        const double second = state.acceleration[axis] * duration * duration;
        const auto controls = curves_[axis].StartControlPoints(state.position[axis], first, second);
        for (std::size_t i = 0; i < controls.size(); ++i) data.control_points[i][axis] = controls[i];
    }
    return TimedBSpline(std::move(data));
}

}  // namespace sand_planner
}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
