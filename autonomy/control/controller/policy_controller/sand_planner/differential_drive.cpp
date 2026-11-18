/* Copyright 2026 The Openbot Authors. */
/** @file differential_drive.cpp
 * @brief Knot-aware continuous bounds for wheel speed, acceleration and jerk.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/differential_drive.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <vector>

namespace autonomy::control::controller::policy_controller::sand_planner {
namespace {
/** @brief A finite positive parameter, including physical actuator limits. */
bool Positive(double value) { return std::isfinite(value) && value > 0.0; }
/** @brief Nonnegative model residuals and allowed reverse velocity. */
bool Nonnegative(double value) { return std::isfinite(value) && value >= 0.0; }

/** @brief One interval contained wholly in one original cubic knot span. */
struct Interval {
    double begin;  ///< Inclusive physical beginning; jerk uses the span's right side.
    double end;    ///< Inclusive physical end; the next span checks its own jerk side.
    double knot_begin; ///< Original normalized knot span; subdivisions cannot cross it.
    double knot_end;   ///< Original normalized knot span upper boundary.
};

/** @brief Body-forward/lateral physical derivatives evaluated at one instant. */
struct BodyState {
    double forward;             ///< Projected XY velocity, m/s.
    double lateral;             ///< Projected lateral XY velocity, m/s.
    double forward_acceleration; ///< Time derivative of forward, m/s^2.
    double lateral_acceleration; ///< Time derivative of lateral, m/s^2.
    double forward_jerk;        ///< Time derivative of forward acceleration, m/s^3.
};

/** @brief Exact derivatives of the rotating body basis, including yaw couplings. */
BodyState Project(const TrajectoryState& state) {
    // Bounded argument reduction is part of the numerical model. An execution
    // owner can start a new unwrapped-yaw scope instead of passing huge angles.
    if (std::abs(state.position[3]) > 1e6)
        throw std::runtime_error("yaw exceeds differential-drive numerical model");
    const double c = std::cos(state.position[3]), s = std::sin(state.position[3]);
    const double forward = c * state.velocity[0] + s * state.velocity[1];
    const double lateral = -s * state.velocity[0] + c * state.velocity[1];
    const double forward_a = c * state.acceleration[0] + s * state.acceleration[1];
    const double lateral_a = -s * state.acceleration[0] + c * state.acceleration[1];
    const double w = state.velocity[3], alpha = state.acceleration[3];
    return {forward, lateral, forward_a + w * lateral, lateral_a - w * forward,
            c * state.jerk[0] + s * state.jerk[1] + 2.0 * w * lateral_a +
                alpha * lateral - w * w * forward};
}

/** @brief Body second derivative and within-span snap bounds from derivative hulls. */
struct ModelBounds {
    double second;        ///< Bounds second derivative of either body velocity component.
    double snap;          ///< Bounds third derivative of body-forward velocity within one span.
    double vertical_a;    ///< Absolute Z acceleration hull, m/s^2.
    double vertical_j;    ///< Absolute Z jerk hull, m/s^3.
    double wheel_j;       ///< Global wheel jerk envelope, m/s^3.
};

/** @brief XYZ/yaw cubic snap vanishes inside each actual knot span. */
ModelBounds Bounds(const TimedBSpline& trajectory, double half_track) {
    const auto hull = trajectory.DerivativeBounds();
    const double v = hull.linear_velocity, a = hull.linear_acceleration, j = hull.linear_jerk;
    const double w = hull.yaw_velocity, alpha = hull.yaw_acceleration, beta = hull.yaw_jerk;
    ModelBounds result;
    result.second = j + 2.0 * w * a + alpha * v + w * w * v;
    result.snap = 3.0 * w * j + 3.0 * alpha * a + beta * v + 3.0 * w * w * a +
                  3.0 * w * alpha * v + w * w * w * v;
    result.vertical_a = 0.0;
    result.vertical_j = 0.0;
    for (const auto& control : trajectory.DerivativeControlPoints(2))
        result.vertical_a = std::max(result.vertical_a, std::abs(control[2]));
    for (const auto& control : trajectory.DerivativeControlPoints(3))
        result.vertical_j = std::max(result.vertical_j, std::abs(control[2]));
    result.wheel_j = result.second + half_track * beta;
    if (!Nonnegative(result.second) || !Nonnegative(result.snap) || !Nonnegative(result.wheel_j) ||
        !Nonnegative(result.vertical_a) || !Nonnegative(result.vertical_j))
        throw std::runtime_error("differential-drive derivative envelope overflow");
    return result;
}

/** @brief Leaf bounds include all intermediate points; sampled feasibility alone is insufficient. */
struct Envelopes {
    double lateral_v;    ///< Whole-interval lateral velocity magnitude bound.
    double lateral_a;    ///< Whole-interval lateral acceleration magnitude bound.
    double vertical_v;   ///< Whole-interval vertical velocity magnitude bound.
    double forward_min;  ///< Whole-interval signed forward lower bound.
    std::array<double, 2> wheel_v; ///< Whole-interval left/right speed magnitude bounds.
    std::array<double, 2> wheel_a; ///< Whole-interval left/right acceleration magnitude bounds.
    std::array<double, 2> wheel_j; ///< Whole-interval left/right one-sided jerk magnitude bounds.
};

/** @brief Taylor envelopes around the midpoint using analytic remainder bounds. */
Envelopes Enclose(const TrajectoryState& state, const BodyState& body,
                  const ModelBounds& bounds, double half_track, double radius) {
    const double quadratic = radius * radius / 2.0;
    Envelopes e;
    e.lateral_v = std::abs(body.lateral) + std::abs(body.lateral_acceleration) * radius + bounds.second * quadratic;
    e.lateral_a = std::abs(body.lateral_acceleration) + bounds.second * radius;
    e.vertical_v = std::abs(state.velocity[2]) + std::abs(state.acceleration[2]) * radius +
                   bounds.vertical_j * quadratic;
    e.forward_min = body.forward - std::abs(body.forward_acceleration) * radius - bounds.second * quadratic;
    for (unsigned wheel = 0; wheel < 2; ++wheel) {
        const double offset = (wheel == 0 ? -half_track : half_track);
        const double v = body.forward + offset * state.velocity[3];
        const double a = body.forward_acceleration + offset * state.acceleration[3];
        const double j = body.forward_jerk + offset * state.jerk[3];
        e.wheel_v[wheel] = std::abs(v) + std::abs(a) * radius + bounds.wheel_j * quadratic;
        e.wheel_a[wheel] = std::abs(a) + std::abs(j) * radius + bounds.snap * quadratic;
        e.wheel_j[wheel] = std::abs(j) + bounds.snap * radius;
    }
    return e;
}

/** @brief Return the first violated envelope; nonfinite arithmetic always denies. */
std::string Violation(const Envelopes& e, const DifferentialDriveLimits& l) {
    const double tolerance = l.numerical_tolerance;
    if (!Nonnegative(e.lateral_v) || e.lateral_v > l.maximum_lateral_velocity + tolerance)
        return "lateral velocity exceeds differential-drive model";
    if (!Nonnegative(e.lateral_a) || e.lateral_a > l.maximum_lateral_acceleration + tolerance)
        return "lateral acceleration exceeds differential-drive model";
    if (!Nonnegative(e.vertical_v) || e.vertical_v > l.maximum_vertical_velocity + tolerance)
        return "vertical velocity exceeds planar differential-drive model";
    if (!std::isfinite(e.forward_min) || e.forward_min < -l.maximum_reverse_velocity - tolerance)
        return "reverse velocity exceeds differential-drive policy";
    for (unsigned wheel = 0; wheel < 2; ++wheel) {
        if (!Nonnegative(e.wheel_v[wheel]) || e.wheel_v[wheel] > l.wheel_velocity[wheel] + tolerance)
            return "wheel velocity exceeds calibrated limit";
        if (!Nonnegative(e.wheel_a[wheel]) || e.wheel_a[wheel] > l.wheel_acceleration[wheel] + tolerance)
            return "wheel acceleration exceeds calibrated limit";
        if (!Nonnegative(e.wheel_j[wheel]) || e.wheel_j[wheel] > l.wheel_jerk[wheel] + tolerance)
            return "wheel jerk exceeds calibrated limit";
    }
    return {};
}

/** @brief Accumulate only proved leaf envelopes; failures cannot certify these partial maxima. */
void Accumulate(const Envelopes& e, DifferentialDriveValidationResult* result) {
    result->lateral_velocity_bound = std::max(result->lateral_velocity_bound, e.lateral_v);
    result->lateral_acceleration_bound = std::max(result->lateral_acceleration_bound, e.lateral_a);
    result->vertical_velocity_bound = std::max(result->vertical_velocity_bound, e.vertical_v);
    for (unsigned wheel = 0; wheel < 2; ++wheel) {
        result->wheel_velocity_bounds[wheel] = std::max(result->wheel_velocity_bounds[wheel], e.wheel_v[wheel]);
        result->wheel_acceleration_bounds[wheel] = std::max(result->wheel_acceleration_bounds[wheel], e.wheel_a[wheel]);
        result->wheel_jerk_bounds[wheel] = std::max(result->wheel_jerk_bounds[wheel], e.wheel_j[wheel]);
    }
}
}  // namespace

bool DifferentialDriveLimits::Valid() const {
    if (!calibration_revision || !Positive(track_width) || !Nonnegative(maximum_lateral_velocity) ||
        !Nonnegative(maximum_lateral_acceleration) || !Nonnegative(maximum_vertical_velocity) ||
        !Nonnegative(maximum_reverse_velocity) || !Positive(numerical_tolerance) || numerical_tolerance > 1e-6)
        return false;
    for (unsigned wheel = 0; wheel < 2; ++wheel)
        if (!Positive(wheel_velocity[wheel]) || !Positive(wheel_acceleration[wheel]) || !Positive(wheel_jerk[wheel]))
            return false;
    return true;
}

DifferentialDriveValidationResult ValidateDifferentialDrive(
    const TimedBSpline& trajectory, const DifferentialDriveLimits& limits,
    const TrajectoryValidationOptions& options, const ExecutionClock& clock) {
    DifferentialDriveValidationResult result;
    if (!limits.Valid() || !clock || !Positive(options.maximum_segment_duration) ||
        !options.maximum_segments || options.maximum_segments > 65536 || !Positive(options.deadline)) {
        result.reason = "invalid differential-drive calibration, limits, clock or budget";
        return result;
    }
    try {
        const auto& data = trajectory.data();
        const auto& timing = data.timing;
        result.timing = timing;
        result.calibration_revision = limits.calibration_revision;
        const double end = timing.start_stamp + timing.duration;
        double last_clock = clock();
        if (!Nonnegative(last_clock) || last_clock >= options.deadline || last_clock > timing.start_stamp ||
            !std::isfinite(end) || end > timing.valid_until)
            throw std::runtime_error("differential-drive reference is late or its horizon/clock expired");
        const double half_track = limits.track_width / 2.0;
        const auto bounds = Bounds(trajectory, half_track);
        // Every positive knot span starts a separate proof. Never apply a
        // within-span snap bound through a cubic jerk discontinuity.
        std::vector<Interval> pending;
        for (std::size_t i = data.control_points.size(); i > 3; --i) {
            const double begin = timing.start_stamp + data.knots[i - 1] * timing.duration;
            const double finish = timing.start_stamp + data.knots[i] * timing.duration;
            if (!std::isfinite(begin) || !std::isfinite(finish) || finish <= begin)
                throw std::runtime_error("execution clock cannot represent differential-drive knot span");
            pending.push_back({begin, finish, data.knots[i - 1], data.knots[i]});
        }
        while (!pending.empty()) {
            const double now = clock();
            if (!Nonnegative(now) || now < last_clock || now >= options.deadline || now > timing.start_stamp)
                throw std::runtime_error("differential-drive clock regressed, deadline expired or start was missed");
            last_clock = now;
            if (result.examined_intervals == options.maximum_segments)
                throw std::runtime_error("continuous differential-drive proof exhausted interval budget");
            ++result.examined_intervals;
            const auto interval = pending.back();
            pending.pop_back();
            const double h = interval.end - interval.begin, midpoint = interval.begin + h / 2.0;
            if (midpoint <= interval.begin || midpoint >= interval.end)
                throw std::runtime_error("clock precision cannot represent differential-drive proof midpoint");
            const double normalized = (midpoint - timing.start_stamp) / timing.duration;
            if (normalized <= interval.knot_begin || normalized >= interval.knot_end)
                throw std::runtime_error("clock precision moved a proof midpoint outside its original knot span");
            const auto state = trajectory.Evaluate(midpoint);
            const auto body = Project(state);
            const auto actual = Enclose(state, body, bounds, half_track, 0.0);
            const auto actual_violation = Violation(actual, limits);
            if (!actual_violation.empty()) throw std::runtime_error(actual_violation);
            const auto envelope = Enclose(state, body, bounds, half_track, h / 2.0);
            const auto unresolved = Violation(envelope, limits);
            if (h > options.maximum_segment_duration || !unresolved.empty()) {
                pending.push_back({midpoint, interval.end, interval.knot_begin, interval.knot_end});
                pending.push_back({interval.begin, midpoint, interval.knot_begin, interval.knot_end});
                continue;
            }
            Accumulate(envelope, &result);
            ++result.certified_intervals;
        }
        const double finished = clock();
        if (!Nonnegative(finished) || finished < last_clock || finished >= options.deadline ||
            finished > timing.start_stamp)
            throw std::runtime_error("differential-drive proof finished too late or with a regressed clock");
        result.admitted = true;
        result.valid_until = timing.valid_until;
    } catch (const std::exception& error) {
        result.reason = error.what();
    } catch (...) {
        result.reason = "nonstandard differential-drive validation dependency exception";
    }
    return result;
}

}  // namespace autonomy::control::controller::policy_controller::sand_planner
