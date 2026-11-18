/* Copyright 2026 The Openbot Authors. */
/** @file differential_drive_test.cpp
 * @brief Physical model regressions, including unsafe intervals between sampled states.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/differential_drive.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace s = autonomy::control::controller::policy_controller::sand_planner;
namespace {
/** @brief Report an unmet physical expectation without disabling release assertions. */
void Require(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}
/** @brief Test actuator/track calibration is explicit and uses rim-linear units. */
s::DifferentialDriveLimits Limits() {
    s::DifferentialDriveLimits l;
    l.calibration_revision = 7; l.track_width = .4;
    l.wheel_velocity = {1, 1}; l.wheel_acceleration = {3, 3}; l.wheel_jerk = {10, 10};
    l.maximum_lateral_velocity = .005; l.maximum_lateral_acceleration = .025;
    l.maximum_vertical_velocity = 1e-8;
    return l;
}
/** @brief Finite execution-clock interval proof budget. */
s::TrajectoryValidationOptions Options() {
    s::TrajectoryValidationOptions o;
    o.maximum_segment_duration = .1; o.geometry_revision = 1; o.deadline = 10.09;
    return o;
}
/** @brief Cubic power coefficients converted to a four-control Bezier reference. */
s::TimedBSpline Polynomial(const std::array<s::TrajectoryVector, 4>& coefficients,
                          double duration = 1) {
    s::BSplineData data;
    for (unsigned point = 0; point < 4; ++point) {
        s::TrajectoryVector q{};
        for (unsigned axis = 0; axis < 4; ++axis) {
            const double c0 = coefficients[0][axis], c1 = coefficients[1][axis];
            const double c2 = coefficients[2][axis], c3 = coefficients[3][axis];
            q[axis] = c0;
            if (point == 1) q[axis] += c1 / 3;
            if (point == 2) q[axis] += 2 * c1 / 3 + c2 / 3;
            if (point == 3) q[axis] += c1 + c2 + c3;
        }
        data.control_points.push_back(q);
    }
    data.knots = {0, 0, 0, 0, 1, 1, 1, 1};
    data.timing = {"odom", "drive_reference", 9, 1, 10.1, duration, 20};
    return s::TimedBSpline(std::move(data));
}
/** @brief Analytical constant-speed straight reference, optionally with linear yaw. */
s::TimedBSpline Line(double speed = .2, double yaw = 0, double yaw_rate = 0) {
    return Polynomial({s::TrajectoryVector{0, 0, 0, yaw}, {speed, 0, 0, yaw_rate}, {}, {}});
}
/** @brief Exercise the production continuous model checker with a controlled time owner. */
s::DifferentialDriveValidationResult Check(const s::TimedBSpline& reference = Line(),
                                          const s::DifferentialDriveLimits& limits = Limits(),
                                          const s::TrajectoryValidationOptions& options = Options()) {
    return s::ValidateDifferentialDrive(reference, limits, options, [] { return 10.05; });
}

/** @brief Forward straight motion and bounded in-place rotation are feasible. */
void StraightAndRotation() {
    const auto straight = Check();
    Require(straight.admitted && straight.certified_intervals && straight.valid_until == 20,
            "explicit calibrated straight drive must be certified");
    Require(straight.timing.execution_id == "drive_reference" && straight.timing.reference_version == 1 &&
            straight.calibration_revision == 7, "certificate retains its actual reference and calibration identity");
    Require(straight.wheel_velocity_bounds[0] < .201 && straight.lateral_velocity_bound < 1e-8,
            "straight certificate has physical wheel and lateral units");
    Require(Check(Line(0, 1.2, .5)).admitted, "stationary rotation obeys wheel limits");
    auto asymmetric = Limits(); asymmetric.wheel_velocity[0] = .05;
    Require(!Check(Line(0, 1.2, .5), asymmetric).admitted,
            "rotation requires both wheels, including separately calibrated limits");
}
/** @brief A smooth spatial curve cannot request sideways or flying motion. */
void SidewaysAndVertical() {
    const auto sideways = Check(Line(.2, 1.5707963267948966));
    Require(!sideways.admitted && sideways.reason.find("lateral velocity") != std::string::npos,
            "differential drive cannot follow sideways constant velocity");
    const auto vertical = Polynomial({s::TrajectoryVector{}, {.1, 0, .1, 0}, {}, {}});
    Require(!Check(vertical).admitted, "planar wheel model does not authorize a vertical reference");
}
/** @brief Reverse is a declared policy, never inferred from the curve's smoothness. */
void ReversePolicy() {
    Require(!Check(Line(-.2)).admitted, "default forward-only model rejects reversing");
    auto limits = Limits(); limits.maximum_reverse_velocity = .3;
    Require(Check(Line(-.2), limits).admitted, "calibrated reverse allowance enables bounded reverse");
}
/** @brief Collision-free and derivative-limited references can still exceed wheel limits. */
void WheelLimits() {
    auto limits = Limits(); limits.wheel_velocity[1] = .1;
    Require(!Check(Line(), limits).admitted, "right wheel speed limit must be enforced");
    limits = Limits(); limits.wheel_acceleration[0] = .3;
    Require(!Check(Polynomial({s::TrajectoryVector{}, {}, {.2, 0, 0, 0}, {}}), limits).admitted,
            "wheel acceleration is a physical derivative");
    limits = Limits(); limits.wheel_jerk[1] = .5;
    Require(!Check(Polynomial({s::TrajectoryVector{}, {}, {}, {.1, 0, 0, 0}}), limits).admitted,
            "cubic wheel jerk cannot be hidden by low spatial speed");
}
/** @brief Physical duration changes wheel derivatives; normalized shape is insufficient. */
void PhysicalTime() {
    const auto fast = Polynomial({s::TrajectoryVector{}, {.2, 0, 0, 0}, {}, {}}, .1);
    Require(!Check(fast).admitted, "same normalized line in short physical time exceeds wheel speed");
    const auto slow = Polynomial({s::TrajectoryVector{}, {.2, 0, 0, 0}, {}, {}}, 2);
    Require(Check(slow).admitted, "slower same line can satisfy physical limits");
}
/** @brief Midpoint alone appears feasible, while its enclosing interval is unsafe. */
void BetweenSamples() {
    const auto reference = Line(.2, -.2, .4);
    auto limits = Limits(); limits.maximum_lateral_acceleration = .2;
    auto options = Options(); options.maximum_segment_duration = 2;
    const auto middle = reference.Evaluate(10.6);
    Require(std::abs(middle.velocity[0] * std::sin(middle.position[3])) < 1e-10,
            "coarse midpoint has zero lateral velocity");
    Require(!Check(reference, limits, options).admitted,
            "continuous envelope must reject lateral error away from the safe midpoint");
}
/** @brief Lateral velocity tolerance cannot mask a forbidden change in that residual. */
void LateralAcceleration() {
    auto limits = Limits(); limits.maximum_lateral_acceleration = .003;
    const auto reference = Polynomial({s::TrajectoryVector{}, {.2, -.002, 0, 0}, {0, .002, 0, 0}, {}});
    const auto verdict = Check(reference, limits);
    Require(!verdict.admitted && verdict.reason.find("lateral acceleration") != std::string::npos,
            "small lateral velocity with excessive lateral acceleration must be rejected");
}
/** @brief Wheel jerk includes yaw/body coupling even when XYZ and yaw jerks vanish. */
void RotatingBasisDerivatives() {
    auto limits = Limits(); limits.maximum_lateral_velocity = 1;
    limits.maximum_lateral_acceleration = 2; limits.maximum_reverse_velocity = .5;
    limits.wheel_jerk = {.3, .3};
    Require(!Check(Line(.2, 0, 2), limits).admitted,
            "wheel jerk must include rotating-basis coupling, not only XYZ/yaw jerk");
}
/** @brief An approximately tangent cubic turn is feasible with declared model residuals. */
void CurvedReferenceAndCertificate() {
    const auto reference = Polynomial({s::TrajectoryVector{}, {.2, 0, 0, .4},
                                       {0, .04, 0, 0}, {-.2 * .16 / 6, 0, 0, 0}});
    const auto limits = Limits();
    const auto verdict = Check(reference, limits);
    Require(verdict.admitted, "tangent cubic turn within declared residuals must be admitted");
    // Independent dense evaluation checks that published interval bounds enclose
    // actual values, including endpoints; it is not used by production admission.
    for (unsigned sample = 0; sample <= 2000; ++sample) {
        const auto state = reference.Evaluate(10.1 + static_cast<double>(sample) / 2000);
        const double c = std::cos(state.position[3]), sn = std::sin(state.position[3]);
        const double f = c * state.velocity[0] + sn * state.velocity[1];
        const double l = -sn * state.velocity[0] + c * state.velocity[1];
        const double lateral_a = -sn * state.acceleration[0] + c * state.acceleration[1];
        const double forward_a = c * state.acceleration[0] + sn * state.acceleration[1] + state.velocity[3] * l;
        const double forward_j = c * state.jerk[0] + sn * state.jerk[1] + 2 * state.velocity[3] * lateral_a +
                                 state.acceleration[3] * l - state.velocity[3] * state.velocity[3] * f;
        Require(std::abs(l) <= verdict.lateral_velocity_bound + 1e-9, "lateral certificate encloses dense states");
        for (unsigned wheel = 0; wheel < 2; ++wheel) {
            const double offset = (wheel == 0 ? -.2 : .2);
            Require(std::abs(f + offset * state.velocity[3]) <= verdict.wheel_velocity_bounds[wheel] + 1e-9,
                    "speed certificate encloses dense wheel states");
            Require(std::abs(forward_a + offset * state.acceleration[3]) <= verdict.wheel_acceleration_bounds[wheel] + 1e-9,
                    "acceleration certificate encloses dense wheel states");
            Require(std::abs(forward_j + offset * state.jerk[3]) <= verdict.wheel_jerk_bounds[wheel] + 1e-9,
                    "jerk certificate encloses dense wheel states");
        }
    }
}
/** @brief Both one-sided jerk values at actual nonuniform cubic knots must satisfy limits. */
void NonuniformKnotJerk() {
    s::BSplineData data;
    for (double yaw : {0.0, 0.0, 0.0, .001, .001, .001}) data.control_points.push_back({0, 0, 0, yaw});
    data.knots = {0, 0, 0, 0, .49, .51, 1, 1, 1, 1};
    data.timing = {"odom", "drive_reference", 9, 1, 10.1, 1, 20};
    const s::TimedBSpline reference(std::move(data));
    const double endpoint = .2 * std::max(std::abs(reference.Evaluate(10.1).jerk[3]),
                                         std::abs(reference.Evaluate(11.1).jerk[3]));
    const double interior = .2 * std::abs(reference.Evaluate(10.6).jerk[3]);
    Require(interior > endpoint * 2, "narrow middle knot span carries larger jerk than both endpoints");
    auto limits = Limits(); limits.wheel_jerk = {(endpoint + interior) / 2, (endpoint + interior) / 2};
    Require(!Check(reference, limits).admitted, "unsafe interior one-sided jerk must be rejected");
    limits.wheel_jerk = {10, 10};
    Require(Check(reference, limits).admitted, "same actual knot vector passes with sufficient actuator limits");
}
/** @brief Missing calibration and nonfinite model parameters deny, never manufacture defaults. */
void InvalidCalibration() {
    auto limits = Limits(); limits.calibration_revision = 0;
    Require(!Check(Line(), limits).admitted, "calibration owner identity is required");
    limits = Limits(); limits.track_width = 0;
    Require(!Check(Line(), limits).admitted, "zero wheel track is not a differential drive");
    limits = Limits(); limits.wheel_jerk[0] = std::numeric_limits<double>::quiet_NaN();
    Require(!Check(Line(), limits).admitted, "nonfinite actuator limit cannot pass comparisons");
    limits = Limits(); limits.numerical_tolerance = .1;
    Require(!Check(Line(), limits).admitted, "numerical tolerance cannot mask physical slip");
    Require(!Check(Line(0, 1e7)).admitted, "unbounded yaw argument reduction is outside numerical contract");
}
/** @brief Insufficient subdivision work cannot yield a partial certificate. */
void IntervalBudget() {
    auto options = Options(); options.maximum_segments = 1;
    const auto verdict = Check(Line(), Limits(), options);
    Require(!verdict.admitted && verdict.valid_until == 0 && verdict.examined_intervals == 1,
            "every attempted interval is charged before adaptive subdivision");
}
/** @brief Late input, time regression, expiration and clock exceptions fail closed. */
void ClockAndValidity() {
    const auto reference = Line(); const auto limits = Limits(); const auto options = Options();
    Require(!s::ValidateDifferentialDrive(reference, limits, options, [] { return 10.2; }).admitted,
            "late reference cannot be admitted");
    unsigned calls = 0;
    Require(!s::ValidateDifferentialDrive(reference, limits, options, [&] { return calls++ ? 10.04 : 10.05; }).admitted,
            "clock regression rejects without an executable certificate");
    Require(!s::ValidateDifferentialDrive(reference, limits, options, []() -> double { throw 7; }).admitted,
            "nonstandard clock exception rejects");
    auto data = reference.data(); data.timing.valid_until = 10.8;
    Require(!Check(s::TimedBSpline(std::move(data))).admitted, "truncated horizon is not a full reference certificate");
    data.control_points.assign(5, s::TrajectoryVector{});
    data.knots = {0, 0, 0, 0, 1e-8, 1, 1, 1, 1};
    data.timing = {"odom", "drive_reference", 9, 1, 10.1, 1e-8, 20};
    Require(!Check(s::TimedBSpline(std::move(data))).admitted,
            "normalized knot span collapsing in physical-clock precision cannot be skipped");
}
}  // namespace

/** @brief Run physical expectations rather than reimplementing the admission algorithm. */
int main() {
    const std::vector<std::pair<const char*, std::function<void()>>> cases = {
        {"straight_rotation", StraightAndRotation}, {"sideways_vertical", SidewaysAndVertical},
        {"reverse_policy", ReversePolicy}, {"wheel_limits", WheelLimits}, {"physical_time", PhysicalTime},
        {"between_samples", BetweenSamples}, {"lateral_acceleration", LateralAcceleration},
        {"rotating_basis_derivatives", RotatingBasisDerivatives}, {"curved_certificate", CurvedReferenceAndCertificate},
        {"nonuniform_knot_jerk", NonuniformKnotJerk}, {"invalid_calibration", InvalidCalibration},
        {"interval_budget", IntervalBudget}, {"clock_validity", ClockAndValidity}};
    try {
        for (const auto& test : cases) { test.second(); std::cout << "PASS " << test.first << '\n'; }
    } catch (const std::exception& error) { std::cerr << "FAIL " << error.what() << '\n'; return 1; }
    return 0;
}
