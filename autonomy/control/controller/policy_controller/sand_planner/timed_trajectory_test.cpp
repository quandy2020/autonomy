/* Copyright 2026 The Openbot Authors. */
/** @file timed_trajectory_test.cpp
 * @brief Independent numerical and failure-admission regressions for timed references.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/reference_handoff.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/trajectory_validator.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
namespace s = autonomy::control::controller::policy_controller::sand_planner;

/** @brief Regression checks remain active in release builds. */
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
/** @brief Checks expected analytical values independently from the production derivative recurrence. */
void Near(double actual, double expected, double tolerance = 1e-8) {
    Require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, "analytical value differs");
}
/** @brief Malformed input and unauthorized evaluations must throw before producing a reference. */
void Reject(const std::function<void()>& operation) {
    bool rejected = false;
    try { operation(); } catch (const std::exception&) { rejected = true; }
    Require(rejected, "malformed or unauthorized operation was accepted");
}
/** @brief Known cubic Bezier with a linear unwrapped-yaw component. */
s::BSplineData Bezier(double duration = 2.0) {
    s::BSplineData data;
    data.control_points = {{{0, 0, 0, 0}}, {{1, 0, 0, 1.0 / 3.0}},
                           {{1, 1, 0, 2.0 / 3.0}}, {{2, 1, 0, 1}}};
    data.knots = {0, 0, 0, 0, 1, 1, 1, 1};
    data.timing = {"odom", "execution-1", 1, 1, 10, duration, 110};
    return data;
}
/** @brief A nonuniform cubic exposes incorrect uniform-span start formulas. */
s::BSplineData Nonuniform() {
    auto data = Bezier(3);
    data.control_points.push_back({3, 1, 0, 1.2});
    data.control_points.push_back({4, 2, 0, 1.4});
    data.knots = {0, 0, 0, 0, .1, .45, 1, 1, 1, 1};
    return data;
}
/** @brief Finite permissive limits isolate geometry/identity cases. */
s::TrajectoryLimits Generous() { return {10000, 10000, 10000, 10000, 10000, 10000}; }
/** @brief Default valid pinned-geometry options with an execution-clock deadline. */
s::TrajectoryValidationOptions ValidationOptions() {
    s::TrajectoryValidationOptions options;
    options.geometry_revision = 7;
    options.deadline = 9.9;
    return options;
}
/** @brief Compatible whole-sweep evidence for numerical-contract tests only. */
s::GeometryVerdict Clear(const s::SweptVolumeQuery& query) {
    return {s::GeometryStatus::kClear, query.geometry_revision, 110};
}
/** @brief An independent XYZ norm for comparing the hull against dense evaluations. */
double Norm(const s::TrajectoryVector& value) {
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

/** @brief Midpoint polynomial values and physical-time units have closed-form expectations. */
void KnownBezier() {
    const s::TimedBSpline curve(Bezier());
    const auto value = curve.Evaluate(11);
    Near(value.position[0], 1); Near(value.position[1], .5);
    Near(value.velocity[0], .75); Near(value.velocity[1], .75);
    Near(value.acceleration[0], 0); Near(value.acceleration[1], 0);
    Near(value.jerk[0], 1.5); Near(value.jerk[1], -1.5);
    Near(value.position[3], .5); Near(value.velocity[3], .5);
    Near(value.acceleration[3], 0); Near(value.jerk[3], 0);
    Near(curve.Evaluate(12).position[0], 2);
}

/** @brief Physical boundary solving must use the actual nonuniform knot vector. */
void NonuniformBoundary() {
    const s::TimedBSpline original(Nonuniform());
    s::TrajectoryState boundary;
    boundary.position = {1, -2, .5, 6.5};
    boundary.velocity = {.2, -.1, .03, .4};
    boundary.acceleration = {.1, .04, -.02, -.2};
    const auto anchored = original.WithStartState(boundary);
    const auto actual = anchored.Evaluate(10);
    for (unsigned axis = 0; axis < 4; ++axis) {
        Near(actual.position[axis], boundary.position[axis]);
        Near(actual.velocity[axis], boundary.velocity[axis]);
        Near(actual.acceleration[axis], boundary.acceleration[axis]);
    }
    for (std::size_t i = 3; i < original.data().control_points.size(); ++i)
        Require(anchored.data().control_points[i] == original.data().control_points[i], "far controls changed");
}

/** @brief Dense evaluations independently check conservative hulls and finite-difference derivatives. */
void HullAndFiniteDifference() {
    const s::TimedBSpline curve(Nonuniform());
    const auto b = curve.DerivativeBounds();
    for (unsigned i = 0; i <= 3000; ++i) {
        const auto x = curve.Evaluate(10 + 3.0 * i / 3000);
        Require(Norm(x.velocity) <= b.linear_velocity + 1e-8, "speed escaped hull");
        Require(Norm(x.acceleration) <= b.linear_acceleration + 1e-8, "acceleration escaped hull");
        Require(Norm(x.jerk) <= b.linear_jerk + 1e-8, "jerk escaped hull");
        Require(std::abs(x.velocity[3]) <= b.yaw_velocity + 1e-8, "yaw rate escaped hull");
    }
    constexpr double time = 10.7, h = 1e-5;
    const auto a = curve.Evaluate(time - h), bstate = curve.Evaluate(time + h), x = curve.Evaluate(time);
    for (unsigned axis = 0; axis < 4; ++axis) {
        Near((bstate.position[axis] - a.position[axis]) / (2 * h), x.velocity[axis], 1e-6);
        Near((bstate.velocity[axis] - a.velocity[axis]) / (2 * h), x.acceleration[axis], 1e-5);
    }
}

/** @brief Simple interior knots are C2 while cubic jerk is allowed to jump. */
void KnotContinuity() {
    const s::TimedBSpline curve(Nonuniform());
    const double knot_stamp = 10.3;
    const auto left = curve.Evaluate(knot_stamp - 1e-8), right = curve.Evaluate(knot_stamp + 1e-8);
    for (unsigned axis = 0; axis < 4; ++axis) {
        Near(left.position[axis], right.position[axis], 1e-5);
        Near(left.velocity[axis], right.velocity[axis], 1e-5);
        Near(left.acceleration[axis], right.acceleration[axis], 1e-4);
    }
    Require(std::abs(left.jerk[0] - right.jerk[0]) > 1, "fixture did not expose cubic jerk jump");
}

/** @brief NaNs, repeated interior knots, oversized input and expired evaluation all fail explicitly. */
void StructuralRejections() {
    auto data = Bezier(); data.control_points[0][1] = std::numeric_limits<double>::quiet_NaN();
    Reject([&] { s::TimedBSpline ignored(data); });
    data = Nonuniform(); data.knots[5] = .1;
    Reject([&] { s::TimedBSpline ignored(data); });
    data = Bezier(); data.timing.execution_id.clear();
    Reject([&] { s::TimedBSpline ignored(data); });
    data = Bezier(); data.timing.duration = 0;
    Reject([&] { s::TimedBSpline ignored(data); });
    data = Bezier(); data.control_points.resize(65);
    Reject([&] { s::TimedBSpline ignored(data); });
    data = Bezier(); data.timing.valid_until = 10.5;
    const s::TimedBSpline expired(data);
    Reject([&] { expired.Evaluate(10.6); });
    Reject([&] { expired.DerivativeControlPoints(4); });
}

/** @brief Accepted slower reference retains all nonzero physical start derivatives and its original lease. */
void RetimingReanchors() {
    const s::TimedBSpline curve(Bezier(.3));
    s::TrajectoryState boundary;
    boundary.velocity = {.2, 0, 0, .1};
    boundary.acceleration = {.05, 0, 0, .03};
    const s::TrajectoryLimits limits{1, 1, 1, 1, 1, 1};
    const auto result = s::ScaleTrajectory(curve, boundary, limits, {24, 20, 1.01});
    Require(result.trajectory.has_value(), result.reason.c_str());
    Require(result.iterations > 1 && result.trajectory->data().timing.duration > .3, "fixture did not retime");
    Require(limits.Contains(result.trajectory->DerivativeBounds()), "final bounds were not rechecked");
    const auto start = result.trajectory->Evaluate(10);
    for (unsigned axis = 0; axis < 4; ++axis) {
        Near(start.velocity[axis], boundary.velocity[axis]);
        Near(start.acceleration[axis], boundary.acceleration[axis]);
    }
    Require(result.trajectory->data().timing.valid_until == 110, "retiming renewed validity");
}

/** @brief Fixed boundary violations and insufficient duration/iterations cannot yield an executable reference. */
void RetimingDenials() {
    const s::TimedBSpline curve(Bezier(.2));
    s::TrajectoryState boundary;
    const s::TrajectoryLimits limits{1, 1, 1, 1, 1, 1};
    boundary.velocity[0] = 2;
    Require(!s::ScaleTrajectory(curve, boundary, limits, {24, 20, 1.01}).trajectory, "fixed speed repaired by retiming");
    boundary = {};
    Require(!s::ScaleTrajectory(curve, boundary, limits, {1, 20, 1.01}).trajectory, "iteration exhaustion admitted");
    Require(!s::ScaleTrajectory(curve, boundary, limits, {24, .3, 1.01}).trajectory, "duration cap ignored");
    auto data = Bezier(.2); data.timing.valid_until = 10.25;
    Require(!s::ScaleTrajectory(s::TimedBSpline(data), boundary, limits, {24, 20, 1.01}).trajectory,
            "authorized horizon extended");
    auto bad = limits; bad.linear_jerk = std::numeric_limits<double>::infinity();
    Require(!s::ScaleTrajectory(curve, boundary, bad, {24, 20, 1.01}).trajectory, "nonfinite limit accepted");
}

/** @brief A handoff inside the old reference is checked again after duration adjustment. */
void FinalHandoff() {
    const s::TimedBSpline previous(Bezier());
    auto proposal = Bezier(1); proposal.timing.start_stamp = 11; proposal.timing.reference_version = 2;
    const auto prepared = s::PrepareReferenceHandoff(previous, s::TimedBSpline(proposal), 10.9);
    Require(prepared.trajectory.has_value(), prepared.reason.c_str());
    const auto scaled = s::ScaleTrajectory(*prepared.trajectory, prepared.switch_state,
                                           {2, 5, 10, 2, 5, 10}, {24, 20, 1.01});
    Require(scaled.trajectory.has_value(), scaled.reason.c_str());
    Require(s::CheckReferenceHandoff(previous, *scaled.trajectory, 10.9).empty(), "final physical handoff differs");
    auto timing = scaled.trajectory->data().timing; timing.duration *= 2;
    Require(!s::CheckReferenceHandoff(previous, scaled.trajectory->WithTiming(timing), 10.9).empty(),
            "unanchored retiming escaped final C2 check");
}

/** @brief Wrong ownership, frame, epoch, ordering or expired old reference denies handoff. */
void HandoffIdentity() {
    const s::TimedBSpline previous(Bezier());
    auto next = Bezier(); next.timing.start_stamp = 11; next.timing.reference_version = 2;
    for (unsigned i = 0; i < 5; ++i) {
        auto changed = next;
        if (i == 0) changed.timing.frame_id = "map";
        if (i == 1) changed.timing.execution_id = "execution-2";
        if (i == 2) changed.timing.alignment_epoch = 2;
        if (i == 3) changed.timing.reference_version = 1;
        if (i == 4) changed.timing.start_stamp = 12.1;
        Require(!s::PrepareReferenceHandoff(previous, s::TimedBSpline(changed), 10.9).trajectory,
                "invalid handoff identity accepted");
    }
    Require(!s::PrepareReferenceHandoff(previous, s::TimedBSpline(next), 11.1).trajectory, "late handoff accepted");
}

/** @brief Admission uses adaptive interval sweeps, each with matching identity and a bounded envelope. */
void AdaptiveSweeps() {
    const s::TimedBSpline curve(Bezier());
    const auto options = ValidationOptions();
    unsigned calls = 0;
    const auto verifier = [&](const s::SweptVolumeQuery& query) {
        ++calls;
        Require(query.linear_deviation <= options.maximum_linear_deviation, "linear envelope too coarse");
        Require(query.yaw_deviation <= options.maximum_yaw_deviation, "yaw envelope too coarse");
        Require(query.end_stamp - query.begin_stamp <= options.maximum_segment_duration, "interval too coarse");
        Require(query.timing.execution_id == "execution-1" && query.geometry_revision == 7, "identity lost");
        return Clear(query);
    };
    const auto result = s::ValidateTrajectory(curve, Generous(), options, verifier, [] { return 9.0; });
    Require(result.admitted && calls > 1 && calls == result.verified_segments, result.reason.c_str());
}

/** @brief An obstacle strictly between safe endpoints is rejected by whole-chord body validation. */
void InteriorObstacle() {
    auto data = Bezier();
    data.control_points = {{{0, 0, 0, 0}}, {{1, 0, 0, 0}}, {{2, 0, 0, 0}}, {{3, 0, 0, 0}}};
    const s::TimedBSpline curve(data);
    auto options = ValidationOptions(); options.maximum_segment_duration = 3;
    const auto result = s::ValidateTrajectory(curve, Generous(), options,
        [](const s::SweptVolumeQuery& query) {
            const bool intersects = query.begin.position[0] <= 1.6 && query.end.position[0] >= 1.4;
            return s::GeometryVerdict{intersects ? s::GeometryStatus::kBlocked : s::GeometryStatus::kClear,
                                      query.geometry_revision, 110};
        }, [] { return 9.0; });
    Require(!result.admitted && result.verified_segments == 0, "interior swept obstacle missed");
}

/** @brief Curved motion needs a chord deviation envelope even when the chord itself is clear. */
void CurvedEnvelope() {
    auto data = Bezier();
    data.control_points = {{{0, 0, 0, 0}}, {{1, 1, 0, 0}}, {{2, 1, 0, 0}}, {{3, 0, 0, 0}}};
    const s::TimedBSpline curve(data);
    Near(curve.Evaluate(11).position[1], .75);
    auto options = ValidationOptions(); options.maximum_segment_duration = 3; options.maximum_linear_deviation = 2;
    bool envelope_seen = false;
    const auto result = s::ValidateTrajectory(curve, Generous(), options,
        [&](const s::SweptVolumeQuery& query) {
            envelope_seen = query.linear_deviation >= .75;
            return s::GeometryVerdict{envelope_seen ? s::GeometryStatus::kBlocked : s::GeometryStatus::kClear,
                                      query.geometry_revision, 110};
        }, [] { return 9.0; });
    Require(envelope_seen && !result.admitted, "curved swept envelope ignored");
}

/** @brief Unknown, mixed revisions, stale evidence and callback exceptions never admit a partial result. */
void EvidenceDenials() {
    const s::TimedBSpline curve(Bezier());
    const auto options = ValidationOptions();
    for (unsigned i = 0; i < 4; ++i) {
        const auto result = s::ValidateTrajectory(curve, Generous(), options,
            [i](const s::SweptVolumeQuery& query) -> s::GeometryVerdict {
                if (i == 3) throw std::runtime_error("backend failure");
                auto verdict = Clear(query);
                if (i == 0) verdict.status = s::GeometryStatus::kIndeterminate;
                if (i == 1) ++verdict.geometry_revision;
                if (i == 2) verdict.valid_until = query.end_stamp - .01;
                return verdict;
            }, [] { return 9.0; });
        Require(!result.admitted && result.valid_until == 0, "invalid backend evidence admitted");
    }
}

/** @brief Budget exhaustion, late callbacks, clock regressions and limit failures are explicit denials. */
void ValidationBudgets() {
    const s::TimedBSpline curve(Bezier());
    auto options = ValidationOptions(); options.maximum_segments = 1;
    Require(!s::ValidateTrajectory(curve, Generous(), options, Clear, [] { return 9.0; }).admitted,
            "exhausted adaptive budget admitted");
    options = ValidationOptions();
    double now = 9;
    Require(!s::ValidateTrajectory(curve, Generous(), options,
        [&](const s::SweptVolumeQuery& query) { now = 10; return Clear(query); }, [&] { return now; }).admitted,
        "late callback admitted");
    unsigned calls = 0;
    Require(!s::ValidateTrajectory(curve, Generous(), options, Clear,
        [&] { return calls++ ? 8.9 : 9.0; }).admitted, "regressed clock admitted");
    Require(!s::ValidateTrajectory(curve, {.1, .1, .1, .1, .1, .1}, options, Clear,
                                  [] { return 9.0; }).admitted, "derivative limit failure admitted");
}
}  // namespace

/** @brief Runs named independent regressions and returns nonzero on any failure. */
int main() {
    const std::vector<std::pair<const char*, std::function<void()>>> cases = {
        {"known physical-time Bezier", KnownBezier}, {"nonuniform start boundary", NonuniformBoundary},
        {"hull and finite-difference derivatives", HullAndFiniteDifference}, {"C2 and cubic jerk jump", KnotContinuity},
        {"structural and expiry rejections", StructuralRejections}, {"retiming preserves physical boundary", RetimingReanchors},
        {"retiming budget and fixed-boundary denials", RetimingDenials}, {"final C2 handoff", FinalHandoff},
        {"handoff ownership and timing", HandoffIdentity}, {"adaptive swept intervals", AdaptiveSweeps},
        {"obstacle between safe endpoints", InteriorObstacle}, {"curved chord envelope", CurvedEnvelope},
        {"independent evidence denials", EvidenceDenials}, {"work clock and limit denials", ValidationBudgets}};
    try {
        for (const auto& test : cases) { test.second(); std::cout << "PASS " << test.first << '\n'; }
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
    return 0;
}
