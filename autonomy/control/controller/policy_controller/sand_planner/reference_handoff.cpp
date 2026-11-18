/* Copyright 2026 The Openbot Authors. */
/** @file reference_handoff.cpp
 * @brief Same-owner handoff at an actual switch time and final C2 verification.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/reference_handoff.hpp"

#include <cmath>
#include <exception>
#include <utility>

namespace autonomy::control::controller::policy_controller::sand_planner {
namespace {
/** @brief All identity and time checks precede evaluation or boundary modification. */
std::string CheckIdentity(const TimedBSpline& previous, const TimedBSpline& next, double now) {
    const auto& old = previous.data().timing;
    const auto& current = next.data().timing;
    if (!std::isfinite(now) || now < 0.0 || current.start_stamp < now)
        return "handoff is late or uses an invalid clock value";
    if (old.frame_id != current.frame_id || old.execution_id != current.execution_id ||
        old.alignment_epoch != current.alignment_epoch)
        return "handoff frame, alignment epoch or execution lease differs";
    if (current.reference_version <= old.reference_version)
        return "handoff reference version must strictly increase";
    if (current.start_stamp < old.start_stamp || current.start_stamp > previous.EndStamp())
        return "previous reference does not authorize the scheduled switch time";
    return {};
}
/** @brief Numerical XYZ continuity tested separately from angular continuity. */
bool Close(const TrajectoryVector& a, const TrajectoryVector& b, double linear, double angular) {
    const double norm = std::hypot(std::hypot(a[0] - b[0], a[1] - b[1]), a[2] - b[2]);
    return std::isfinite(norm) && norm <= linear && std::abs(a[3] - b[3]) <= angular;
}
/** @brief Reject nonfinite or negative continuity tolerances. */
bool Valid(double tolerance) { return std::isfinite(tolerance) && tolerance >= 0.0; }
}  // namespace

HandoffResult PrepareReferenceHandoff(const TimedBSpline& previous,
                                     const TimedBSpline& proposal, double now) {
    HandoffResult result;
    result.reason = CheckIdentity(previous, proposal, now);
    if (!result.reason.empty()) return result;
    try {
        result.switch_state = previous.Evaluate(proposal.data().timing.start_stamp);
        result.trajectory = proposal.WithStartState(result.switch_state);
    } catch (const std::exception& exception) {
        result.reason = exception.what();
    }
    return result;
}

std::string CheckReferenceHandoff(const TimedBSpline& previous,
                                 const TimedBSpline& replacement, double now,
                                 const HandoffTolerance& t) {
    auto reason = CheckIdentity(previous, replacement, now);
    if (!reason.empty()) return reason;
    if (!Valid(t.position) || !Valid(t.velocity) || !Valid(t.acceleration) ||
        !Valid(t.yaw) || !Valid(t.yaw_velocity) || !Valid(t.yaw_acceleration))
        return "invalid physical continuity tolerance";
    try {
        const auto stamp = replacement.data().timing.start_stamp;
        const auto a = previous.Evaluate(stamp);
        const auto b = replacement.Evaluate(stamp);
        if (!Close(a.position, b.position, t.position, t.yaw) ||
            !Close(a.velocity, b.velocity, t.velocity, t.yaw_velocity) ||
            !Close(a.acceleration, b.acceleration, t.acceleration, t.yaw_acceleration))
            return "final replacement violates physical C2 reference continuity";
    } catch (const std::exception& exception) {
        return exception.what();
    }
    return {};
}

}  // namespace autonomy::control::controller::policy_controller::sand_planner
