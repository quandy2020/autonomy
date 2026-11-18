/* Copyright 2026 The Openbot Authors. */
/** @file reference_handoff.hpp
 * @brief Explicit identity and physical C2 continuity for reference replacement.
 */
#pragma once

#include <optional>
#include <string>

#include "autonomy/control/controller/policy_controller/sand_planner/timed_bspline.hpp"

namespace autonomy::control::controller::policy_controller::sand_planner {

/** @brief Independent numerical tolerances in physical units, not tracking allowances. */
struct HandoffTolerance {
    double position = 1e-8;      ///< Maximum XYZ discrepancy, meters.
    double velocity = 1e-8;      ///< Maximum XYZ discrepancy, meters/second.
    double acceleration = 1e-8;  ///< Maximum XYZ discrepancy, meters/second squared.
    double yaw = 1e-8;           ///< Maximum unwrapped yaw discrepancy, radians.
    double yaw_velocity = 1e-8;  ///< Maximum yaw-rate discrepancy, radians/second.
    double yaw_acceleration = 1e-8;  ///< Maximum yaw-acceleration discrepancy, radians/second squared.
};

/** @brief Prepared reference; final retiming and Safety must still validate it. */
struct HandoffResult {
    std::optional<TimedBSpline> trajectory;  ///< Anchored replacement, absent on failure.
    TrajectoryState switch_state;           ///< Previous reference evaluated at the scheduled start.
    std::string reason;                     ///< Failure explanation; empty on success.
};

/** @brief Anchors a proposal at its start time to the same lease/frame/epoch's old reference.
 * Versions must strictly increase, and the scheduled switch must not precede now.
 * This proves reference continuity only; it does not authorize actual motion.
 */
HandoffResult PrepareReferenceHandoff(const TimedBSpline& previous,
                                     const TimedBSpline& proposal, double now);

/** @brief Rechecks the final replacement after retiming or any geometry modification.
 * @return Empty on a valid C2 reference handoff; an explicit reason otherwise.
 */
std::string CheckReferenceHandoff(const TimedBSpline& previous,
                                 const TimedBSpline& replacement, double now,
                                 const HandoffTolerance& tolerance = {});

}  // namespace autonomy::control::controller::policy_controller::sand_planner
