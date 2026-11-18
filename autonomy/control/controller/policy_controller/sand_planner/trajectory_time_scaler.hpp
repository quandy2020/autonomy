/* Copyright 2026 The Openbot Authors. */
/** @file trajectory_time_scaler.hpp
 * @brief Bounded retiming that preserves a physical C2 start boundary.
 */
#pragma once

#include <optional>
#include <string>

#include "autonomy/control/controller/policy_controller/sand_planner/timed_bspline.hpp"

namespace autonomy::control::controller::policy_controller::sand_planner {

/** @brief Strictly positive finite limits; XYZ and yaw have separate units. */
struct TrajectoryLimits {
    double linear_velocity = 0.0;      ///< Maximum XYZ speed, meters/second.
    double linear_acceleration = 0.0;  ///< Maximum XYZ acceleration norm, meters/second squared.
    double linear_jerk = 0.0;          ///< Maximum XYZ jerk norm, meters/second cubed.
    double yaw_velocity = 0.0;         ///< Maximum absolute yaw rate, radians/second.
    double yaw_acceleration = 0.0;     ///< Maximum absolute yaw acceleration, radians/second squared.
    double yaw_jerk = 0.0;             ///< Maximum absolute yaw jerk, radians/second cubed.
    /** @brief Checks all six limits before divisions or execution admission. */
    bool Valid() const;
    /** @brief Tests finite derivative hull bounds against all six limits. */
    bool Contains(const TrajectoryDerivativeBounds& bounds) const;
};

/** @brief Retiming work and duration bounds; no implicit validity renewal. */
struct TimeScalingOptions {
    unsigned maximum_iterations = 24;  ///< Maximum anchored candidates examined, including the original.
    double maximum_duration = 0.0;     ///< Caller-supplied physical horizon limit, seconds.
    double growth_margin = 1.01;       ///< Finite multiplier greater than one for iterative retiming.
};

/** @brief Failure carries no executable candidate. */
struct TimeScalingResult {
    std::optional<TimedBSpline> trajectory;  ///< Present only when all final hull bounds satisfy limits.
    unsigned iterations = 0;                ///< Actual anchored candidates examined.
    std::string reason;                    ///< Explicit failure reason, empty on success.
};

/**
 * @brief Retimes and reanchors each trial before testing derivative bounds.
 *
 * The physical start position, velocity and acceleration are invariant.
 * Fixed boundary violations cannot be repaired by slowing down. This bounded
 * search is sufficient, not complete: reanchoring can change geometry, so an
 * admissible trajectory may not be found. Final geometry and handoff admission
 * are independent responsibilities. Timing identity and valid_until are retained.
 */
TimeScalingResult ScaleTrajectory(const TimedBSpline& proposal,
                                 const TrajectoryState& boundary,
                                 const TrajectoryLimits& limits,
                                 const TimeScalingOptions& options);

}  // namespace autonomy::control::controller::policy_controller::sand_planner
