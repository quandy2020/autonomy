/* Copyright 2026 The Openbot Authors. */
/**
 * @file differential_drive.hpp
 * @brief Continuous-interval differential-drive and wheel derivative admission.
 */
#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "autonomy/control/controller/policy_controller/sand_planner/trajectory_validator.hpp"

namespace autonomy::control::controller::policy_controller::sand_planner {

/** @brief Calibrated planar differential drive; wheel values are rim-linear units.
 * These limits describe a reference model, not wheel contact or actuator health.
 * A nonzero calibration revision and positive track/actuator limits are mandatory.
 * Vertical and lateral tolerances must come from the robot/model specification;
 * they do not grant permission to follow an arbitrary spatial or sideways curve.
 */
struct DifferentialDriveLimits {
    std::uint64_t calibration_revision = 0;  ///< Owner-supplied wheel/track calibration identity.
    double track_width = 0.0;                ///< Distance between drive wheel contact lines, meters.
    std::array<double, 2> wheel_velocity{};  ///< Left/right absolute rim velocity limits, m/s.
    std::array<double, 2> wheel_acceleration{}; ///< Left/right absolute rim acceleration limits, m/s^2.
    std::array<double, 2> wheel_jerk{};       ///< Left/right absolute rim jerk limits, m/s^3.
    double maximum_lateral_velocity = 0.0;  ///< Allowed body-lateral reference residual, m/s.
    double maximum_lateral_acceleration = 0.0; ///< Allowed derivative of that residual, m/s^2.
    double maximum_vertical_velocity = 0.0; ///< Allowed vertical reference residual, m/s.
    double maximum_reverse_velocity = 0.0;  ///< Allowed reverse body-forward speed, m/s; zero forbids reverse.
    double numerical_tolerance = 1e-9;      ///< Declared arithmetic comparison tolerance; at most 1e-6.
    /** @brief Rejects missing calibration, nonfinite values and invalid limits. */
    bool Valid() const;
};

/** @brief Whole-reference certificate, never an authorization or physical tracking result.
 * Bounds are maxima over all certified intervals and both sides of cubic jerk
 * discontinuities. Partial bounds on a failed result are not certificates.
 */
struct DifferentialDriveValidationResult {
    bool admitted = false;                  ///< All intervals certified within the declared model/tolerance.
    TrajectoryTiming timing;                ///< Exact reference/frame/epoch identity checked, not a new lease.
    std::uint64_t calibration_revision = 0; ///< Wheel/track calibration under which these bounds were checked.
    unsigned examined_intervals = 0;        ///< Every initial, split or certified interval consumes work.
    unsigned certified_intervals = 0;       ///< Intervals accepted using derivative envelopes.
    double lateral_velocity_bound = 0.0;    ///< Largest accepted lateral velocity bound, m/s.
    double lateral_acceleration_bound = 0.0; ///< Largest accepted lateral acceleration bound, m/s^2.
    double vertical_velocity_bound = 0.0;   ///< Largest accepted vertical velocity bound, m/s.
    std::array<double, 2> wheel_velocity_bounds{}; ///< Accepted left/right speed bounds, m/s.
    std::array<double, 2> wheel_acceleration_bounds{}; ///< Accepted left/right acceleration bounds, m/s^2.
    std::array<double, 2> wheel_jerk_bounds{}; ///< Accepted one-sided left/right jerk bounds, m/s^3.
    double valid_until = 0.0;               ///< Original reference validity, nonzero only on success.
    std::string reason;                     ///< Explicit model, timing, limit or unresolved-budget failure.
};

/** @brief Certifies the full physical interval, rather than only sampled states.
 * Intervals start at every actual knot. Cubic XYZ/yaw jerk is constant within
 * each knot span; analytic derivative bounds enclose body/wheel quantities
 * between evaluations. Adaptive work and time use the caller's existing limits.
 * This sufficient test can conservatively reject a feasible curve. It does not
 * prove load-bearing support, traction, slope/contact dynamics, tracking,
 * stopping or dynamic-obstacle safety. Those require independent consumers.
 */
DifferentialDriveValidationResult ValidateDifferentialDrive(
    const TimedBSpline& trajectory, const DifferentialDriveLimits& limits,
    const TrajectoryValidationOptions& options, const ExecutionClock& clock);

}  // namespace autonomy::control::controller::policy_controller::sand_planner
