/* Copyright 2026 The Openbot Authors. */
/** @file trajectory_validator.hpp
 * @brief Fail-closed derivative and adaptive swept-volume admission contract.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "autonomy/control/controller/policy_controller/sand_planner/trajectory_time_scaler.hpp"

namespace autonomy::control::controller::policy_controller::sand_planner {

/** @brief Verdict from an independent geometric verifier. */
enum class GeometryStatus { kClear, kBlocked, kIndeterminate };

/**
 * @brief Swept-body interval with conservative deviations from its time chord.
 *
 * The backend must verify the entire robot body swept along the interpolated
 * XYZ/unwrapped-yaw chord, enlarged by the linear and angular deviations.
 * Endpoint-only or centerline-only checks do not satisfy this interface.
 * Unknown geometry cannot receive kClear. Robot footprint, height, ground,
 * observation/pose uncertainty and stopping margins belong to the backend.
 */
struct SweptVolumeQuery {
    TrajectoryState begin;       ///< Exact spline state at begin_stamp.
    TrajectoryState end;         ///< Exact spline state at end_stamp.
    double begin_stamp = 0.0;    ///< Physical execution time, seconds.
    double end_stamp = 0.0;      ///< Physical execution time, seconds.
    double linear_deviation = 0.0;  ///< XYZ chord envelope, meters.
    double yaw_deviation = 0.0;     ///< Unwrapped-yaw chord envelope, radians.
    std::uint64_t geometry_revision = 0;  ///< Required immutable geometry snapshot revision.
    TrajectoryTiming timing;    ///< Execution/frame/epoch/reference identity for evidence matching.
    double deadline = 0.0;      ///< Backend must honor this execution-clock deadline.
};

/** @brief Geometry evidence cannot renew the reference or change snapshots midway. */
struct GeometryVerdict {
    GeometryStatus status = GeometryStatus::kIndeterminate;  ///< Default denies execution.
    std::uint64_t geometry_revision = 0;  ///< Actual snapshot checked by the backend.
    double valid_until = 0.0;            ///< Evidence must cover the entire queried interval.
};

/** @brief Independent verifier; it must be bounded and honor the query deadline. */
using SweptVolumeVerifier = std::function<GeometryVerdict(const SweptVolumeQuery&)>;
/** @brief Authoritative monotonic execution-clock seconds; throws and regressions deny admission. */
using ExecutionClock = std::function<double()>;

/** @brief Required numerical envelopes, immutable snapshot and bounded work. */
struct TrajectoryValidationOptions {
    double maximum_segment_duration = 0.1;  ///< Largest swept chord interval, seconds.
    double maximum_linear_deviation = 0.005;  ///< Maximum XYZ chord envelope, meters.
    double maximum_yaw_deviation = 0.005;     ///< Maximum yaw chord envelope, radians.
    unsigned maximum_segments = 4096;       ///< Total split/verified work bound, at most 65536.
    std::uint64_t geometry_revision = 0;     ///< Nonzero pinned geometry revision.
    double deadline = 0.0;                  ///< Authoritative validation deadline, seconds.
};

/** @brief No partial safe result survives an exception, deadline, invalidity or exhausted budget. */
struct TrajectoryValidationResult {
    bool admitted = false;             ///< Geometric/reference admission only, not physical tracking proof.
    unsigned examined_segments = 0;    ///< Split or checked intervals count toward the same bound.
    unsigned verified_segments = 0;    ///< Whole swept-volume checks that returned compatible kClear.
    double valid_until = 0.0;          ///< Earliest evidence/reference deadline, nonzero only on success.
    std::string reason;               ///< Explicit denial reason; empty on success.
};

/**
 * @brief Checks final derivative limits and adaptive conservative chord sweeps.
 *
 * For a C2 reference with global acceleration bound A, its deviation from
 * a time chord over h seconds is bounded by A*h*h/8; yaw is checked separately.
 * A production ESDF/body backend, nonholonomic admission, motion lease and
 * feedback tracker are still required. No external callback can be forcibly
 * preempted here; callback deadline compliance needs separate integration tests.
 */
TrajectoryValidationResult ValidateTrajectory(const TimedBSpline& trajectory,
                                             const TrajectoryLimits& limits,
                                             const TrajectoryValidationOptions& options,
                                             const SweptVolumeVerifier& verifier,
                                             const ExecutionClock& clock);

}  // namespace autonomy::control::controller::policy_controller::sand_planner
