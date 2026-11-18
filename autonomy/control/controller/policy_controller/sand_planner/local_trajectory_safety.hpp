/* Copyright 2026 The Openbot Authors. */
/** @file local_trajectory_safety.hpp
 * @brief Concrete immutable ESDF/body consumer for final timed spline admission.
 */
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "autonomy/control/controller/policy_controller/sand_planner/execution_esdf.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/differential_drive.hpp"

namespace autonomy::control::controller::policy_controller::sand_planner {

/** @brief Calibrated cylinder enclosing the full robot for every allowed orientation.
 * Radius must include protrusions; Z bounds include allowed roll/pitch effects.
 * This conservative envelope also covers rotation without assuming a circular
 * actual footprint. A height/center-point check alone is insufficient.
 */
struct RobotBodyEnvelope {
    double horizontal_radius = 0.3;  ///< Circumscribed footprint radius, meters.
    double minimum_z = 0.1;          ///< Lowest body point relative to reference XYZ, meters.
    double maximum_z = 0.8;          ///< Highest body point relative to reference XYZ, meters.
};

/** @brief Explicit margins and bounded work shared by the entire trajectory check. */
struct LocalTrajectorySafetyOptions {
    RobotBodyEnvelope body;          ///< Calibrated full-body envelope.
    std::string clock_domain;        ///< Must match the ESDF's shared time domain.
    double position_margin = 0.03;   ///< Current robot localization/tracking uncertainty, meters.
    double reaction_time = 0.1;      ///< Guaranteed worst observation/control/brake reaction, seconds.
    double minimum_braking_deceleration = 0.5;  ///< Guaranteed physical braking capability, m/s^2.
    double maximum_sample_spacing = 0.05;       ///< Largest centerline sweep interval, meters.
    std::size_t maximum_work = 2000000;         ///< Aggregate samples/voxel/support checks, at most 100 million.
};

/** @brief Required entire chord sweep footprint/height including all safety margins. */
struct GroundSupportQuery {
    SweptVolumeQuery sweep;           ///< Original continuous trajectory chord and its identity.
    RobotBodyEnvelope expanded_body;  ///< Full swept body including curvature, uncertainty and stopping.
    double stopping_distance = 0.0;   ///< Already included in expanded horizontal radius, meters.
    std::size_t maximum_work = 0;     ///< Remaining whole-trajectory work; zero denies production checks.
    double reference_position_error = 0.0;  ///< Current reference/tracking position uncertainty, meters.
};

/** @brief Independent measured-ground attestation; no default floor is manufactured. */
struct GroundSupportVerdict {
    GeometryStatus status = GeometryStatus::kIndeterminate;  ///< Missing/unknown support denies admission.
    std::string frame_id;             ///< Exact continuous geometry frame checked.
    std::uint64_t alignment_epoch = 0; ///< Exact execution alignment checked.
    std::uint64_t geometry_revision = 0; ///< Evidence must belong to the pinned geometric snapshot.
    double valid_until = 0.0;         ///< Support must cover the whole stopping sweep and interval.
    std::size_t work = 0;             ///< Actual ground work, charged to the shared trajectory budget.
};

/** @brief Verify measured support continuity, step/drop limits and safe stopping floor.
 * This producer must use actual localized surface measurements, not a constant
 * nominal height. Its bounded execution is an independent integration obligation.
 */
using GroundSupportVerifier = std::function<GroundSupportVerdict(const GroundSupportQuery&)>;

/** @brief Actual reference/geometric result and aggregate concrete-backend work. */
struct LocalTrajectorySafetyResult {
    TrajectoryValidationResult validation;  ///< Admission is geometric/reference only, not a motion lease.
    std::size_t examined_voxels = 0;         ///< All checked cells count, including unknown/occupied.
    std::size_t sweep_samples = 0;          ///< Covering cylinders; spacing adds a continuous sweep margin.
    std::size_t support_queries = 0;        ///< Whole-sweep requests to independent ground evidence.
    std::size_t ground_work = 0;            ///< Measured-ground work across every trajectory chord.
    DifferentialDriveValidationResult differential_drive; ///< Populated only by explicit drive-model admission.
};

/** @brief Validate a final timed spline against actual immutable observation geometry.
 * Uses derivative hull limits, adaptive curve chords, conservative covering
 * cylinders, measured voxel evidence, ground support and braking-volume margins.
 * The entire trajectory shares one work/deadline budget. Missing ground support
 * fails closed. Nonholonomic feasibility, dynamic-object prediction, physical
 * tracker/watchdog and actual source/controller wiring still require validation;
 * a successful result here does not certify those separate obligations.
 */
LocalTrajectorySafetyResult ValidateLocalTrajectory(
    const TimedBSpline& trajectory, const TrajectoryLimits& limits,
    const TrajectoryValidationOptions& validation_options,
    const LocalTrajectorySafetyOptions& safety_options,
    const std::shared_ptr<const LocalEsdfSnapshot>& snapshot,
    const GroundSupportVerifier& ground_support, const ExecutionClock& clock);

class LocalGroundSnapshot;

/** @brief Production admission using measured ground pinned to the exact ESDF instance.
 * Rebuilding an equal-numbered ESDF does not authorize reusing another ground
 * snapshot. Ground cell, continuity and body work share one trajectory budget.
 */
LocalTrajectorySafetyResult ValidateLocalTrajectory(
    const TimedBSpline& trajectory, const TrajectoryLimits& limits,
    const TrajectoryValidationOptions& validation_options,
    const LocalTrajectorySafetyOptions& safety_options,
    const std::shared_ptr<const LocalEsdfSnapshot>& snapshot,
    const std::shared_ptr<const LocalGroundSnapshot>& ground, const ExecutionClock& clock);

/** @brief Production differential-drive admission followed by measured body/ground safety.
 * Kinematic intervals and geometric intervals share maximum_segments; kinematic,
 * body and ground work share maximum_work and one continuously monitored deadline.
 * Both stages must admit the same immutable reference. The geometric-only
 * overload remains useful for its separate obligation, but does not certify a
 * differential drive. Wheel contact, feedback tracking and physical stopping
 * remain independent requirements even when this overload succeeds.
 */
LocalTrajectorySafetyResult ValidateLocalTrajectory(
    const TimedBSpline& trajectory, const TrajectoryLimits& limits,
    const TrajectoryValidationOptions& validation_options,
    const LocalTrajectorySafetyOptions& safety_options,
    const std::shared_ptr<const LocalEsdfSnapshot>& snapshot,
    const std::shared_ptr<const LocalGroundSnapshot>& ground,
    const DifferentialDriveLimits& drive, const ExecutionClock& clock);

}  // namespace autonomy::control::controller::policy_controller::sand_planner
