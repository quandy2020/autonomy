/* Copyright 2026 The Openbot Authors. */
/**
 * @file trajectory_tracker.hpp
 * @brief Feedback tracking with immutable admission, leases and explicit stop receipts.
 * @details This component returns expiring command proposals. A base driver must
 * enforce their identity, start/end times and watchdog independently of this
 * process. Command geometry verification and physical stop receipts are required
 * external dependencies; no missing dependency receives a permissive default.
 */
#pragma once

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "autonomy/control/controller/policy_controller/sand_planner/local_ground.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/reference_handoff.hpp"

namespace autonomy::control::controller::policy_controller::sand_planner {

struct AdmittedReferenceResult;

/** @brief Immutable reference certificate created only by combined production admission.
 * Retains the actual geometry and ground instances; equal revision numbers do
 * not permit replacing them. This is not a motion lease or a stop certificate.
 */
class AdmittedReference {
public:
    /** @brief Exact reference checked by the production body, ground and drive validators. */
    const TimedBSpline& trajectory() const { return *trajectory_; }
    /** @brief Exact retained geometry, for independent command verification. */
    const std::shared_ptr<const LocalEsdfSnapshot>& geometry() const { return geometry_; }
    /** @brief Exact retained measured-ground evidence. */
    const std::shared_ptr<const LocalGroundSnapshot>& ground() const { return ground_; }
    /** @brief Calibrated drive limits used for admission and feedback commands. */
    const DifferentialDriveLimits& drive() const { return drive_; }
    /** @brief Full-body/stopping contract used by reference admission. */
    const LocalTrajectorySafetyOptions& safety() const { return safety_; }
    /** @brief Earliest original evidence expiry, never extended by tracking. */
    double valid_until() const { return valid_until_; }

private:
    /** @brief Construction is private so callers cannot manufacture an admitted flag. */
    AdmittedReference() = default;
    friend AdmittedReferenceResult AdmitTrackingReference(
        std::shared_ptr<const TimedBSpline>, const TrajectoryLimits&,
        const TrajectoryValidationOptions&, const LocalTrajectorySafetyOptions&,
        std::shared_ptr<const LocalEsdfSnapshot>, std::shared_ptr<const LocalGroundSnapshot>,
        const DifferentialDriveLimits&, const ExecutionClock&);
    std::shared_ptr<const TimedBSpline> trajectory_; ///< Exact owned immutable reference.
    std::shared_ptr<const LocalEsdfSnapshot> geometry_; ///< Exact admitted occupancy evidence.
    std::shared_ptr<const LocalGroundSnapshot> ground_; ///< Exact admitted support evidence.
    DifferentialDriveLimits drive_; ///< Original wheel/track calibration and limits.
    LocalTrajectorySafetyOptions safety_; ///< Original full-body and braking contract.
    double valid_until_ = 0.0; ///< Original admission expiry.
};

/** @brief Failed admission cannot carry a reference usable by the tracker. */
struct AdmittedReferenceResult {
    std::shared_ptr<const AdmittedReference> reference; ///< Nonnull only after all production checks.
    LocalTrajectorySafetyResult evidence; ///< Actual measured-body/ground/drive work and verdict.
};

/** @brief Build the unforgeable certificate using the concrete combined admission path. */
AdmittedReferenceResult AdmitTrackingReference(
    std::shared_ptr<const TimedBSpline> trajectory, const TrajectoryLimits& limits,
    const TrajectoryValidationOptions& validation, const LocalTrajectorySafetyOptions& safety,
    std::shared_ptr<const LocalEsdfSnapshot> geometry, std::shared_ptr<const LocalGroundSnapshot> ground,
    const DifferentialDriveLimits& drive, const ExecutionClock& clock);

/** @brief Authoritative base-owner lease; defaults authorize no movement. */
struct TrackingLease {
    std::string execution_id; ///< Motion owner scope, independent of reference version.
    std::string frame_id; ///< Continuous execution frame.
    std::string clock_domain; ///< Shared acquisition/execution/actuator time domain.
    std::uint64_t alignment_epoch = 0; ///< Exact continuous-frame generation.
    std::uint64_t calibration_revision = 0; ///< Exact wheel/track calibration.
    std::uint64_t generation = 0; ///< Strictly increasing base-owner grant generation.
    double valid_from = 0.0; ///< First authorized command time.
    double valid_until = 0.0; ///< Absolute expiry, not refreshed by a tick.
    double actuator_watchdog = 0.0; ///< Attested maximum time before the base stops stale commands.
    bool active = false; ///< Explicit current owner grant, not inferred from a trajectory.
};

/** @brief Actual synchronized localization and wheel feedback, not previous commands. */
struct TrackingFeedback {
    TrajectoryState state; ///< Measured state in the continuous execution frame.
    std::array<double, 2> wheel_velocity{}; ///< Actual left/right rim speeds, m/s.
    std::array<double, 2> wheel_acceleration{}; ///< Actual left/right rim accelerations, m/s^2.
    std::string frame_id; ///< Frame of measured XYZ/yaw.
    std::string clock_domain; ///< Explicit feedback time domain.
    std::uint64_t alignment_epoch = 0; ///< Localization alignment generation.
    std::uint64_t calibration_revision = 0; ///< Wheel feedback calibration generation.
    std::uint64_t sequence = 0; ///< Monotonic fresh feedback sequence within the lease.
    double stamp = 0.0; ///< Actual synchronized measurement time.
    bool localization_healthy = false; ///< Independent localization-owner health.
    bool actuator_healthy = false; ///< Independent base feedback/watchdog health.
};

/** @brief Receipt issued by the configured physical-stop authority, never by the tracker. */
struct TrackingStopReceipt {
    TrackingLease lease; ///< Owner/generation for which the base has stopped.
    std::string authority; ///< Exact configured base authority identity.
    std::uint64_t request_sequence = 0; ///< Matching outstanding stop request.
    double stamp = 0.0; ///< Actual authority receipt time.
    double stopped_since = 0.0; ///< Authority-attested continuous stationary interval start.
    bool physically_stopped = false; ///< Explicit base proof, not a zero-command inference.
    bool watchdog_armed = false; ///< Authority confirms stale commands remain inhibited.
};

/** @brief Tracking and finite work/time bounds; gains are explicit feedback corrections. */
struct TrajectoryTrackerOptions {
    std::string stop_authority; ///< Required authority used to attest physical stops.
    double maximum_feedback_age = 0.05; ///< Largest measurement age, seconds.
    double maximum_tick_gap = 0.1; ///< Missed controller-heartbeat threshold, seconds.
    double command_duration = 0.04; ///< Maximum actuator command interval, seconds.
    double validation_budget = 0.005; ///< Exclusive command-verifier budget, seconds.
    double maximum_position_error = 0.02; ///< Maximum XYZ reference tracking discrepancy, meters.
    double maximum_yaw_error = 0.05; ///< Maximum wrapped yaw tracking discrepancy, radians.
    double maximum_velocity_error = 0.1; ///< Maximum measured/reference XYZ speed discrepancy, m/s.
    double stopped_velocity = 0.002; ///< Maximum measured wheel and XYZ speed for a stop receipt.
    double stopped_yaw_velocity = 0.002; ///< Maximum measured yaw speed for a stop receipt.
    double minimum_stopped_duration = 0.1; ///< Required authority-attested stationary dwell, seconds.
    double longitudinal_gain = 1.0; ///< Forward-position feedback gain, 1/s.
    double lateral_gain = 2.0; ///< Lateral error feedback gain, 1/m^2.
    double yaw_gain = 2.0; ///< Heading error feedback gain, 1/s.
    /** @brief Validate all bounds and required authority before creating a tracker. */
    bool Valid() const;
};

/** @brief Expiring bounded-ramp target; a plain Twist cannot carry this contract.
 * The calibrated base must reach the target with its bounded transition profile
 * by valid_until, then stop unless superseded. Finite average transition slopes
 * checked here are necessary bounds, not a proof of continuous actuator jerk.
 * The command verifier must check the actual calibrated ramp and stopping sweep.
 */
struct TrackingCommand {
    TrackingLease lease; ///< Exact actuator owner/generation.
    TrajectoryTiming timing; ///< Exact reference/frame/epoch identity.
    std::uint64_t sequence = 0; ///< Monotonic tracker command sequence.
    std::uint64_t feedback_sequence = 0; ///< Actual feedback used by this command.
    std::uint64_t geometry_revision = 0; ///< Exact reference geometry revision.
    double begin_stamp = 0.0; ///< Earliest authorized application time.
    double valid_until = 0.0; ///< Absolute actuator expiry, never converted into an unbounded Twist.
    double linear_velocity = 0.0; ///< Body-forward ramp endpoint target, m/s.
    double yaw_velocity = 0.0; ///< Body yaw-rate ramp endpoint target, rad/s.
};

/** @brief Independent command-body/ground/stopping sweep, including feedback correction.
 * The verifier must use actual latest geometry and prove the whole command plus
 * reaction/braking interval, honoring deadline and the pinned source policy.
 * Reference collision admission alone does not satisfy this check. Implementations
 * must be bounded; this caller detects overruns but cannot preempt a blocking callback.
 */
struct TrackingCommandQuery {
    TrackingCommand command; ///< Exact candidate whose feedback correction needs verification.
    TrackingFeedback feedback; ///< Actual start state and measured wheel dynamics.
    std::shared_ptr<const AdmittedReference> reference; ///< Immutable original body/ground contract.
    double deadline = 0.0; ///< Exclusive command-validation deadline.
};

/** @brief Command certificate must bind the exact proposal and cover its whole interval. */
struct TrackingCommandVerdict {
    GeometryStatus status = GeometryStatus::kIndeterminate; ///< Missing evidence denies a command.
    TrackingCommand command; ///< Exact checked proposal; any identity/value alteration denies it.
    double valid_until = 0.0; ///< Actual command/stop evidence horizon.
};

/** @brief Required independent geometry verifier; no default success callback exists. */
using TrackingCommandVerifier = std::function<TrackingCommandVerdict(const TrackingCommandQuery&)>;

/** @brief One owner-produced update; absent references never fabricate a safe local plan. */
struct TrajectoryTrackingInput {
    TrackingLease lease; ///< Current grant, rechecked on every tick.
    TrackingFeedback feedback; ///< Actual current measurement.
    std::shared_ptr<const AdmittedReference> reference; ///< New or retained admitted reference.
    std::optional<TrackingStopReceipt> stop_receipt; ///< Optional physical-stop authority evidence.
};

/** @brief A stop request is distinct from an acknowledged physical stop. */
enum class TrajectoryTrackingStatus {
    kWaiting, ///< Physical stop was confirmed; a future admitted reference is not yet due.
    kTracking, ///< An identity-bound finite command passed independent command validation.
    kStopRequested, ///< Base must stop; no matching physical confirmation is present.
    kStopped ///< Matching authority receipt and current quiet feedback confirmed the stop.
};

/** @brief No denied result contains an executable nonzero command. */
struct TrajectoryTrackingResult {
    TrajectoryTrackingStatus status = TrajectoryTrackingStatus::kStopRequested; ///< Actual tracker state.
    std::optional<TrackingCommand> command; ///< Nonnull only after full command validation.
    TrackingLease stop_lease; ///< Identity the actuator must stop, even after an owner switch.
    std::uint64_t stop_request_sequence = 0; ///< Outstanding authority request, stable until receipt.
    double stop_requested_at = 0.0; ///< Original stop-request time, never renewed by retries.
    std::string reason; ///< Explicit waiting or rejection explanation.
};

/** @brief Single-owner tick state; caller must serialize Tick and RequestStop.
 * Startup, revocation, missed heartbeat, stale feedback and every validation
 * failure latch a stop request. Only matching independent authority evidence can
 * release it. Reference replacements use physical C2 continuity and retain their
 * original versions and expiry. The external base watchdog is still required
 * when this process dies; this class is not an actuator or a SLAM provider.
 */
class TrajectoryTracker {
public:
    /** @brief Reject malformed options; initial state requires a physical-stop receipt. */
    explicit TrajectoryTracker(TrajectoryTrackerOptions options);
    /** @brief Duplicating a live tracker would duplicate command and stop identities. */
    TrajectoryTracker(const TrajectoryTracker&) = delete;
    /** @brief Preserve the single serialized command owner. */
    TrajectoryTracker& operator=(const TrajectoryTracker&) = delete;
    /** @brief Compute one feedback command or retain a latched stop; never throws dependency errors. */
    TrajectoryTrackingResult Tick(const TrajectoryTrackingInput& input,
                                  const ExecutionClock& clock, const TrackingCommandVerifier& verifier);
    /** @brief Latch a stop without claiming completion; cannot clear replay history. */
    TrajectoryTrackingResult RequestStop(const TrackingLease& lease, double now, std::string reason);

private:
    /** @brief Current stop result without issuing another request on every retry. */
    TrajectoryTrackingResult StopResult(std::string reason) const;
    TrajectoryTrackerOptions options_; ///< Validated tracker bounds and authority.
    std::shared_ptr<const AdmittedReference> active_; ///< Currently executing immutable reference.
    std::shared_ptr<const AdmittedReference> pending_; ///< C2 replacement at a future switch time.
    TrackingLease lease_; ///< Retained original lease, including its original expiry.
    TrackingLease stop_lease_; ///< Retained identity for the outstanding stop request.
    std::uint64_t stop_sequence_ = 0; ///< Stop-request high-water mark, never reset.
    std::uint64_t command_sequence_ = 0; ///< Command replay high-water mark, never reset.
    std::uint64_t highest_lease_generation_ = 0; ///< Accepted owner grant replay high-water mark.
    std::uint64_t feedback_sequence_ = 0; ///< Last used measured feedback sequence for this grant.
    double stop_stamp_ = 0.0; ///< Original request timestamp.
    double last_clock_ = 0.0; ///< Last actual clock read, including callback completion.
    double last_tick_ = 0.0; ///< Last tick start, for controller heartbeat detection.
    bool has_clock_ = false; ///< First actual read is distinct from time zero.
    bool stop_latched_ = true; ///< Startup cannot presume the robot is already stationary.
    bool stopped_ = false; ///< True only after matching independent physical receipt.
    bool in_tick_ = false; ///< Callback reentry cannot issue a second command or erase a stop.
    bool retired_ = false; ///< Exhausted identity counters permanently deny new motion grants.
};

}  // namespace autonomy::control::controller::policy_controller::sand_planner
