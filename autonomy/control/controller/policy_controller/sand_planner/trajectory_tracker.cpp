/* Copyright 2026 The Openbot Authors. */
/** @file trajectory_tracker.cpp
 * @brief Immutable admission, measured feedback control and latched physical-stop gating.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/trajectory_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace autonomy::control::controller::policy_controller::sand_planner {
namespace {
/** @brief Numerical guards run before any division, scheduling or command arithmetic. */
bool Positive(double value) { return std::isfinite(value) && value > 0.0; }
/** @brief Callback exceptions and early returns release the single-tick reentry guard. */
class TickScope {
public:
    /** @brief Mark the owner as inside a tick until this scope unwinds. */
    explicit TickScope(bool& entered) : entered_(entered) { entered_ = true; }
    /** @brief Unwind the guard on every exit, including dependency exceptions. */
    ~TickScope() { entered_ = false; }
private:
    bool& entered_; ///< Serialized tracker's active-tick flag.
};
/** @brief XYZ magnitude without including the angular coordinate in linear units. */
double Norm(const TrajectoryVector& value) { return std::hypot(std::hypot(value[0], value[1]), value[2]); }
/** @brief Position/velocity error in physical XYZ units. */
double Distance(const TrajectoryVector& a, const TrajectoryVector& b) {
    return std::hypot(std::hypot(a[0] - b[0], a[1] - b[1]), a[2] - b[2]);
}
/** @brief Wrap only the measured heading error; stored spline yaw remains unwrapped. */
double HeadingError(double desired, double measured) { return std::remainder(desired - measured, 2.0 * std::acos(-1.0)); }
/** @brief Stable sinc for the continuous lateral-feedback expression at zero yaw error. */
double Sinc(double value) { return std::abs(value) < 1e-6 ? 1.0 - value * value / 6.0 : std::sin(value) / value; }
/** @brief Exact grant comparison detects both revocation and silent deadline renewal. */
bool SameLease(const TrackingLease& a, const TrackingLease& b) {
    return a.execution_id == b.execution_id && a.frame_id == b.frame_id &&
        a.clock_domain == b.clock_domain && a.alignment_epoch == b.alignment_epoch &&
        a.calibration_revision == b.calibration_revision && a.generation == b.generation &&
        a.valid_from == b.valid_from && a.valid_until == b.valid_until &&
        a.actuator_watchdog == b.actuator_watchdog && a.active == b.active;
}
/** @brief A verifier cannot certify a different reference interval under the same command number. */
bool SameTiming(const TrajectoryTiming& a, const TrajectoryTiming& b) {
    return a.frame_id == b.frame_id && a.execution_id == b.execution_id &&
        a.alignment_epoch == b.alignment_epoch && a.reference_version == b.reference_version &&
        a.start_stamp == b.start_stamp && a.duration == b.duration && a.valid_until == b.valid_until;
}
/** @brief Bind the verdict to every command field, not just an admitted boolean. */
bool SameCommand(const TrackingCommand& a, const TrackingCommand& b) {
    return SameLease(a.lease, b.lease) && SameTiming(a.timing, b.timing) &&
        a.sequence == b.sequence && a.feedback_sequence == b.feedback_sequence &&
        a.geometry_revision == b.geometry_revision && a.begin_stamp == b.begin_stamp &&
        a.valid_until == b.valid_until && a.linear_velocity == b.linear_velocity && a.yaw_velocity == b.yaw_velocity;
}
/** @brief Grants must cover feedback, command validation and an independently enforced watchdog. */
bool ValidLease(const TrackingLease& lease, double now, const TrajectoryTrackerOptions& options) {
    return lease.active && !lease.execution_id.empty() && !lease.frame_id.empty() &&
        !lease.clock_domain.empty() && lease.alignment_epoch && lease.calibration_revision && lease.generation &&
        std::isfinite(lease.valid_from) && lease.valid_from >= 0.0 && Positive(lease.valid_until) &&
        lease.valid_from <= now && lease.valid_until > now + options.validation_budget &&
        Positive(lease.actuator_watchdog) && lease.actuator_watchdog <= options.maximum_tick_gap;
}
/** @brief Actual wheels and localization must share identity and finite acquisition time. */
bool ValidFeedback(const TrackingFeedback& feedback, const TrackingLease& lease,
                   double now, const TrajectoryTrackerOptions& options) {
    return feedback.localization_healthy && feedback.actuator_healthy && feedback.state.Finite() &&
        feedback.frame_id == lease.frame_id && feedback.clock_domain == lease.clock_domain &&
        feedback.alignment_epoch == lease.alignment_epoch &&
        feedback.calibration_revision == lease.calibration_revision && feedback.sequence &&
        std::isfinite(feedback.stamp) && feedback.stamp >= lease.valid_from &&
        feedback.stamp <= now && now - feedback.stamp <= options.maximum_feedback_age &&
        std::all_of(feedback.wheel_velocity.begin(), feedback.wheel_velocity.end(),
                    [](double value) { return std::isfinite(value); }) &&
        std::all_of(feedback.wheel_acceleration.begin(), feedback.wheel_acceleration.end(),
                    [](double value) { return std::isfinite(value); });
}
/** @brief Reference admission and the actuator grant have distinct, matching identities. */
bool Matches(const AdmittedReference& reference, const TrackingLease& lease) {
    const auto& timing = reference.trajectory().data().timing;
    return timing.execution_id == lease.execution_id && timing.frame_id == lease.frame_id &&
        timing.alignment_epoch == lease.alignment_epoch &&
        reference.drive().calibration_revision == lease.calibration_revision &&
        reference.safety().clock_domain == lease.clock_domain &&
        timing.start_stamp >= lease.valid_from && reference.trajectory().EndStamp() <= lease.valid_until;
}
/** @brief A reference switch cannot silently change the robot or braking calibration. */
bool SameExecutionContract(const AdmittedReference& a, const AdmittedReference& b) {
    const auto& x = a.drive(); const auto& y = b.drive();
    const auto& p = a.safety(); const auto& q = b.safety();
    return x.calibration_revision == y.calibration_revision && x.track_width == y.track_width &&
        x.wheel_velocity == y.wheel_velocity && x.wheel_acceleration == y.wheel_acceleration && x.wheel_jerk == y.wheel_jerk &&
        x.maximum_lateral_velocity == y.maximum_lateral_velocity && x.maximum_lateral_acceleration == y.maximum_lateral_acceleration &&
        x.maximum_vertical_velocity == y.maximum_vertical_velocity && x.maximum_reverse_velocity == y.maximum_reverse_velocity &&
        x.numerical_tolerance == y.numerical_tolerance && p.body.horizontal_radius == q.body.horizontal_radius &&
        p.body.minimum_z == q.body.minimum_z && p.body.maximum_z == q.body.maximum_z &&
        p.position_margin == q.position_margin && p.reaction_time == q.reaction_time &&
        p.minimum_braking_deceleration == q.minimum_braking_deceleration && p.clock_domain == q.clock_domain;
}
}  // namespace

AdmittedReferenceResult AdmitTrackingReference(
    std::shared_ptr<const TimedBSpline> trajectory, const TrajectoryLimits& limits,
    const TrajectoryValidationOptions& validation, const LocalTrajectorySafetyOptions& safety,
    std::shared_ptr<const LocalEsdfSnapshot> geometry, std::shared_ptr<const LocalGroundSnapshot> ground,
    const DifferentialDriveLimits& drive, const ExecutionClock& clock) {
    AdmittedReferenceResult result;
    if (!trajectory) {
        result.evidence.validation.reason = "missing immutable tracking reference";
        return result;
    }
    // Take a private immutable copy before validation: a caller may retain a
    // mutable shared_ptr alias to its proposal despite passing a const view.
    trajectory = std::make_shared<const TimedBSpline>(*trajectory);
    result.evidence = ValidateLocalTrajectory(*trajectory, limits, validation, safety, geometry, ground, drive, clock);
    if (!result.evidence.validation.admitted || !result.evidence.differential_drive.admitted) return result;
    auto admitted = std::shared_ptr<AdmittedReference>(new AdmittedReference);
    admitted->trajectory_ = std::move(trajectory);
    admitted->geometry_ = std::move(geometry);
    admitted->ground_ = std::move(ground);
    admitted->drive_ = drive;
    admitted->safety_ = safety;
    admitted->valid_until_ = std::min(result.evidence.validation.valid_until,
                                    result.evidence.differential_drive.valid_until);
    result.reference = std::move(admitted);
    return result;
}

bool TrajectoryTrackerOptions::Valid() const {
    return !stop_authority.empty() && Positive(maximum_feedback_age) && Positive(maximum_tick_gap) &&
        Positive(command_duration) && command_duration <= maximum_tick_gap && Positive(validation_budget) &&
        validation_budget < command_duration && Positive(maximum_position_error) && Positive(maximum_yaw_error) &&
        maximum_yaw_error < std::acos(-1.0) && Positive(maximum_velocity_error) && Positive(stopped_velocity) &&
        Positive(stopped_yaw_velocity) && Positive(minimum_stopped_duration) && Positive(longitudinal_gain) &&
        Positive(lateral_gain) && Positive(yaw_gain) && Positive(controller_budget) &&
        controller_budget + validation_budget < command_duration;
}

TrajectoryTracker::TrajectoryTracker(TrajectoryTrackerOptions options) : options_(std::move(options)) {
    if (!options_.Valid()) throw std::invalid_argument("invalid trajectory tracker bounds or stop authority");
}

TrajectoryTrackingResult TrajectoryTracker::StopResult(std::string reason) const {
    TrajectoryTrackingResult result;
    result.status = stopped_ ? TrajectoryTrackingStatus::kStopped : TrajectoryTrackingStatus::kStopRequested;
    result.stop_lease = stop_lease_;
    result.stop_request_sequence = stop_sequence_;
    result.stop_requested_at = stop_stamp_;
    result.reason = std::move(reason);
    return result;
}

TrajectoryTrackingResult TrajectoryTracker::RequestStop(const TrackingLease& lease, double now, std::string reason) {
    if (!stop_latched_ || !stop_sequence_) {
        if (stop_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
            retired_ = true;
            stop_latched_ = true;
            stopped_ = false;
            active_.reset();
            pending_.reset();
            return StopResult("stop request sequence exhausted; authority must retire this tracker");
        }
        ++stop_sequence_;
        stop_stamp_ = std::isfinite(now) && now >= 0.0 ? now : last_clock_;
        stop_lease_ = lease_.generation ? lease_ : lease;
        stopped_ = false;
    }
    stopped_ = false;
    stop_latched_ = true;
    active_.reset();
    pending_.reset();
    return StopResult(std::move(reason));
}

TrajectoryTrackingResult TrajectoryTracker::Tick(const TrajectoryTrackingInput& input,
                                                const ExecutionClock& clock, const TrackingCommandVerifier& verifier) {
    return TickImpl(input, clock, verifier, nullptr);
}

TrajectoryTrackingResult TrajectoryTracker::TickWithController(const TrajectoryTrackingInput& input,
    const ExecutionClock& clock, const TrackingCommandVerifier& verifier, const TrackingController& controller) {
    return TickImpl(input, clock, verifier, &controller);
}

TrajectoryTrackingResult TrajectoryTracker::TickImpl(const TrajectoryTrackingInput& input,
    const ExecutionClock& clock, const TrackingCommandVerifier& verifier, const TrackingController* controller) {
    if (in_tick_) return RequestStop(input.lease, last_clock_, "trajectory tracker callback reentry is forbidden");
    TickScope tick_scope(in_tick_);
    if (retired_) return StopResult("identity counter exhausted; this tracker remains retired");
    try {
        if (!clock) return RequestStop(input.lease, last_clock_, "missing authoritative execution clock");
        const double now = clock();
        if (!std::isfinite(now) || now < 0.0 || (has_clock_ && now < last_clock_))
            return RequestStop(input.lease, last_clock_, "execution clock is invalid or regressed");
        const bool missed_tick = has_clock_ && now - last_tick_ > options_.maximum_tick_gap;
        last_clock_ = now;
        last_tick_ = now;
        has_clock_ = true;
        if (!stop_sequence_) return RequestStop(input.lease, now, "startup requires physical-stop authority confirmation");
        if (!stop_latched_ && missed_tick) return RequestStop(input.lease, now, "controller heartbeat deadline missed");

        if (stop_latched_) {
            const bool quiet_feedback = ValidFeedback(input.feedback, stop_lease_, now, options_) &&
                std::all_of(input.feedback.wheel_velocity.begin(), input.feedback.wheel_velocity.end(),
                    [&](double speed) { return std::abs(speed) <= options_.stopped_velocity; }) &&
                Norm(input.feedback.state.velocity) <= options_.stopped_velocity &&
                std::abs(input.feedback.state.velocity[3]) <= options_.stopped_yaw_velocity;
            if (!quiet_feedback) stopped_ = false;
            if (!stopped_ && input.stop_receipt) {
                const auto& receipt = *input.stop_receipt;
                const auto& feedback = input.feedback;
                const bool quiet_wheels = std::all_of(feedback.wheel_velocity.begin(), feedback.wheel_velocity.end(),
                    [&](double speed) { return std::abs(speed) <= options_.stopped_velocity; });
                if (SameLease(receipt.lease, stop_lease_) && receipt.authority == options_.stop_authority &&
                    receipt.request_sequence == stop_sequence_ && receipt.physically_stopped && receipt.watchdog_armed &&
                    std::isfinite(receipt.stamp) && std::isfinite(receipt.stopped_since) &&
                    receipt.stopped_since >= stop_stamp_ && receipt.stamp >= receipt.stopped_since &&
                    receipt.stamp - receipt.stopped_since >= options_.minimum_stopped_duration &&
                    receipt.stamp <= now && now - receipt.stamp <= options_.maximum_feedback_age &&
                    ValidFeedback(feedback, stop_lease_, now, options_) && feedback.stamp >= receipt.stamp &&
                    quiet_wheels && Norm(feedback.state.velocity) <= options_.stopped_velocity &&
                    std::abs(feedback.state.velocity[3]) <= options_.stopped_yaw_velocity)
                    stopped_ = true;
            }
            if (!stopped_) return StopResult("waiting for matching physical-stop receipt and stationary feedback");
            if (!ValidLease(input.lease, now, options_) || input.lease.generation <= highest_lease_generation_)
                return StopResult("physical stop confirmed; a fresh active base grant is required");
            if (!ValidFeedback(input.feedback, input.lease, now, options_) || !input.reference ||
                !Matches(*input.reference, input.lease) || input.reference->trajectory().data().timing.start_stamp < now)
                return StopResult("physical stop confirmed; waiting for valid feedback and a future admitted reference");
            lease_ = input.lease;
            highest_lease_generation_ = lease_.generation;
            feedback_sequence_ = 0;
            active_ = input.reference;
            stop_latched_ = false;
            stopped_ = false;
        }

        if (!ValidLease(input.lease, now, options_) || !SameLease(input.lease, lease_))
            return RequestStop(input.lease, now, "motion lease revoked, replaced, expired or silently renewed");
        if (!ValidFeedback(input.feedback, lease_, now, options_) || input.feedback.sequence <= feedback_sequence_)
            return RequestStop(input.lease, now, "feedback is unhealthy, stale, replayed or from another identity");
        if (!input.reference || !Matches(*input.reference, lease_))
            return RequestStop(input.lease, now, "missing or incompatible admitted reference");
        if (input.reference != active_ && input.reference != pending_) {
            const auto previous = pending_ ? pending_ : active_;
            // One future replacement at a time keeps the exact handoff predecessor unambiguous.
            if (pending_ || !SameExecutionContract(*previous, *input.reference) ||
                !CheckReferenceHandoff(previous->trajectory(), input.reference->trajectory(), now).empty())
                return RequestStop(input.lease, now, "reference replacement is replayed, late or violates physical C2 continuity");
            pending_ = input.reference;
        }
        if (pending_ && now >= pending_->trajectory().data().timing.start_stamp) {
            active_ = std::move(pending_);
            pending_.reset();
        }
        const auto& trajectory = active_->trajectory();
        if (now < trajectory.data().timing.start_stamp) {
            if (Norm(input.feedback.state.velocity) > options_.stopped_velocity ||
                std::abs(input.feedback.state.velocity[3]) > options_.stopped_yaw_velocity ||
                std::any_of(input.feedback.wheel_velocity.begin(), input.feedback.wheel_velocity.end(),
                    [&](double speed) { return std::abs(speed) > options_.stopped_velocity; }))
                return RequestStop(input.lease, now, "robot moved before the scheduled initial reference start");
            TrajectoryTrackingResult result;
            result.status = TrajectoryTrackingStatus::kWaiting;
            result.reason = "waiting for scheduled reference start; base remains stopped";
            return result;
        }
        if (now >= trajectory.EndStamp() || now >= active_->valid_until())
            return RequestStop(input.lease, now, "reference or its original evidence horizon expired");
        const auto desired = trajectory.Evaluate(now);
        const auto& feedback = input.feedback;
        const double yaw_error = HeadingError(desired.position[3], feedback.state.position[3]);
        if (Distance(desired.position, feedback.state.position) > options_.maximum_position_error ||
            std::abs(yaw_error) > options_.maximum_yaw_error ||
            Distance(desired.velocity, feedback.state.velocity) > options_.maximum_velocity_error)
            return RequestStop(input.lease, now, "measured pose/velocity left the permitted tracking envelope");

        const double cosine = std::cos(feedback.state.position[3]);
        const double sine = std::sin(feedback.state.position[3]);
        const double dx = desired.position[0] - feedback.state.position[0];
        const double dy = desired.position[1] - feedback.state.position[1];
        const double forward_error = cosine * dx + sine * dy;
        const double lateral_error = -sine * dx + cosine * dy;
        TrackingCommand command;
        command.lease = lease_;
        command.timing = trajectory.data().timing;
        if (command_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
            retired_ = true;
            return RequestStop(input.lease, now, "command sequence exhausted");
        }
        command.sequence = ++command_sequence_;
        command.feedback_sequence = feedback.sequence;
        command.geometry_revision = active_->geometry()->options().revision;
        // Validation occurs before dispatch. The base must schedule begin_stamp;
        // a consumer that immediately publishes a naked Twist violates this contract.
        const double solve_deadline = now + (controller ? options_.controller_budget : 0.0);
        command.begin_stamp = solve_deadline + options_.validation_budget;
        command.valid_until = std::min({now + options_.command_duration, trajectory.EndStamp(),
                                       active_->valid_until(), lease_.valid_until,
                                       now + lease_.actuator_watchdog});
        if (pending_) command.valid_until = std::min(command.valid_until, pending_->trajectory().data().timing.start_stamp);
        if (command.valid_until <= command.begin_stamp)
            return RequestStop(input.lease, now, "insufficient command/evidence time before the next switch or expiry");
        const auto target = trajectory.Evaluate(command.valid_until);
        const double reference_speed = std::cos(target.position[3]) * target.velocity[0] +
                                       std::sin(target.position[3]) * target.velocity[1];
        const auto& drive = active_->drive();
        const double measured_forward = cosine * feedback.state.velocity[0] + sine * feedback.state.velocity[1];
        const double measured_lateral = -sine * feedback.state.velocity[0] + cosine * feedback.state.velocity[1];
        const std::array<double, 2> expected_wheels{
            measured_forward - drive.track_width * feedback.state.velocity[3] / 2.0,
            measured_forward + drive.track_width * feedback.state.velocity[3] / 2.0};
        if (std::abs(measured_lateral) > drive.maximum_lateral_velocity ||
            std::abs(feedback.state.velocity[2]) > drive.maximum_vertical_velocity ||
            std::abs(expected_wheels[0] - feedback.wheel_velocity[0]) > options_.maximum_velocity_error ||
            std::abs(expected_wheels[1] - feedback.wheel_velocity[1]) > options_.maximum_velocity_error)
            return RequestStop(input.lease, now, "measured body/wheel feedback is inconsistent with the calibrated planar drive");
        if (controller) {
            if (!*controller) return RequestStop(input.lease, now, "missing required native tracking controller");
            TrackingCommandQuery solve{command, feedback, active_, solve_deadline};
            TrackingCommand candidate;
            std::string reason;
            const bool solved = (*controller)(solve, clock, &candidate, &reason);
            const double finished = clock();
            if (stop_latched_ || active_ != solve.reference || command_sequence_ != command.sequence)
                return RequestStop(input.lease, last_clock_, "stop or reference ownership changed during native solve");
            if (!std::isfinite(finished) || finished < last_clock_ || finished >= solve.deadline)
                return RequestStop(input.lease, last_clock_, "native controller overran its deadline or execution clock regressed");
            last_clock_ = finished;
            if (!ValidFeedback(feedback, lease_, finished, options_))
                return RequestStop(input.lease, finished, "measured feedback expired during native solve");
            if (!solved)
                return RequestStop(input.lease, finished, reason.empty() ? "native controller supplied no command" : reason);
            command.linear_velocity = candidate.linear_velocity;
            command.yaw_velocity = candidate.yaw_velocity;
            if (!SameCommand(candidate, command))
                return RequestStop(input.lease, finished, "native controller changed command identity, evidence or original timing");
        } else {
            command.linear_velocity = reference_speed * std::cos(yaw_error) + options_.longitudinal_gain * forward_error;
            command.yaw_velocity = target.velocity[3] + options_.yaw_gain * yaw_error +
                                   options_.lateral_gain * reference_speed * Sinc(yaw_error) * lateral_error;
        }
        if (!std::isfinite(command.linear_velocity) || !std::isfinite(command.yaw_velocity) ||
            command.linear_velocity < -drive.maximum_reverse_velocity)
            return RequestStop(input.lease, now, "nonfinite or forbidden reverse feedback command");
        const double horizon = command.valid_until - command.begin_stamp;
        const std::array<double, 2> wheels{command.linear_velocity - drive.track_width * command.yaw_velocity / 2.0,
                                         command.linear_velocity + drive.track_width * command.yaw_velocity / 2.0};
        for (unsigned i = 0; i < 2; ++i) {
            const double start_speed = feedback.wheel_velocity[i] + feedback.wheel_acceleration[i] *
                                       (command.begin_stamp - feedback.stamp);
            const double acceleration = (wheels[i] - start_speed) / horizon;
            const double jerk = (acceleration - feedback.wheel_acceleration[i]) / horizon;
            if (!std::isfinite(acceleration) || !std::isfinite(jerk) || std::abs(wheels[i]) > drive.wheel_velocity[i] ||
                std::abs(acceleration) > drive.wheel_acceleration[i] || std::abs(jerk) > drive.wheel_jerk[i])
                return RequestStop(input.lease, now, "feedback command exceeds calibrated wheel speed/acceleration/jerk limits");
        }
        if (!verifier) return RequestStop(input.lease, now, "missing independent command body/ground/stopping verifier");
        const double verification_deadline = std::min(command.begin_stamp, last_clock_ + options_.validation_budget);
        TrackingCommandQuery query{command, feedback, active_, verification_deadline};
        const auto verdict = verifier(query);
        const double finished = clock();
        if (stop_latched_ || active_ != query.reference || command_sequence_ != command.sequence)
            return RequestStop(input.lease, last_clock_, "stop or reference ownership changed during command validation");
        if (!std::isfinite(finished) || finished < last_clock_ || finished >= query.deadline)
            return RequestStop(input.lease, last_clock_, "command verifier overran its deadline or execution clock regressed");
        last_clock_ = finished;
        if (verdict.status != GeometryStatus::kClear || !SameCommand(verdict.command, command) ||
            !std::isfinite(verdict.valid_until) || verdict.valid_until < command.valid_until)
            return RequestStop(input.lease, finished, "command safety is blocked, unknown, expired or for a different proposal");
        if (!ValidFeedback(feedback, lease_, finished, options_))
            return RequestStop(input.lease, finished, "measured feedback expired during command validation");
        feedback_sequence_ = feedback.sequence;
        TrajectoryTrackingResult result;
        result.status = TrajectoryTrackingStatus::kTracking;
        result.command = std::move(command);
        return result;
    } catch (const std::exception& exception) {
        return RequestStop(input.lease, last_clock_, exception.what());
    } catch (...) {
        return RequestStop(input.lease, last_clock_, "nonstandard trajectory tracking dependency exception");
    }
}

}  // namespace autonomy::control::controller::policy_controller::sand_planner
