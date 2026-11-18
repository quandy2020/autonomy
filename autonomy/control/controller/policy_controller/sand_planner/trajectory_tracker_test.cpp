/* Copyright 2026 The Openbot Authors. */
/** @file trajectory_tracker_test.cpp
 * @brief Real measured-floor admission and adversarial tracking/stop-owner scenarios.
 * @details Wire decoding, ESDF, ground fitting and reference admission are real
 * production consumers. Localization, base grant/stop authority and command
 * geometry replies are controlled dependencies; no physical robot is certified.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/trajectory_tracker.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace p = autonomy::perception;
namespace s = autonomy::control::controller::policy_controller::sand_planner;
namespace {
/** @brief Assertions are active in release builds. */
void Require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
/** @brief Independent little-endian FLOAT32 encoding for real floor hit records. */
void Float(std::string& bytes, std::size_t offset, float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned i = 0; i < 4; ++i) bytes[offset + i] = static_cast<char>((bits >> (8 * i)) & 255);
}
/** @brief Explicit calibrated reference bounds for these controlled base fixtures. */
s::DifferentialDriveLimits Drive() {
    s::DifferentialDriveLimits drive;
    drive.calibration_revision = 7; drive.track_width = .4;
    drive.wheel_velocity = {1, 1}; drive.wheel_acceleration = {3, 3}; drive.wheel_jerk = {10, 10};
    drive.maximum_lateral_velocity = .005; drive.maximum_lateral_acceleration = .025;
    drive.maximum_vertical_velocity = 1e-8;
    return drive;
}
/** @brief Build one real geometry/support fixture, retaining actual wire provenance. */
std::shared_ptr<const s::LocalGroundSnapshot> Ground() {
    constexpr unsigned count = 79 * 79;
    std::string bytes(count * 12, '\0');
    unsigned index = 0;
    for (int y = -39; y <= 39; ++y)
        for (int x = -39; x <= 39; ++x, ++index) {
            Float(bytes, index * 12, x * .05F); Float(bytes, index * 12 + 4, y * .05F);
            Float(bytes, index * 12 + 8, -2.0F);
        }
    p::PointCloudBytes cloud{count, 1, 12, count * 12, false, "lidar", 10,
        {{"x", 0, p::FieldScalar::kFloat32, 1}, {"y", 4, p::FieldScalar::kFloat32, 1},
         {"z", 8, p::FieldScalar::kFloat32, 1}}, bytes};
    p::SensorDecodingOptions decoding; decoding.valid_for = 3; decoding.instantaneous_cloud = true;
    p::ObservationPacket packet;
    p::ObservationIngress ingress(p::SensorMode::kLidarOnly, decoding,
        [](p::SensorKind sensor, const std::string& frame, double stamp) {
            p::AcquisitionLocalization pose;
            pose.world_frame = "map"; pose.sensor_frame = frame; pose.map_epoch = 7;
            pose.calibration_revision = 3; pose.support = p::Mask(sensor); pose.healthy = true;
            pose.valid_from = stamp - 1; pose.valid_until = stamp + 3;
            pose.world_from_sensor.translation.z = 2;
            return pose;
        }, [&](auto value, double) { packet = std::move(value); return true; }, [] { return 10.05; });
    Require(ingress.PushCloud(cloud, 1) == p::ObservationIngressResult::kAccepted, "real floor wire records accepted");
    s::ExecutionAlignment alignment;
    alignment.world_frame = "map"; alignment.execution_frame = "odom";
    alignment.clock_domain = "tracker_test_clock"; alignment.map_epoch = 7; alignment.alignment_epoch = 9;
    alignment.healthy = true; alignment.valid_from = 9; alignment.valid_until = 13;
    s::LocalEsdfOptions options;
    options.origin = {-2, -2, -1.125}; options.dimensions = {16, 16, 16}; options.resolution = .25;
    options.mode = p::SensorMode::kLidarOnly; options.clock_domain = "tracker_test_clock"; options.revision = 17;
    options.maximum_observation_age = 3; options.measurement_error = .002; options.surface_margin = .005;
    options.deadline = 10.09;
    auto geometry = s::BuildLocalEsdf({{packet, alignment}}, options, [] { return 10.05; });
    Require(static_cast<bool>(geometry.snapshot), "actual ESDF built");
    s::LocalGroundOptions ground_options;
    ground_options.calibration = {"odom", "tracker_test_clock", 9, 9, 13, 0, .002, 0, true, true};
    ground_options.deadline = 10.09;
    auto ground = s::BuildLocalGround(geometry.snapshot, ground_options, [] { return 10.05; });
    Require(static_cast<bool>(ground.snapshot), "actual ground patches fitted");
    return ground.snapshot;
}
/** @brief Analytical x=.1*u^3 reference starts at rest with zero acceleration. */
s::TimedBSpline Reference() {
    s::BSplineData data;
    data.control_points = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {.1, 0, 0, 0}};
    data.knots = {0, 0, 0, 0, 1, 1, 1, 1};
    data.timing = {"odom", "test_execution", 9, 1, 10.25, 1, 13};
    return s::TimedBSpline(std::move(data));
}
/** @brief Unforgeable certificate goes through the actual combined production validator. */
std::shared_ptr<const s::AdmittedReference> Admit(const std::shared_ptr<const s::LocalGroundSnapshot>& ground,
                                               const s::TimedBSpline& trajectory = Reference()) {
    s::TrajectoryValidationOptions validation; validation.geometry_revision = 17; validation.deadline = 10.09;
    s::LocalTrajectorySafetyOptions safety;
    safety.body = {.12, .2, .8}; safety.position_margin = .005; safety.clock_domain = "tracker_test_clock";
    const auto result = s::AdmitTrackingReference(std::make_shared<const s::TimedBSpline>(trajectory),
        {3, 10, 100, 4, 10, 100}, validation, safety, ground->geometry(), ground, Drive(), [] { return 10.05; });
    if (!result.reference) throw std::runtime_error(result.evidence.validation.reason);
    Require(result.evidence.ground_work && result.evidence.differential_drive.admitted,
            "certificate consumed real ground and differential-drive evidence");
    return result.reference;
}
/** @brief Exact owner grant with independently attested watchdog. */
s::TrackingLease Lease() { return {"test_execution", "odom", "tracker_test_clock", 9, 7, 1, 9, 13, .08, true}; }
/** @brief Actual test measurements with explicit acquisition time and health owners. */
s::TrackingFeedback Feedback(double now, std::uint64_t sequence, const s::TrajectoryState& state = {}) {
    s::TrackingFeedback feedback;
    feedback.state = state; feedback.wheel_velocity = {state.velocity[0], state.velocity[0]};
    feedback.wheel_acceleration = {state.acceleration[0], state.acceleration[0]};
    feedback.frame_id = "odom"; feedback.clock_domain = "tracker_test_clock";
    feedback.alignment_epoch = 9; feedback.calibration_revision = 7;
    feedback.sequence = sequence; feedback.stamp = now;
    feedback.localization_healthy = true; feedback.actuator_healthy = true;
    return feedback;
}
/** @brief Explicit physical-stop authority fixture; no zero command generates this receipt. */
s::TrackingStopReceipt Receipt(const s::TrajectoryTrackingResult& request, double stamp) {
    return {request.stop_lease, "base_test_authority", request.stop_request_sequence,
            stamp, request.stop_requested_at, true, true};
}
/** @brief Controlled command verifier for separate identity/deadline tests, not production geometry. */
s::TrackingCommandVerdict Clear(const s::TrackingCommandQuery& query) {
    Require(query.reference && query.reference->geometry() && query.reference->ground(),
            "independent command verifier receives actual retained snapshots");
    return {s::GeometryStatus::kClear, query.command, query.command.valid_until};
}
/** @brief Fixture serializes a real tracker and supplies independent clock/base-owner replies. */
struct Runner {
    double now = 10.1; ///< Controlled authoritative execution time.
    std::uint64_t feedback_sequence = 1; ///< Fresh measurement sequence for each scenario tick.
    s::TrajectoryTracker tracker; ///< Actual production state machine.
    s::TrajectoryTrackingInput input; ///< Controlled external dependencies and real admission certificate.
    explicit Runner(std::shared_ptr<const s::AdmittedReference> reference)
        : tracker([] { s::TrajectoryTrackerOptions options; options.stop_authority = "base_test_authority"; return options; }()) {
        input.lease = Lease(); input.reference = std::move(reference); input.feedback = Feedback(now, feedback_sequence);
    }
    /** @brief Issue one tick without overriding a deliberately adversarial feedback fixture. */
    s::TrajectoryTrackingResult Tick(const s::TrackingCommandVerifier& verifier = Clear) {
        return tracker.Tick(input, [&] { return now; }, verifier);
    }
    /** @brief Real owner stop gate precedes scheduled initial trajectory execution. */
    void Start() {
        const auto request = Tick();
        Require(!request.command && request.status == s::TrajectoryTrackingStatus::kStopRequested,
                "startup never assumes that the physical base is already stopped");
        now = 10.21; input.feedback = Feedback(now, ++feedback_sequence);
        input.stop_receipt = Receipt(request, now);
        const auto waiting = Tick();
        Require(waiting.status == s::TrajectoryTrackingStatus::kWaiting && !waiting.command,
                "matching physical-stop receipt waits until the scheduled reference start");
        input.stop_receipt.reset();
        Advance(10.25);
        Require(static_cast<bool>(Tick().command), "actual admitted reference begins after independent stop confirmation");
    }
    /** @brief Analytical measurement oracle independently supplies state at each physical time. */
    void Advance(double stamp) {
        now = stamp;
        input.feedback = Feedback(now, ++feedback_sequence, input.reference->trajectory().Evaluate(now));
    }
};

/** @brief Real source geometry admission cannot be bypassed using missing ground. */
void AdmissionFailure(const std::shared_ptr<const s::LocalGroundSnapshot>& ground) {
    s::TrajectoryValidationOptions validation; validation.geometry_revision = 17; validation.deadline = 10.09;
    s::LocalTrajectorySafetyOptions safety; safety.clock_domain = "tracker_test_clock";
    auto result = s::AdmitTrackingReference(std::make_shared<const s::TimedBSpline>(Reference()),
        {3, 10, 100, 4, 10, 100}, validation, safety, ground->geometry(), nullptr, Drive(), [] { return 10.05; });
    Require(!result.reference && !result.evidence.validation.admitted, "no measured ground cannot manufacture a tracker certificate");
    result = s::AdmitTrackingReference(nullptr, {}, validation, safety, {}, {}, Drive(), [] { return 10.05; });
    Require(!result.reference, "missing reference cannot produce a certificate");
    auto mutable_proposal = std::make_shared<s::TimedBSpline>(Reference());
    safety.body = {.12, .2, .8}; safety.position_margin = .005;
    result = s::AdmitTrackingReference(mutable_proposal, {3, 10, 100, 4, 10, 100}, validation,
        safety, ground->geometry(), ground, Drive(), [] { return 10.05; });
    Require(static_cast<bool>(result.reference), "actual mutable proposal was admitted using a private immutable copy");
    auto changed = Reference().data();
    for (auto& control : changed.control_points) control[0] += 1;
    *mutable_proposal = s::TimedBSpline(std::move(changed));
    Require(result.reference->trajectory().Evaluate(10.25).position[0] == 0 &&
            mutable_proposal->Evaluate(10.25).position[0] == 1,
            "caller mutation after admission cannot change the certified trajectory");
}
/** @brief External consumers receive future application time, identity and finite absolute expiry. */
void CommandsAndStop(const std::shared_ptr<const s::AdmittedReference>& reference) {
    Runner runner(reference); runner.Start(); runner.Advance(10.29);
    const auto tracked = runner.Tick();
    Require(tracked.command && tracked.command->begin_stamp > runner.now &&
            tracked.command->valid_until <= runner.now + .04 && tracked.command->feedback_sequence == runner.feedback_sequence &&
            tracked.command->timing.reference_version == 1 && tracked.command->geometry_revision == 17,
            "command carries exact identity, measured-feedback sequence and absolute scheduled expiry");
    const double expected = .3 * std::pow(tracked.command->valid_until - 10.25, 2);
    Require(std::abs(tracked.command->linear_velocity - expected) < 1e-12 && tracked.command->yaw_velocity == 0,
            "straight ramp endpoint matches an independent analytical physical-time derivative");
    const auto stop = runner.tracker.RequestStop(runner.input.lease, runner.now, "operator stop");
    Require(!stop.command && stop.status == s::TrajectoryTrackingStatus::kStopRequested && stop.stop_request_sequence == 2,
            "zero output is a stop request, never a physical-stop confirmation");
    runner.now = 10.4; runner.input.feedback = Feedback(runner.now, ++runner.feedback_sequence);
    runner.input.stop_receipt = Receipt(stop, runner.now);
    const auto stopped = runner.Tick();
    Require(stopped.status == s::TrajectoryTrackingStatus::kStopped && !stopped.command,
            "physical receipt confirms stop but does not regrant the same retired lease");
    runner.input.feedback.state.velocity[0] = .01;
    Require(runner.Tick().status == s::TrajectoryTrackingStatus::kStopRequested,
            "contradictory moving feedback invalidates a remembered physical-stop receipt");
}
/** @brief Faults must latch a stop; repeated ticks cannot resurrect the old reference grant. */
void FeedbackFailures(const std::shared_ptr<const s::AdmittedReference>& reference) {
    using Mutator = std::function<void(Runner&)>;
    const std::array<Mutator, 10> faults{{
        [](Runner& r) { r.input.feedback.stamp -= .06; },
        [](Runner& r) { r.input.feedback.sequence = 1; },
        [](Runner& r) { r.input.feedback.localization_healthy = false; },
        [](Runner& r) { r.input.feedback.actuator_healthy = false; },
        [](Runner& r) { ++r.input.feedback.calibration_revision; },
        [](Runner& r) { r.input.feedback.clock_domain = "wrong_clock"; },
        [](Runner& r) { r.input.feedback.state.position[0] += .03; },
        [](Runner& r) { r.input.feedback.state.position[3] += .1; },
        [](Runner& r) { r.input.feedback.wheel_velocity[0] = 2; },
        [](Runner& r) { r.input.feedback.state.position[0] = std::numeric_limits<double>::quiet_NaN(); }
    }};
    for (const auto& fault : faults) {
        Runner runner(reference); runner.Start(); runner.Advance(10.29); fault(runner);
        const auto denied = runner.Tick();
        Require(!denied.command && denied.status == s::TrajectoryTrackingStatus::kStopRequested, "feedback fault denies movement");
        runner.input.feedback = Feedback(runner.now, ++runner.feedback_sequence);
        const auto repeated = runner.Tick();
        Require(!repeated.command && repeated.stop_request_sequence == denied.stop_request_sequence,
                "healthy retry cannot erase the outstanding stop request");
    }
}
/** @brief Deadline, exception and identity/value attacks are independent of reference geometry. */
void CommandVerdictFailures(const std::shared_ptr<const s::AdmittedReference>& reference) {
    using Mutator = std::function<void(s::TrackingCommandVerdict&)>;
    const std::array<Mutator, 6> faults{{
        [](auto& v) { v.status = s::GeometryStatus::kBlocked; },
        [](auto& v) { v.status = s::GeometryStatus::kIndeterminate; },
        [](auto& v) { ++v.command.sequence; },
        [](auto& v) { v.command.linear_velocity += .001; },
        [](auto& v) { v.command.lease.valid_until += 1; },
        [](auto& v) { v.valid_until = v.command.valid_until - .001; }
    }};
    for (const auto& fault : faults) {
        Runner runner(reference); runner.Start(); runner.Advance(10.29);
        const auto result = runner.Tick([&](const auto& query) { auto verdict = Clear(query); fault(verdict); return verdict; });
        Require(!result.command && result.status == s::TrajectoryTrackingStatus::kStopRequested, "wrong command evidence cannot authorize execution");
    }
    Runner absent(reference); absent.Start(); absent.Advance(10.29);
    Require(!absent.Tick({}).command, "missing command geometry dependency fails closed");
    Runner throwing(reference); throwing.Start(); throwing.Advance(10.29);
    Require(!throwing.Tick([](const auto&) -> s::TrackingCommandVerdict { throw std::runtime_error("backend error"); }).command,
            "verifier exception never leaks a partial command");
    Runner delayed(reference); delayed.Start(); delayed.Advance(10.29);
    Require(!delayed.Tick([&](const auto& query) { delayed.now += .006; return Clear(query); }).command,
            "command verifier overrun cannot return a late executable command");
    Runner cancelled(reference); cancelled.Start(); cancelled.Advance(10.29);
    Require(!cancelled.Tick([&](const auto& query) {
        cancelled.tracker.RequestStop(cancelled.input.lease, cancelled.now, "stop during validation");
        return Clear(query);
    }).command, "a concurrent logical stop from a callback cannot be overwritten by a successful verdict");
    Runner reentrant(reference); reentrant.Start(); reentrant.Advance(10.29);
    Require(!reentrant.Tick([&](const auto& query) {
        Require(!reentrant.Tick().command, "reentrant callback cannot issue a second command");
        return Clear(query);
    }).command, "outer tick cannot issue a command after callback reentry latched a stop");
}
/** @brief Base grant revocation/renewal, clock regression and lost heartbeats are separate fault paths. */
void OwnershipFailures(const std::shared_ptr<const s::AdmittedReference>& reference) {
    for (unsigned fault = 0; fault < 5; ++fault) {
        Runner runner(reference); runner.Start(); runner.Advance(10.29);
        if (fault == 0) runner.input.lease.active = false;
        if (fault == 1) runner.input.lease.valid_until += 1;
        if (fault == 2) ++runner.input.lease.generation;
        if (fault == 3) runner.now = 10.2;
        if (fault == 4) runner.now = 10.5;
        Require(!runner.Tick().command, "grant changes, regressed time and missed heartbeat all request a stop");
    }
    Runner runner(reference);
    const auto request = runner.Tick(); runner.now = 10.21;
    runner.input.feedback = Feedback(runner.now, ++runner.feedback_sequence);
    runner.input.stop_receipt = Receipt(request, runner.now);
    ++runner.input.stop_receipt->request_sequence;
    Require(runner.Tick().status == s::TrajectoryTrackingStatus::kStopRequested, "unrelated stop receipt is not accepted");
}
/** @brief Final physical C2 handoff survives execution; equal-version and discontinuous inputs do not. */
void Handoff(const std::shared_ptr<const s::LocalGroundSnapshot>& ground,
             const std::shared_ptr<const s::AdmittedReference>& reference) {
    auto data = Reference().data(); data.timing.reference_version = 2; data.timing.start_stamp = 10.45;
    const s::TimedBSpline proposal(std::move(data));
    const auto prepared = s::PrepareReferenceHandoff(reference->trajectory(), proposal, 10.3);
    Require(static_cast<bool>(prepared.trajectory), "independent handoff preparation anchors physical derivatives");
    const auto next = Admit(ground, *prepared.trajectory);
    Runner runner(reference); runner.Start(); runner.Advance(10.3); runner.input.reference = next;
    auto tracked = runner.Tick();
    Require(tracked.command && tracked.command->timing.reference_version == 1,
            "future replacement retains the old reference until scheduled switch");
    runner.now = 10.37; runner.input.feedback = Feedback(runner.now, ++runner.feedback_sequence, reference->trajectory().Evaluate(runner.now));
    tracked = runner.Tick();
    Require(tracked.command && tracked.command->timing.reference_version == 1,
            "C2 handoff keeps the exact original predecessor");
    runner.Advance(10.45);
    const auto switched = runner.Tick();
    Require(switched.command && switched.command->timing.reference_version == 2, "scheduled switch consumes the final admitted C2 replacement");
    Runner discontinuous(reference); discontinuous.Start(); discontinuous.Advance(10.3);
    discontinuous.input.reference = Admit(ground, proposal);
    Require(!discontinuous.Tick().command, "unanchored discontinuous reference cannot replace the executing spline");
    Runner replay(reference); replay.Start(); replay.Advance(10.3); replay.input.reference = Admit(ground);
    Require(!replay.Tick().command, "new certificate with the same reference version cannot bypass replay checks");
}

/** @brief Native controller proposals retain the original gates and exact command identity. */
void NativeControllerGate(const std::shared_ptr<const s::AdmittedReference>& reference) {
    Runner runner(reference);
    unsigned solves = 0, checks = 0;
    const s::TrackingController controller = [&](const auto& query, const auto&, auto* output, auto*) {
        ++solves;
        Require(query.reference == reference && query.deadline == runner.now + .02 &&
                query.command.begin_stamp == query.deadline + .005, "native solve changed reference or charged budgets");
        *output = query.command;
        output->linear_velocity = query.feedback.state.velocity[0] + .0001;
        output->yaw_velocity = 0;
        return true;
    };
    const s::TrackingCommandVerifier verifier = [&](const auto& query) {
        ++checks;
        Require(query.command.linear_velocity == runner.input.feedback.state.velocity[0] + .0001,
                "native command was replaced by built-in feedback before actual-command verification");
        return Clear(query);
    };
    const auto tick = [&] { return runner.tracker.TickWithController(runner.input, [&] { return runner.now; }, verifier, controller); };
    const auto request = tick();
    Require(!request.command && solves == 0, "native solve ran before physical-stop confirmation");
    runner.now = 10.21; runner.input.feedback = Feedback(runner.now, ++runner.feedback_sequence);
    runner.input.stop_receipt = Receipt(request, runner.now);
    Require(tick().status == s::TrajectoryTrackingStatus::kWaiting && solves == 0,
            "native solve ran before the scheduled admitted reference");
    runner.input.stop_receipt.reset(); runner.Advance(10.25);
    const auto started = tick();
    Require(started.command && solves == 1 && checks == 1 && started.command->valid_until == runner.now + .04,
            "native solve did not preserve the original command horizon and independent verifier");
    runner.Advance(10.29); runner.input.lease.active = false;
    Require(!tick().command && solves == 1, "revoked grant reached native controller");

    for (unsigned fault = 0; fault < 9; ++fault) {
        Runner bad(reference); bad.Start(); bad.Advance(10.29);
        unsigned checked = 0;
        s::TrackingController broken;
        if (fault != 0) broken = [&](const auto& query, const auto&, auto* output, auto*) {
            *output = query.command; output->linear_velocity = bad.input.feedback.state.velocity[0];
            output->yaw_velocity = 0;
            if (fault == 1) return false;
            if (fault == 2) throw std::runtime_error("actual native dependency exception");
            if (fault == 3) bad.now = query.deadline;
            if (fault == 4) ++output->timing.reference_version;
            if (fault == 5) output->valid_until += 1;
            if (fault == 6) output->linear_velocity = 100;
            if (fault == 7) output->yaw_velocity = std::numeric_limits<double>::quiet_NaN();
            if (fault == 8) bad.tracker.RequestStop(bad.input.lease, bad.now, "stop during native solve");
            return true;
        };
        const auto result = bad.tracker.TickWithController(bad.input, [&] { return bad.now; },
            [&](const auto& query) { ++checked; return Clear(query); }, broken);
        Require(!result.command && checked == 0, "missing/late/malformed native solve bypassed the original gate");
        bad.Advance(10.33);
        Require(!bad.Tick().command, "failed native mode silently fell back and revived movement");
    }
    Runner stale(reference); stale.Start(); stale.Advance(10.29);
    stale.input.feedback.stamp = stale.now - .049;
    const auto expired = stale.tracker.TickWithController(stale.input, [&] { return stale.now; }, Clear,
        [&](const auto& query, const auto&, auto* output, auto*) {
            *output = query.command; output->linear_velocity = query.feedback.state.velocity[0];
            output->yaw_velocity = 0; stale.now += .002; return true;
        });
    Require(!expired.command, "native solve revived feedback that expired during computation");

    Runner budget(reference); budget.Start(); budget.Advance(10.29);
    double verified_at = 0;
    unsigned verifications = 0;
    const auto exceeded = budget.tracker.TickWithController(budget.input, [&] { return budget.now; },
        [&](const auto& query) {
            ++verifications;
            Require(query.deadline == verified_at + .005 && query.deadline < query.command.begin_stamp,
                    "unused solver time silently extended the independent verifier budget");
            budget.now = query.deadline;
            return Clear(query);
        }, [&](const auto& query, const auto&, auto* output, auto*) {
            *output = query.command; output->linear_velocity = query.feedback.state.velocity[0];
            output->yaw_velocity = 0; budget.now += .001; verified_at = budget.now;
            return true;
        });
    Require(!exceeded.command && verifications == 1,
            "verifier overran its own budget but used spare native solve time to authorize movement");
}
}  // namespace

/** @brief Standalone scenarios use real production admission and report the first failed obligation. */
int main() {
    try {
        const auto ground = Ground(); const auto reference = Admit(ground);
        AdmissionFailure(ground); CommandsAndStop(reference); FeedbackFailures(reference);
        CommandVerdictFailures(reference); OwnershipFailures(reference); Handoff(ground, reference);
        NativeControllerGate(reference);
        std::cout << "PASS trajectory tracker admission, identity, deadlines, physical-stop gating and C2 handoff\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "FAIL trajectory tracker: " << exception.what() << '\n';
        return 1;
    }
}
