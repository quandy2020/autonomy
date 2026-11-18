/* Copyright 2026 The Openbot Authors (duyongquan). */
/** @file native_spline_tracker_test.cpp
 * @brief Native message, fixed-frame reference, lifecycle and existing controller tests.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/native_spline_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

#include "autonomy/common/math/bspline.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/transform/buffer.hpp"

namespace s = autonomy::control::controller::policy_controller::sand_planner;
namespace p = autonomy::control::proto;
using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;
using TwistStamped = automsgs::msgs::geometry_msgs::TwistStamped;
using Path = automsgs::msgs::nav_msgs::Path;
namespace {
/** @brief Fail the bounded scenario with its actual cause. */
void Require(bool condition, const std::string& reason) {
    if (!condition) throw std::runtime_error(reason);
}
/** @brief Compare known geometric expectations without testing an implementation copy. */
void Near(double value, double expected, double tolerance = 1e-8) {
    Require(std::isfinite(value) && std::abs(value - expected) <= tolerance, "unexpected geometric result");
}
/** @brief Real stamped planar message fixture. */
PoseStamped Pose(double x = 0.0, double y = 0.0, double yaw = 0.0) {
    PoseStamped pose;
    pose.mutable_header()->set_frame_id("odom");
    pose.mutable_header()->mutable_stamp()->set_sec(1000);
    pose.mutable_pose()->mutable_position()->set_x(x);
    pose.mutable_pose()->mutable_position()->set_y(y);
    pose.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw / 2));
    pose.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw / 2));
    return pose;
}
/** @brief Stamped measured body velocity; has_twist distinguishes server fallback. */
TwistStamped Velocity() {
    TwistStamped velocity;
    velocity.mutable_header()->set_frame_id("base_link");
    velocity.mutable_header()->mutable_stamp()->set_sec(1000);
    velocity.mutable_twist()->mutable_linear()->set_x(0.0);
    velocity.mutable_twist()->mutable_angular()->set_z(0.0);
    return velocity;
}
/** @brief A cubic Bezier-equivalent shared spline with a nonstraight terminal tangent. */
std::vector<Eigen::Vector3d> Curve() { return {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {2, 1, 0}}; }

/** @brief Independent little-endian FLOAT32 encoding for real floor hit records. */
void Float(std::string& bytes, std::size_t offset, float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned i = 0; i < 4; ++i) bytes[offset + i] = static_cast<char>((bits >> (8 * i)) & 255);
}
/** @brief Explicit broad solver-fixture bounds; not a calibration claim for a physical base. */
s::DifferentialDriveLimits Drive() {
    s::DifferentialDriveLimits drive;
    drive.calibration_revision = 7; drive.track_width = .4;
    drive.wheel_velocity = {1, 1}; drive.wheel_acceleration = {1000, 1000}; drive.wheel_jerk = {100000, 100000};
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
    autonomy::perception::PointCloudBytes cloud{count, 1, 12, count * 12, false, "lidar", 10,
        {{"x", 0, autonomy::perception::FieldScalar::kFloat32, 1}, {"y", 4, autonomy::perception::FieldScalar::kFloat32, 1},
         {"z", 8, autonomy::perception::FieldScalar::kFloat32, 1}}, bytes};
    autonomy::perception::SensorDecodingOptions decoding; decoding.valid_for = 3; decoding.instantaneous_cloud = true;
    autonomy::perception::ObservationPacket packet;
    autonomy::perception::ObservationIngress ingress(autonomy::perception::SensorMode::kLidarOnly, decoding,
        [](autonomy::perception::SensorKind sensor, const std::string& frame, double stamp) {
            autonomy::perception::AcquisitionLocalization pose;
            pose.world_frame = "map"; pose.sensor_frame = frame; pose.map_epoch = 7;
            pose.calibration_revision = 3; pose.support = autonomy::perception::Mask(sensor); pose.healthy = true;
            pose.valid_from = stamp - 1; pose.valid_until = stamp + 3;
            pose.world_from_sensor.translation.z = 2;
            return pose;
        }, [&](auto value, double) { packet = std::move(value); return true; }, [] { return 10.05; });
    Require(ingress.PushCloud(cloud, 1) == autonomy::perception::ObservationIngressResult::kAccepted, "real floor wire records accepted");
    s::ExecutionAlignment alignment;
    alignment.world_frame = "map"; alignment.execution_frame = "odom";
    alignment.clock_domain = "tracker_test_clock"; alignment.map_epoch = 7; alignment.alignment_epoch = 9;
    alignment.healthy = true; alignment.valid_from = 9; alignment.valid_until = 13;
    s::LocalEsdfOptions options;
    options.origin = {-2, -2, -1.125}; options.dimensions = {16, 16, 16}; options.resolution = .25;
    options.mode = autonomy::perception::SensorMode::kLidarOnly; options.clock_domain = "tracker_test_clock"; options.revision = 17;
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

/** @brief Shared C2 boundary fitting materializes Eigen expressions and preserves far controls. */
void SharedVectorBoundary() {
    using Cubic = autonomy::common::math::ClampedCubicBSpline<Eigen::Vector3d>;
    const std::vector<Eigen::Vector3d> controls{
        {0, 0, 0}, {1, 0, .2}, {1, 1, .1}, {2, 1, 0}, {3, 2, 0}, {4, 2, 0}};
    const Cubic original(controls, {0, 0, 0, 0, .1, .45, 1, 1, 1, 1});
    const Eigen::Vector3d position(1, -2, .5), first(.4, -.2, .06), second(.4, .16, -.08);
    const auto start = original.StartControlPoints(position, first, second);
    auto replaced = controls;
    std::copy(start.begin(), start.end(), replaced.begin());
    const Cubic fitted(std::move(replaced), original.knots());
    Near((fitted.Evaluate(0) - position).norm(), 0);
    Near((fitted.EvaluateDerivative(1, 0) - first).norm(), 0);
    Near((fitted.EvaluateDerivative(2, 0) - second).norm(), 0);
    for (std::size_t i = 3; i < controls.size(); ++i)
        Near((fitted.control_points()[i] - controls[i]).norm(), 0);
    Near((original.control_points()[0] - controls[0]).norm(), 0);
    auto invalid = first;
    invalid.y() = std::numeric_limits<double>::quiet_NaN();
    bool rejected = false;
    try { original.StartControlPoints(position, invalid, second); }
    catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected, "nonfinite vector boundary was accepted");
}

/** @brief Controlled native interface dependency for message and failure contracts. */
class Recorder : public autonomy::control::common::ControllerInterface {
public:
    Path path;                         ///< Last SetPlan message, held in its original frame.
    int plans = 0, computes = 0, resets = 0;  ///< Observable native calls.
    bool fail = false, lateral = false, late = false;  ///< Injected native backend outcomes.
    bool omit_command = false;         ///< Backend reports success without supplying a command.
    double speed_limit = 0.0;           ///< Exact forwarded SanD execution speed.
    uint32_t ComputeVelocityCommands(const PoseStamped& pose, const TwistStamped& velocity,
                                    TwistStamped& command, autonomy::control::common::GoalChecker* checker,
                                    std::string& message) override {
        ++computes;
        Require(checker == nullptr, "local endpoint was treated as the task goal");
        Require(pose.header().frame_id() == "odom" && velocity.has_twist(), "feedback was replaced");
        if (omit_command) return p::CONTROLLER_RESULT_SUCCESS;
        command.mutable_twist()->mutable_linear()->set_x(0.3);
        command.mutable_twist()->mutable_linear()->set_y(lateral ? 0.1 : 0.0);
        command.mutable_twist()->mutable_angular()->set_z(0.2);
        if (late) std::this_thread::sleep_for(std::chrono::milliseconds(30));
        message = fail ? "native collision" : "";
        return fail ? p::CONTROLLER_RESULT_COLLISION : p::CONTROLLER_RESULT_SUCCESS;
    }
    bool IsGoalReached(double, double) override { return false; }
    void SetPlan(const Path& value) override { path = value; ++plans; }
    void SetSpeedLimit(const double& value, const bool& percentage) override {
        Require(!percentage, "speed limit changed units"); speed_limit = value;
    }
    void Reset() override { ++resets; }
};

/** @brief Translation/rotation, endpoint inclusion and tangent orientation use the shared curve. */
void GeometryAndRejections() {
    const auto controls = Curve();
    const auto anchor = Pose(3, -2, M_PI / 2);
    Path path;
    std::string error;
    Require(s::BuildSandPlannerPath(controls, anchor, 25, &path, &error), error);
    Require(path.poses_size() == 25 && path.header().frame_id() == "odom", "Path message contract");
    Near(path.poses(0).pose().position().x(), 3); Near(path.poses(0).pose().position().y(), -2);
    Near(path.poses(24).pose().position().x(), 2); Near(path.poses(24).pose().position().y(), 0);
    const auto midpoint = path.poses(12).pose();
    Near(midpoint.position().x(), 2.5); Near(midpoint.position().y(), -1);
    Near(std::atan2(2 * midpoint.orientation().w() * midpoint.orientation().z(),
                    1 - 2 * midpoint.orientation().z() * midpoint.orientation().z()), 3 * M_PI / 4);
    for (const auto& point : path.poses()) {
        Require(point.header().SerializeAsString() == anchor.header().SerializeAsString(), "fabricated point time/frame");
    }
    auto bad = controls; bad.back().z() = 0.2;
    Require(!s::BuildSandPlannerPath(bad, anchor, 25, &path, &error) && path.poses().empty(), "projected nonplanar spline");
    bad = controls; bad[1].x() = std::numeric_limits<double>::quiet_NaN();
    Require(!s::BuildSandPlannerPath(bad, anchor, 25, &path, &error), "accepted NaN controls");
    bad = controls; bad[0].x() = 1;
    Require(!s::BuildSandPlannerPath(bad, anchor, 25, &path, &error), "silently reanchored Q0");
    Require(!s::BuildSandPlannerPath(controls, anchor, 513, &path, &error), "unbounded Path samples");
    Require(!s::BuildSandPlannerPath(std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero()), anchor, 25, &path, &error), "stationary path admitted");
}

/** @brief Cached references stay fixed as the robot moves and command failures invalidate them. */
void NativeContract() {
    auto backend = std::make_shared<Recorder>();
    s::NativeSplineTracker tracker;
    tracker.BindController(backend, "base_link", 1.0);
    tracker.Activate();
    const auto pose = Pose(); const auto velocity = Velocity();
    std::string error;
    Require(tracker.SetReference(Curve(), pose, 32, 0.5, &error), error);
    const std::string original = tracker.reference_path().SerializeAsString();
    TwistStamped command;
    Require(tracker.Compute(Pose(0.1), velocity, 1000.05, &command, &error) == p::CONTROLLER_RESULT_SUCCESS, error);
    Require(backend->plans == 1 && backend->computes == 1, "native controller was bypassed");
    Require(tracker.reference_path().SerializeAsString() == original, "reference moved with robot");
    Near(command.twist().linear().x(), 0.3); Near(command.twist().angular().z(), 0.2);
    Near(backend->speed_limit, 0.5);
    Require(command.header().frame_id() == "base_link", "Twist is not in the body frame");
    Require(!tracker.CanReuse(std::chrono::steady_clock::now() + std::chrono::seconds(2)), "deadline renewed");
    backend->fail = true;
    Require(tracker.Compute(pose, velocity, 1000.05, &command, &error) == p::CONTROLLER_RESULT_COLLISION, "lost native failure");
    Near(command.twist().linear().x(), 0); Near(command.twist().angular().z(), 0);
    Require(tracker.reference_path().poses().empty(), "failed reference retained");
    backend->fail = false;
    Require(tracker.SetReference(Curve(), pose, 32, 0.5, &error), error);
    backend->omit_command = true;
    Require(tracker.Compute(pose, velocity, 1000.05, &command, &error) != p::CONTROLLER_RESULT_SUCCESS,
            "missing native command accepted as a measured zero command");
    Near(command.twist().linear().x(), 0);
    Require(tracker.reference_path().poses().empty(), "missing command retained reference");
    backend->omit_command = false;
    Require(tracker.SetReference(Curve(), pose, 32, 0.5, &error), error);
    auto missing = velocity; missing.clear_twist();
    Require(!tracker.ValidateFeedback(pose, missing, 1000.05, &error), "default server Twist accepted as measurement");
    Require(!tracker.ValidateFeedback(pose, velocity, 1001.0, &error), "stale measured state accepted");
    auto wrong = velocity; wrong.mutable_header()->set_frame_id("map");
    Require(!tracker.ValidateFeedback(pose, wrong, 1000.05, &error), "world velocity treated as body velocity");
    auto invalid = velocity; invalid.mutable_twist()->mutable_linear()->set_y(std::numeric_limits<double>::quiet_NaN());
    Require(!tracker.ValidateFeedback(pose, invalid, 1000.05, &error), "nonfinite measured feedback accepted");
    backend->lateral = true;
    Require(tracker.Compute(pose, velocity, 1000.05, &command, &error) != p::CONTROLLER_RESULT_SUCCESS, "nonplanar backend command accepted");
    Near(command.twist().linear().x(), 0);
    tracker.Deactivate();
    Require(!tracker.SetReference(Curve(), pose, 32, 0.5, &error), "inactive reference admitted");
}

/** @brief An optimization finishing after the original deadline cannot refresh its own validity. */
void LateNativeCommand() {
    auto backend = std::make_shared<Recorder>(); backend->late = true;
    s::NativeSplineTracker tracker; tracker.BindController(backend, "base_link", 0.01); tracker.Activate();
    std::string error; TwistStamped command;
    Require(tracker.SetReference(Curve(), Pose(), 32, 0.5, &error), error);
    Require(tracker.Compute(Pose(), Velocity(), 1000.05, &command, &error) != p::CONTROLLER_RESULT_SUCCESS, "late native output accepted");
    Near(command.twist().linear().x(), 0);
    // A long reference lease cannot authorize a result whose measured state
    // became stale during optimization, even when the reference itself is live.
    tracker.BindController(backend, "base_link", 1.0); tracker.Activate();
    Require(tracker.SetReference(Curve(), Pose(), 32, 0.5, &error), error);
    Require(tracker.Compute(Pose(), Velocity(), 1000.19, &command, &error) != p::CONTROLLER_RESULT_SUCCESS,
            "feedback aged during native solve but command was accepted");
    Near(command.twist().linear().x(), 0);
}

/** @brief Admission samples retain exact placement and native solves remain behind the stop gate. */
void AdmittedNativeExecution(s::NativeSplineTracker& native, const std::string& backend) {
    const auto ground = Ground();
    const auto reference = Admit(ground);
    Path path; std::string error;
    Require(s::BuildAdmittedPlannerPath(*reference, 64, &path, &error), error);
    Require(path.header().frame_id() == "odom" && path.poses_size() == 64,
            "admitted sample lost its actual reference frame");
    Near(path.poses(0).pose().position().x(), 0);
    Near(path.poses(63).pose().position().x(), .1);
    Near(path.poses(32).pose().position().x(), .1 * std::pow(32.0 / 63, 3));
    s::TrajectoryTrackerOptions options; options.stop_authority = "base_test_authority";
    s::TrajectoryTracker gate(options);
    s::TrajectoryTrackingInput input; input.reference = reference; input.lease = Lease();
    double now = 10.1; std::uint64_t sequence = 1;
    input.feedback = Feedback(now, sequence);
    const auto controller = native.TrackingCallback();
    const auto tick = [&] {
        const auto started = std::chrono::steady_clock::now();
        bool first_read = true;
        // Keep the fixture acquisition epoch while charging actual native work
        // to the production solve/verification deadlines without scaling time.
        const s::ExecutionClock clock = [&] {
            if (first_read) { first_read = false; return now; }
            return now + std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        };
        return gate.TickWithController(input, clock, Clear, controller);
    };
    const auto request = tick();
    Require(!request.command && request.status == s::TrajectoryTrackingStatus::kStopRequested,
            backend + " bypassed startup physical-stop confirmation");
    now = 10.21; input.feedback = Feedback(now, ++sequence); input.stop_receipt = Receipt(request, now);
    Require(tick().status == s::TrajectoryTrackingStatus::kWaiting, backend + " ran before scheduled reference");
    input.stop_receipt.reset(); now = 10.25; input.feedback = Feedback(now, ++sequence);
    const auto result = tick();
    Require(result.command.has_value(), backend + " admitted native gate: " + result.reason);
    Require(result.command->timing.reference_version == 1 && result.command->geometry_revision == 17 &&
            result.command->valid_until == now + .04 && result.command->begin_stamp == now + .02 + .005,
            "native command lost original identity, solve budget or absolute command expiry");
    Require(native.reference_path().poses_size() == 64, backend + " did not receive the admitted Path");
    Near(native.reference_path().poses(63).pose().position().x(), .1);
    now = 10.29; input.feedback = Feedback(now, ++sequence, reference->trajectory().Evaluate(now));
    input.lease.active = false;
    Require(!tick().command, backend + " continued after actual owner revocation");
    native.Reset();
}

/** @brief Exercise the actual NMPC and MPPI implementations with the existing map API. */
void ExistingBackends() {
    p::MPPIControllerOptions zero_bound;
    Require(!zero_bound.has_vx_min(), "omitted MPPI lower bound acquired explicit presence");
    zero_bound.set_vx_min(0);
    p::MPPIControllerOptions decoded;
    Require(decoded.ParseFromString(zero_bound.SerializeAsString()) && decoded.has_vx_min() && decoded.vx_min() == 0,
            "explicit forward-only MPPI bound was lost in the existing configuration message");
    autonomy::map::proto::Costmap2DOptions map_options;
    map_options.set_frame_id("odom"); map_options.set_width(8); map_options.set_height(8);
    map_options.set_resolution(0.1); map_options.add_plugins("noop_layer");
    auto map = std::make_shared<autonomy::map::costmap_2d::Costmap2DWrapper>(map_options, "sand_test");
    auto tf = std::shared_ptr<autonomy::transform::Buffer>(
        autonomy::transform::Buffer::Instance(), [](autonomy::transform::Buffer*) {});
    Require(tf != nullptr, "native transform singleton unavailable");
    // Existing costmap APIs create this bounded, deterministic free-space fixture.
    map->getCostmap()->resizeMap(80, 80, 0.1, -4, -4);
    for (unsigned y = 0; y < 80; ++y) {
        for (unsigned x = 0; x < 80; ++x) {
            map->getCostmap()->setCost(x, y, autonomy::map::costmap_2d::FREE_SPACE);
        }
    }
    std::vector<Eigen::Vector3d> straight{{0,0,0}, {0.5,0,0}, {1,0,0}, {1.5,0,0}};
    for (const std::string name : {"NMPCController", "MPPIController"}) {
        p::ControllerOptions options;
        options.set_controller_frequency(20);
        options.mutable_policy_controller_options()->set_sand_tracker(name);
        options.mutable_policy_controller_options()->set_sand_replan_period(1.0);
        options.mutable_nmpc_controller_options()->set_horizon_steps(8);
        options.mutable_nmpc_controller_options()->set_model_dt(0.05);
        // Exercise the real smoothing path, which requires more than 20 steps.
        options.mutable_mppi_controller_options()->set_time_steps(32);
        options.mutable_mppi_controller_options()->set_model_dt(0.05);
        options.mutable_mppi_controller_options()->set_batch_size(64);
        options.mutable_mppi_controller_options()->set_motion_model("DiffDrive");
        options.mutable_mppi_controller_options()->set_vx_min(0);
        // Match the native costmap's circular robot configuration; keep the
        // collision critic enabled rather than requesting an absent footprint.
        options.mutable_mppi_controller_options()->mutable_cost_critic()->set_enabled(true);
        options.mutable_mppi_controller_options()->mutable_cost_critic()->set_consider_footprint(false);
        s::NativeSplineTracker tracker;
        tracker.Configure(options, "sand_test", tf, map); tracker.Activate();
        std::string error; TwistStamped command;
        const bool reference_ready = tracker.SetReference(straight, Pose(), 32, 0.5, &error);
        Require(reference_ready, name + " SetReference: " + error);
        const auto status = tracker.Compute(Pose(), Velocity(), 1000.05, &command, &error);
        Require(status == p::CONTROLLER_RESULT_SUCCESS, name + " Compute: " + error);
        Require(std::isfinite(command.twist().linear().x()) && std::isfinite(command.twist().angular().z()), "nonfinite native solve");
        Require(command.twist().linear().x() >= 0, name + " ignored the configured forward-only lower bound");
        Near(command.twist().linear().y(), 0); Near(command.twist().linear().z(), 0);
        tracker.Reset();
        AdmittedNativeExecution(tracker, name);
        tracker.Cleanup();
    }
}
}  // namespace

/** @brief Run bounded integration scenarios against actual Autonomy messages and libraries. */
int main() {
    try {
        SharedVectorBoundary(); GeometryAndRejections(); NativeContract();
        LateNativeCommand(); ExistingBackends();
        std::cout << "native spline Path -> NMPC/MPPI -> TwistStamped scenarios passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
