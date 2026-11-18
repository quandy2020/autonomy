/* Copyright 2026 The Openbot Authors (duyongquan). */
/** @file native_spline_tracker_test.cpp
 * @brief Native message, fixed-frame reference, lifecycle and existing controller tests.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/native_spline_tracker.hpp"

#include <algorithm>
#include <cmath>
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

/** @brief Exercise the actual NMPC and MPPI implementations with the existing map API. */
void ExistingBackends() {
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
        options.mutable_mppi_controller_options()->set_time_steps(12);
        options.mutable_mppi_controller_options()->set_model_dt(0.05);
        options.mutable_mppi_controller_options()->set_batch_size(64);
        options.mutable_mppi_controller_options()->set_motion_model("DiffDrive");
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
        Near(command.twist().linear().y(), 0); Near(command.twist().linear().z(), 0);
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
