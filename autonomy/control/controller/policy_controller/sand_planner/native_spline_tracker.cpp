/* Copyright 2026 The Openbot Authors (duyongquan). */
/** @file native_spline_tracker.cpp
 * @brief Shared spline sampling and delegation to the existing NMPC/MPPI plugins.
 */
#include "autonomy/control/controller/policy_controller/sand_planner/native_spline_tracker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>

#include "autonomy/common/math/bspline.hpp"
#include "autonomy/control/controller/mppi_controller/controller.hpp"
#include "autonomy/control/controller/nmpc_controller/controller.hpp"
#include "autonomy/control/controller/policy_controller/types.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace sand_planner {
namespace {
/** @brief Uniform rejection reporting without retaining a previous command. */
bool Reject(const std::string& reason, std::string* error) {
    if (error) *error = reason;
    return false;
}
/** @brief Validate a real planar robot pose before quaternion algebra. */
bool ValidPose(const automsgs::msgs::geometry_msgs::PoseStamped& pose) {
    const auto& p = pose.pose().position();
    const auto& q = pose.pose().orientation();
    const Eigen::Quaterniond rotation(q.w(), q.x(), q.y(), q.z());
    return !pose.header().frame_id().empty() &&
        pose.header().stamp().nanosec() < 1000000000 &&
        HeaderStampSeconds(pose.header()) > 0.0 &&
        Eigen::Vector3d(p.x(), p.y(), p.z()).allFinite() &&
        rotation.coeffs().allFinite() && std::abs(rotation.norm() - 1.0) < 1e-3 &&
        std::abs(rotation.toRotationMatrix()(2, 2) - 1.0) < 1e-3;
}
/** @brief Enforce differential-drive command axes before the outer safety filter. */
bool PlanarTwist(const automsgs::msgs::geometry_msgs::Twist& twist) {
    return std::isfinite(twist.linear().x()) && std::isfinite(twist.angular().z()) &&
        std::abs(twist.linear().y()) < 1e-9 && std::abs(twist.linear().z()) < 1e-9 &&
        std::abs(twist.angular().x()) < 1e-9 && std::abs(twist.angular().y()) < 1e-9;
}
/** @brief Check all measured axes without replacing odometry residuals with zero. */
bool FiniteTwist(const automsgs::msgs::geometry_msgs::Twist& twist) {
    return std::isfinite(twist.linear().x()) && std::isfinite(twist.linear().y()) &&
        std::isfinite(twist.linear().z()) && std::isfinite(twist.angular().x()) &&
        std::isfinite(twist.angular().y()) && std::isfinite(twist.angular().z());
}

/** @brief Encode acquisition seconds without rounding a measurement into the future. */
void Stamp(automsgs::msgs::std_msgs::Header* header, double seconds, const std::string& frame) {
    using Seconds = decltype(header->stamp().sec());
    if (!std::isfinite(seconds) || seconds <= 0.0 || frame.empty() ||
        static_cast<long double>(seconds) >= static_cast<long double>(std::numeric_limits<Seconds>::max()))
        throw std::invalid_argument("SanD tracker: time cannot be represented in automsgs Header");
    auto whole = static_cast<Seconds>(std::floor(seconds));
    const auto nanos = static_cast<std::uint32_t>(std::floor((seconds - whole) * 1e9));
    header->set_frame_id(frame);
    header->mutable_stamp()->set_sec(whole);
    header->mutable_stamp()->set_nanosec(nanos);
}

/** @brief A native proposal cannot outlive or detach from its original private admission. */
bool ValidTrackingQuery(const TrackingCommandQuery& query, double now) {
    if (!query.reference || !query.reference->geometry() || !query.reference->ground()) return false;
    const auto& reference = *query.reference;
    const auto& timing = reference.trajectory().data().timing;
    const auto& command = query.command;
    const auto& lease = command.lease;
    const auto& feedback = query.feedback;
    return std::isfinite(now) && std::isfinite(query.deadline) && now < query.deadline &&
        std::isfinite(command.begin_stamp) && std::isfinite(command.valid_until) &&
        query.deadline < command.begin_stamp && command.begin_stamp < command.valid_until &&
        now >= timing.start_stamp && now < reference.trajectory().EndStamp() &&
        command.valid_until <= reference.trajectory().EndStamp() && command.valid_until <= reference.valid_until() &&
        lease.active && lease.generation && lease.valid_from <= now && command.valid_until <= lease.valid_until &&
        std::isfinite(lease.actuator_watchdog) && lease.actuator_watchdog > 0 &&
        command.valid_until <= now + lease.actuator_watchdog &&
        lease.execution_id == timing.execution_id && lease.frame_id == timing.frame_id &&
        lease.alignment_epoch == timing.alignment_epoch &&
        lease.clock_domain == reference.safety().clock_domain &&
        lease.calibration_revision == reference.drive().calibration_revision &&
        command.timing.execution_id == timing.execution_id && command.timing.frame_id == timing.frame_id &&
        command.timing.alignment_epoch == timing.alignment_epoch && command.timing.reference_version == timing.reference_version &&
        command.timing.start_stamp == timing.start_stamp && command.timing.duration == timing.duration &&
        command.timing.valid_until == timing.valid_until && command.sequence &&
        command.feedback_sequence == feedback.sequence && feedback.sequence &&
        command.geometry_revision == reference.geometry()->options().revision && feedback.state.Finite() &&
        feedback.localization_healthy && feedback.actuator_healthy && feedback.frame_id == lease.frame_id &&
        feedback.clock_domain == lease.clock_domain && feedback.alignment_epoch == lease.alignment_epoch &&
        feedback.calibration_revision == lease.calibration_revision &&
        std::isfinite(feedback.stamp) && feedback.stamp >= lease.valid_from && feedback.stamp <= now;
}
}  // namespace

bool BuildSandPlannerPath(const std::vector<Eigen::Vector3d>& controls,
                         const automsgs::msgs::geometry_msgs::PoseStamped& anchor,
                         int sample_count, automsgs::msgs::nav_msgs::Path* path,
                         std::string* error) {
    if (!path) return Reject("SanD tracker: null Path output", error);
    path->Clear();
    if (error) error->clear();
    if (!ValidPose(anchor) || controls.size() < 4 || controls.size() > 64 ||
        sample_count < 2 || sample_count > 512) {
        return Reject("SanD tracker: invalid pose, controls or sample count", error);
    }
    for (const auto& control : controls) {
        if (!control.allFinite() || std::abs(control.z()) > 0.05) {
            return Reject("SanD tracker: nonfinite or nonplanar spline", error);
        }
    }
    if (controls.front().norm() > 1e-6) return Reject("SanD tracker: Q0 is not anchored", error);
    const auto& q = anchor.pose().orientation();
    const double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                  1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    const Eigen::AngleAxisd rotation(yaw, Eigen::Vector3d::UnitZ());
    const auto& p = anchor.pose().position();
    const Eigen::Vector3d translation(p.x(), p.y(), p.z());
    const ::autonomy::common::math::ClampedCubicBSpline<Eigen::Vector3d> curve(controls);
    const auto samples = curve.Sample(sample_count);
    automsgs::msgs::nav_msgs::Path result;
    result.mutable_header()->CopyFrom(anchor.header());
    double length = 0.0;
    double heading = yaw;
    for (int i = 0; i < sample_count; ++i) {
        const auto& sample = samples[static_cast<std::size_t>(i)];
        const Eigen::Vector3d position = rotation * sample + translation;
        const Eigen::Vector3d tangent = rotation * curve.EvaluateDerivative(
            1, static_cast<double>(i) / (sample_count - 1));
        if (!position.allFinite() || !tangent.allFinite()) {
            return Reject("SanD tracker: spline evaluation overflow", error);
        }
        if (i > 0) length += (sample - samples[static_cast<std::size_t>(i - 1)]).norm();
        if (tangent.head<2>().norm() > 1e-9) heading = std::atan2(tangent.y(), tangent.x());
        auto* point = result.add_poses();
        point->mutable_header()->CopyFrom(anchor.header());
        point->mutable_pose()->mutable_position()->set_x(position.x());
        point->mutable_pose()->mutable_position()->set_y(position.y());
        point->mutable_pose()->mutable_position()->set_z(position.z());
        point->mutable_pose()->mutable_orientation()->set_z(std::sin(heading * 0.5));
        point->mutable_pose()->mutable_orientation()->set_w(std::cos(heading * 0.5));
    }
    if (!std::isfinite(length) || length < 1e-6) return Reject("SanD tracker: stationary spline", error);
    path->Swap(&result);
    return true;
}

bool BuildAdmittedPlannerPath(const AdmittedReference& reference, int sample_count,
    automsgs::msgs::nav_msgs::Path* path, std::string* error) {
    if (!path) return Reject("SanD tracker: null admitted Path output", error);
    path->Clear();
    if (error) error->clear();
    if (sample_count < 2 || sample_count > 512)
        return Reject("SanD tracker: invalid admitted Path sample count", error);
    try {
        const auto& curve = reference.trajectory();
        const auto& timing = curve.data().timing;
        automsgs::msgs::nav_msgs::Path result;
        Stamp(result.mutable_header(), timing.start_stamp, timing.frame_id);
        double length = 0.0;
        Eigen::Vector3d previous = Eigen::Vector3d::Zero();
        for (int i = 0; i < sample_count; ++i) {
            const double fraction = static_cast<double>(i) / (sample_count - 1);
            const double stamp = i + 1 == sample_count ? curve.EndStamp() :
                timing.start_stamp + fraction * (curve.EndStamp() - timing.start_stamp);
            const auto value = curve.Evaluate(stamp);
            const Eigen::Vector3d position(value.position[0], value.position[1], value.position[2]);
            if (i) length += (position - previous).norm();
            previous = position;
            auto* pose = result.add_poses();
            Stamp(pose->mutable_header(), stamp, timing.frame_id);
            auto* point = pose->mutable_pose()->mutable_position();
            point->set_x(position.x()); point->set_y(position.y()); point->set_z(position.z());
            auto* rotation = pose->mutable_pose()->mutable_orientation();
            rotation->set_z(std::sin(value.position[3] / 2));
            rotation->set_w(std::cos(value.position[3] / 2));
        }
        if (!std::isfinite(length) || length < 1e-6)
            return Reject("SanD tracker: stationary admitted geometry needs a separate stop/rotation action", error);
        path->Swap(&result);
        return true;
    } catch (const std::exception& exception) {
        return Reject(exception.what(), error);
    }
}

void NativeSplineTracker::Configure(const proto::ControllerOptions& options,
                                    const std::string& name,
                                    std::shared_ptr<transform::Buffer> tf_buffer,
                                    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap) {
    Cleanup();
    const auto& policy = options.policy_controller_options();
    const std::string backend = policy.sand_tracker().empty() ? "NMPCController" : policy.sand_tracker();
    const double period = policy.sand_replan_period() > 0.0 ? policy.sand_replan_period() : 0.1;
    if (!std::isfinite(period) || period > 1.0) throw std::invalid_argument("SanD replan period must be in (0,1]");
    common::ControllerInterface::SharedPtr controller;
    if (backend == "NMPCController") {
        auto native = std::make_shared<nmpc_controller::NMPCController>();
        native->Configure(options, name + ".nmpc", tf_buffer, costmap);
        controller = std::move(native);
    } else if (backend == "MPPIController") {
        const auto& motion_model = options.mppi_controller_options().motion_model();
        if (!motion_model.empty() && motion_model != "DiffDrive") {
            throw std::invalid_argument("SanD MPPI tracker requires the existing DiffDrive motion model");
        }
        auto native = std::make_shared<mppi_controller::MPPIController>();
        native->Configure(options, name + ".mppi", tf_buffer, costmap);
        controller = std::move(native);
    } else {
        throw std::invalid_argument("SanD sand_tracker must be NMPCController or MPPIController");
    }
    BindController(std::move(controller), policy.robot_frame().empty() ? "base_link" : policy.robot_frame(), period);
    costmap_ = std::move(costmap);
    max_feedback_age_ = policy.max_obs_age() > 0.0 ? policy.max_obs_age() : 0.2;
}

void NativeSplineTracker::BindController(common::ControllerInterface::SharedPtr controller,
                                         const std::string& robot_frame, double replan_period) {
    Cleanup();
    if (!controller || robot_frame.empty() || !std::isfinite(replan_period) ||
        replan_period <= 0.0 || replan_period > 1.0) throw std::invalid_argument("Invalid native tracker binding");
    controller_ = std::move(controller);
    robot_frame_ = robot_frame;
    replan_period_ = replan_period;
}

void NativeSplineTracker::Activate() {
    if (auto native = std::dynamic_pointer_cast<nmpc_controller::NMPCController>(controller_)) native->Activate();
    if (auto native = std::dynamic_pointer_cast<mppi_controller::MPPIController>(controller_)) native->Activate();
    active_ = controller_ != nullptr;
}

void NativeSplineTracker::Deactivate() {
    active_ = false;
    Reset();
    if (auto native = std::dynamic_pointer_cast<nmpc_controller::NMPCController>(controller_)) native->Deactivate();
    if (auto native = std::dynamic_pointer_cast<mppi_controller::MPPIController>(controller_)) native->Deactivate();
}

void NativeSplineTracker::Cleanup() {
    Deactivate();
    if (auto native = std::dynamic_pointer_cast<nmpc_controller::NMPCController>(controller_)) native->Cleanup();
    if (auto native = std::dynamic_pointer_cast<mppi_controller::MPPIController>(controller_)) native->Cleanup();
    controller_.reset();
    costmap_.reset();
}

void NativeSplineTracker::Reset() {
    reference_path_.Clear();
    reference_deadline_ = {};
    admitted_reference_.reset();
    if (controller_) controller_->Reset();
}

bool NativeSplineTracker::CanReuse(std::chrono::steady_clock::time_point now) const {
    return active_ && controller_ && !reference_path_.poses().empty() && now < reference_deadline_;
}

bool NativeSplineTracker::SetReference(const std::vector<Eigen::Vector3d>& controls,
                                      const automsgs::msgs::geometry_msgs::PoseStamped& anchor,
                                      int sample_count, double execute_speed, std::string* error) {
    if (!active_ || !controller_ || !std::isfinite(execute_speed) || execute_speed <= 0.0) {
        Reset(); return Reject("SanD tracker: inactive controller or invalid speed", error);
    }
    automsgs::msgs::nav_msgs::Path path;
    if (!BuildSandPlannerPath(controls, anchor, sample_count, &path, error)) { Reset(); return false; }
    admitted_reference_.reset();
    try {
        controller_->SetPlan(path);
        controller_->SetSpeedLimit(execute_speed, false);
        reference_path_.Swap(&path);
        reference_deadline_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(replan_period_));
    } catch (const std::exception& exception) {
        Reset(); return Reject(std::string("SanD tracker SetPlan: ") + exception.what(), error);
    }
    return true;
}

bool NativeSplineTracker::ValidateFeedback(const automsgs::msgs::geometry_msgs::PoseStamped& pose,
                                           const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
                                           double now_sec, std::string* error) const {
    const double pose_stamp = HeaderStampSeconds(pose.header());
    const double velocity_stamp = HeaderStampSeconds(velocity.header());
    if (!ValidPose(pose) || !std::isfinite(now_sec) || !velocity.has_twist() || velocity_stamp <= 0.0 ||
        velocity.header().stamp().nanosec() >= 1000000000 ||
        velocity.header().frame_id() != robot_frame_ || !FiniteTwist(velocity.twist())) {
        return Reject("SanD tracker: Missing or invalid stamped feedback", error);
    }
    if (now_sec < pose_stamp || now_sec < velocity_stamp || now_sec - pose_stamp > max_feedback_age_ ||
        now_sec - velocity_stamp > max_feedback_age_ || std::abs(pose_stamp - velocity_stamp) > max_feedback_age_) {
        return Reject("SanD tracker: Stale or future feedback", error);
    }
    if (costmap_ && pose.header().frame_id() != costmap_->getGlobalFrameID()) {
        return Reject("SanD tracker: robot pose must use the native costmap frame", error);
    }
    return true;
}

uint32 NativeSplineTracker::Compute(const automsgs::msgs::geometry_msgs::PoseStamped& pose,
                                    const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
                                    double now_sec, automsgs::msgs::geometry_msgs::TwistStamped* command,
                                    std::string* error) {
    if (!command) return proto::CONTROLLER_RESULT_INTERNAL_ERROR;
    const auto solve_begin = std::chrono::steady_clock::now();
    command->Clear();
    command->mutable_header()->CopyFrom(pose.header());
    command->mutable_header()->set_frame_id(robot_frame_);
    if (!CanReuse(std::chrono::steady_clock::now()) ||
        !ValidateFeedback(pose, velocity, now_sec, error) ||
        reference_path_.header().frame_id() != pose.header().frame_id()) {
        Reset();
        if (error && error->empty()) *error = "SanD tracker: expired or mismatched reference";
        return proto::CONTROLLER_RESULT_NO_VALID_CMD;
    }
    automsgs::msgs::geometry_msgs::TwistStamped candidate;
    candidate.mutable_header()->CopyFrom(command->header());
    std::string detail;
    uint32 status = proto::CONTROLLER_RESULT_INTERNAL_ERROR;
    try {
        status = controller_->ComputeVelocityCommands(pose, velocity, candidate, nullptr, detail);
    } catch (const std::exception& exception) { detail = exception.what(); }
    const auto solve_end = std::chrono::steady_clock::now();
    const double feedback_now = now_sec + std::chrono::duration<double>(solve_end - solve_begin).count();
    if (status != proto::CONTROLLER_RESULT_SUCCESS || !candidate.has_twist() || !PlanarTwist(candidate.twist()) ||
        !CanReuse(solve_end) || !ValidateFeedback(pose, velocity, feedback_now, &detail)) {
        Reset();
        Reject(detail.empty() ? "SanD tracker: invalid or late native command" : detail, error);
        return status == proto::CONTROLLER_RESULT_SUCCESS ? static_cast<uint32>(proto::CONTROLLER_RESULT_NO_VALID_CMD) : status;
    }
    command->mutable_twist()->CopyFrom(candidate.twist());
    if (error) error->clear();
    return proto::CONTROLLER_RESULT_SUCCESS;
}

bool NativeSplineTracker::SolveTrackingCommand(const TrackingCommandQuery& query, const ExecutionClock& clock,
    automsgs::msgs::geometry_msgs::TwistStamped* command, std::string* error) {
    if (!command) return Reject("SanD tracker: null native command output", error);
    command->Clear();
    if (error) error->clear();
    if (in_admitted_solve_) { Reset(); return Reject("SanD tracker: admitted solver callback reentry", error); }
    struct Scope {
        bool& entered; ///< Owner flag remains set during all native callbacks.
        /** @brief Mark the serialized native solve as entered until stack unwind. */
        explicit Scope(bool& value) : entered(value) { entered = true; }
        /** @brief Release callback entry even when the native dependency throws. */
        ~Scope() { entered = false; }
    } scope(in_admitted_solve_);
    try {
        const double now = clock ? clock() : std::numeric_limits<double>::quiet_NaN();
        if (!active_ || !controller_ || !ValidTrackingQuery(query, now)) {
            Reset(); return Reject("SanD tracker: incompatible or expired admitted command query", error);
        }
        const auto reference = query.reference;
        if (admitted_reference_ != reference || !CanReuse(std::chrono::steady_clock::now())) {
            automsgs::msgs::nav_msgs::Path path;
            if (!BuildAdmittedPlannerPath(*reference, 64, &path, error)) { Reset(); return false; }
            controller_->SetPlan(path);
            const auto& drive = reference->drive();
            controller_->SetSpeedLimit(std::min(drive.wheel_velocity[0], drive.wheel_velocity[1]), false);
            reference_path_.Swap(&path);
            admitted_reference_ = reference;
            const double horizon = std::min({replan_period_, reference->valid_until() - now,
                reference->trajectory().EndStamp() - now, query.command.lease.valid_until - now});
            reference_deadline_ = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(horizon));
        }
        const double solve_at = clock();
        if (!ValidTrackingQuery(query, solve_at) || solve_at < now) {
            Reset(); return Reject("SanD tracker: native reference setup consumed its original solve budget", error);
        }
        const auto& feedback = query.feedback;
        automsgs::msgs::geometry_msgs::PoseStamped pose;
        Stamp(pose.mutable_header(), feedback.stamp, feedback.frame_id);
        auto* point = pose.mutable_pose()->mutable_position();
        point->set_x(feedback.state.position[0]); point->set_y(feedback.state.position[1]);
        point->set_z(feedback.state.position[2]);
        const double yaw = feedback.state.position[3];
        pose.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw / 2));
        pose.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw / 2));
        automsgs::msgs::geometry_msgs::TwistStamped velocity;
        Stamp(velocity.mutable_header(), feedback.stamp, robot_frame_);
        velocity.mutable_twist()->mutable_linear()->set_x(std::cos(yaw) * feedback.state.velocity[0] +
                                                         std::sin(yaw) * feedback.state.velocity[1]);
        velocity.mutable_twist()->mutable_linear()->set_y(-std::sin(yaw) * feedback.state.velocity[0] +
                                                          std::cos(yaw) * feedback.state.velocity[1]);
        velocity.mutable_twist()->mutable_linear()->set_z(feedback.state.velocity[2]);
        velocity.mutable_twist()->mutable_angular()->set_z(feedback.state.velocity[3]);
        automsgs::msgs::geometry_msgs::TwistStamped candidate;
        const auto status = Compute(pose, velocity, solve_at, &candidate, error);
        const double finished = clock();
        if (status != proto::CONTROLLER_RESULT_SUCCESS || admitted_reference_ != reference ||
            finished < solve_at || !ValidTrackingQuery(query, finished)) {
            Reset(); return Reject("SanD tracker: native admitted solve failed, changed reference or exceeded its deadline", error);
        }
        command->Swap(&candidate);
        return true;
    } catch (const std::exception& exception) {
        Reset(); return Reject(exception.what(), error);
    }
}

TrackingController NativeSplineTracker::TrackingCallback() {
    return [this](const TrackingCommandQuery& query, const ExecutionClock& clock,
                  TrackingCommand* command, std::string* error) {
        if (!command) return Reject("SanD tracker: null gated command output", error);
        *command = {};
        automsgs::msgs::geometry_msgs::TwistStamped velocity;
        if (!SolveTrackingCommand(query, clock, &velocity, error)) return false;
        *command = query.command;
        command->linear_velocity = velocity.twist().linear().x();
        command->yaw_velocity = velocity.twist().angular().z();
        return true;
    };
}

}  // namespace sand_planner
}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
