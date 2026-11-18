/* Copyright 2026 The Openbot Authors (duyongquan). */
/**
 * @file native_spline_tracker.hpp
 * @brief SanD reference adapter for existing Autonomy NMPC/MPPI controllers.
 */
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/trajectory_tracker.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace sand_planner {

/**
 * @brief Sample the shared-math SanD curve into an existing automsgs Path.
 * @param controls Complete robot-frame XYZ controls, including anchored Q0.
 * @param anchor Actual robot pose at inference start, in the execution frame.
 * @param sample_count Output samples, bounded to [2,512].
 * @param path Existing wire message; cleared on failure.
 * @param error Rejection reason, if requested.
 * @return False for malformed, stationary, nonplanar or nonfinite input.
 * The message is a geometric reference. It does not encode a timed trajectory.
 */
bool BuildSandPlannerPath(const std::vector<Eigen::Vector3d>& controls,
                         const automsgs::msgs::geometry_msgs::PoseStamped& anchor,
                         int sample_count, automsgs::msgs::nav_msgs::Path* path,
                         std::string* error = nullptr);

/** @brief Sample the exact admitted physical reference into existing automsgs poses.
 * @param reference Private immutable result of actual combined trajectory admission.
 * @param sample_count Bounded [2,512] geometric samples over the original represented interval.
 * @param path Output in the reference's execution frame, cleared on failure.
 * @param error Optional rejection reason.
 * @return False for invalid timestamps, nonfinite or stationary geometry.
 * XYZ and yaw come from TimedBSpline/common/math, preserving nonuniform knots,
 * original absolute placement and endpoint yaw. Path remains a geometric solver
 * input; per-pose timestamps alone do not add timed tracking to NMPC/MPPI.
 */
bool BuildAdmittedPlannerPath(const AdmittedReference& reference, int sample_count,
    automsgs::msgs::nav_msgs::Path* path, std::string* error = nullptr);

/**
 * @brief Owns reference lifetime and an existing ControllerInterface backend.
 *
 * No optimizer, motion model or parallel feedback/command message is defined
 * here. Callers serialize Configure, lifecycle, reference and Compute calls.
 * SanD inference uses a monotonic cadence; all message age checks use the
 * caller's acquisition-compatible clock in seconds.
 */
class NativeSplineTracker {
public:
    /** @brief Configure the existing NMPCController or MPPIController using ControllerOptions. */
    void Configure(const proto::ControllerOptions& options, const std::string& name,
                   std::shared_ptr<transform::Buffer> tf_buffer,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap);
    /** @brief Release backend resources and invalidate the reference. */
    void Cleanup();
    /** @brief Activate the configured native backend. */
    void Activate();
    /** @brief Invalidate execution before deactivating the native backend. */
    void Deactivate();
    /** @brief Clear the local reference and native optimizer warm start. */
    void Reset();
    /** @brief Whether a reference can be reused before the next monotonic replan deadline. */
    bool CanReuse(std::chrono::steady_clock::time_point now) const;
    /** @brief Submit the entire selected curve using the native SetPlan(Path) contract. */
    bool SetReference(const std::vector<Eigen::Vector3d>& controls,
                      const automsgs::msgs::geometry_msgs::PoseStamped& anchor,
                      int sample_count, double execute_speed, std::string* error);
    /**
     * @brief Invoke the native feedback controller and return its TwistStamped.
     * @param now_sec Acquisition-compatible seconds; never steady-clock epoch seconds.
     * Local endpoints are not task goals: the outer PolicyController owns task completion.
     * Failed, expired, stale or nonplanar commands are cleared and invalidate the reference.
     */
    uint32 Compute(const automsgs::msgs::geometry_msgs::PoseStamped& pose,
                   const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
                   double now_sec, automsgs::msgs::geometry_msgs::TwistStamped* command,
                   std::string* error);
    /** @brief Validate real stamped feedback before inference or cached tracking. */
    bool ValidateFeedback(const automsgs::msgs::geometry_msgs::PoseStamped& pose,
                          const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
                          double now_sec, std::string* error) const;
    /** @brief Native solver callback for TrajectoryTracker::TickWithController.
     * @param query Exact active admission, grant, measured feedback and original solve deadline.
     * @param clock Actual acquisition/execution clock, read before and after native work.
     * @param command Existing body-frame TwistStamped proposal, cleared on every failure.
     * @param error Optional explicit native/deadline rejection.
     * @return True only for a finite timely native proposal; this never dispatches it.
     * Preserves the private admitted reference, source horizons and fixed Path.
     * The outer tracker still checks wheel limits, command sweep and stop authority.
     * Caller serializes lifecycle and ticks; callback reentry clears the reference.
     */
    bool SolveTrackingCommand(const TrackingCommandQuery& query, const ExecutionClock& clock,
        automsgs::msgs::geometry_msgs::TwistStamped* command, std::string* error);
    /** @brief Bind this native solver to the existing tracker without defining a new message.
     * The callback preserves every TrackingCommand identity/time field and only
     * inserts the actual TwistStamped body-forward velocity and yaw rate.
     * This object must outlive the callback; calls share its serialized lifecycle.
     */
    TrackingController TrackingCallback();
    /** @brief Inspect the retained native reference without copying the message. */
    const automsgs::msgs::nav_msgs::Path& reference_path() const { return reference_path_; }
    /** @brief Bind an externally configured ControllerInterface for native integration tests. */
    void BindController(common::ControllerInterface::SharedPtr controller,
                        const std::string& robot_frame, double replan_period);

private:
    common::ControllerInterface::SharedPtr controller_;  ///< Existing solver/controller implementation.
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;  ///< Existing map owner; read only through its API.
    automsgs::msgs::nav_msgs::Path reference_path_;  ///< Full selected spline sampled in the fixed execution frame.
    std::string robot_frame_ = "base_link";  ///< Expected measured body-velocity and command frame.
    double replan_period_ = 0.1;  ///< Monotonic SanD generation period, seconds.
    double max_feedback_age_ = 0.2;  ///< Maximum acquisition age and pose/velocity skew, seconds.
    bool active_ = false;  ///< Native lifecycle permission; Reset alone never activates it.
    std::shared_ptr<const AdmittedReference> admitted_reference_; ///< Exact private admission supplied by the tracker.
    bool in_admitted_solve_ = false; ///< Prevent callback reentry from sharing a native optimizer.
    std::chrono::steady_clock::time_point reference_deadline_{};  ///< Original bounded reuse deadline; never renewed by Compute.
};

}  // namespace sand_planner
}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
