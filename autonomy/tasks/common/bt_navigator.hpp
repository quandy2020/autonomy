/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_engine.hpp"
#include "autonomy/tasks/proto/bt_action.pb.h"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace tasks {
namespace common {

struct FeedbackUtils {
    std::string robot_frame;
    std::string global_frame;
    double transform_tolerance = 0.1;
    double local_survival_timeout = 120.0;
    std::shared_ptr<autonomy::transform::Buffer> tf;
    std::string default_bt_xml_filename;
    std::function<std::string(const std::string&)> bt_xml_path_resolver;
};

inline std::string ResolveBehaviorTreeFile(const std::string& bt_file,
                                           const FeedbackUtils& feedback) {
    if (bt_file.empty() || bt_file.find('/') != std::string::npos) {
        return bt_file;
    }
    if (feedback.bt_xml_path_resolver) {
        return feedback.bt_xml_path_resolver(bt_file);
    }
    return bt_file;
}

namespace behavior_tree = autonomy::tasks::behavior_tree;

using BtStatus = behavior_tree::BtStatus;
using OdomSmoother = autonomy::control::utils::OdomSmoother;

class NavigatorBase
{
public:
    virtual ~NavigatorBase() = default;

    virtual std::string GetNavigatorId() const = 0;

    virtual void RequestCancel() = 0;
};

class NavigatorMuxer
{
public:
    NavigatorMuxer() : current_navigator_("") {}

    bool IsNavigating() const {
        std::scoped_lock lock(mutex_);
        return !current_navigator_.empty();
    }

    void StartNavigating(const std::string& navigator_name) {
        std::scoped_lock lock(mutex_);
        if (!current_navigator_.empty()) {
            AERROR << "NavigatorMuxer: navigation requested while another "
                      "navigation task is in progress.";
        }
        current_navigator_ = navigator_name;
    }

    void StopNavigating(const std::string& navigator_name) {
        std::scoped_lock lock(mutex_);
        if (current_navigator_ != navigator_name) {
            AERROR << "NavigatorMuxer: stop from " << navigator_name
                   << " while current is " << current_navigator_;
        } else {
            current_navigator_.clear();
        }
    }

protected:
    std::string current_navigator_;
    mutable std::mutex mutex_;
};

/**
 * @brief In-process BT navigator: loads XML, runs tick loop, owns blackboard.
 */
template <class ActionT>
class BtNavigator : public NavigatorBase
{
public:
    using Goal = typename ActionT::Goal;
    using Result = typename ActionT::Result;
    using Feedback = typename ActionT::Feedback;

    BtNavigator() : NavigatorBase(), plugin_muxer_(nullptr) {}

    BtNavigator(
        std::string navigator_name, std::string default_bt_filepath,
        const autonomy::tasks::proto::TaskOptions& options,
        const std::shared_ptr<TaskContext>& task_context,
        const std::vector<std::string>& plugin_lib_names,
        const FeedbackUtils& feedback_utils,
        const std::shared_ptr<NavigatorMuxer>& plugin_muxer,
        std::shared_ptr<OdomSmoother> odom_smoother);

    virtual ~BtNavigator() { HaltTree(); }

    virtual std::string GetName() const { return navigator_name_; }

    std::string GetNavigatorId() const override { return navigator_name_; }

    void RequestCancel() override { cancel_requested_.store(true); }

    bool IsCancelRequested() const { return cancel_requested_.load(); }

    bool LoadBehaviorTree(const std::string& bt_xml_filename = "");

    void SetLoopDuration(std::chrono::milliseconds bt_loop_duration) {
        bt_loop_duration_ = bt_loop_duration;
        if (blackboard_) {
            blackboard_->set("bt_loop_duration", bt_loop_duration_);  // NOLINT
        }
    }

    BT::Blackboard::Ptr GetBlackboard() const { return blackboard_; }

    std::string GetCurrentBTFilename() const { return current_bt_xml_filename_; }

    std::string GetDefaultBTFilename() const { return default_bt_xml_filename_; }

    BtStatus Run(std::shared_ptr<const Goal> goal,
                 std::function<bool()> is_canceling = {});

    void SetPendingGoal(std::shared_ptr<const Goal> goal) {
        pending_goal_ = std::move(goal);
        preempt_requested_.store(pending_goal_ != nullptr);
    }

    std::shared_ptr<const Goal> AcceptPendingGoal() {
        if (!pending_goal_) {
            return nullptr;
        }
        current_goal_ = pending_goal_;
        pending_goal_.reset();
        preempt_requested_.store(false);
        return current_goal_;
    }

    void TerminatePendingGoal() {
        pending_goal_.reset();
        preempt_requested_.store(false);
    }

    std::shared_ptr<const Goal> GetCurrentGoal() const { return current_goal_; }

    std::shared_ptr<const Goal> GetPendingGoal() const { return pending_goal_; }

    bool IsPreemptRequested() const { return preempt_requested_.load(); }

    void PublishFeedback(std::shared_ptr<Feedback> /*feedback*/) {}

    const BT::Tree& GetTree() const { return tree_; }

    void HaltTree() {
        if (tree_.rootNode()) {
            tree_.haltTree();
        }
    }

    void SetInternalError(uint16_t error_code, const std::string& error_msg) {
        internal_error_code_ = error_code;
        internal_error_msg_ = error_msg;
    }

    void ResetInternalError() {
        internal_error_code_ = 0;
        internal_error_msg_.clear();
    }

    bool PopulateInternalError(std::shared_ptr<Result> result) {
        if (internal_error_code_ != 0 && result) {
            using ErrorCode = decltype(result->error_code());
            result->set_error_code(static_cast<ErrorCode>(internal_error_code_));
            result->set_error_msg(internal_error_msg_);
            return true;
        }
        return false;
    }

protected:
    bool OnGoalReceived(std::shared_ptr<const Goal> goal) {
        if (plugin_muxer_ && plugin_muxer_->IsNavigating()) {
            AERROR << "Requested navigation from " << GetName()
                   << " while another navigator is processing, rejecting.";
            return false;
        }
        const bool goal_accepted = GoalReceived(goal);
        if (goal_accepted && plugin_muxer_) {
            plugin_muxer_->StartNavigating(GetName());
        }
        return goal_accepted;
    }

    void OnCompletion(std::shared_ptr<Result> result, const BtStatus final_bt_status) {
        if (plugin_muxer_) {
            plugin_muxer_->StopNavigating(GetName());
        }
        GoalCompleted(result, final_bt_status);
    }

    void PopulateErrorCode(std::shared_ptr<Result> result) { (void)result; }

    void CleanErrorCodes() {}

    virtual bool GoalReceived(std::shared_ptr<const Goal> goal) = 0;
    virtual void OnLoop() = 0;
    virtual void OnPreempt(std::shared_ptr<const Goal> goal) = 0;
    virtual void GoalCompleted(std::shared_ptr<Result> result,
                               const BtStatus final_bt_status) = 0;

    std::string navigator_name_;
    std::string default_bt_filepath_;
    FeedbackUtils feedback_utils_;
    NavigatorMuxer* plugin_muxer_{nullptr};

    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;
    std::unique_ptr<behavior_tree::BtEngine> bt_engine_;
    std::chrono::milliseconds bt_loop_duration_{10};
    std::chrono::milliseconds wait_for_service_timeout_{1000};
    bool always_reload_bt_xml_{false};

    uint16_t internal_error_code_{0};
    std::string internal_error_msg_;

    std::shared_ptr<const Goal> current_goal_;
    std::shared_ptr<const Goal> pending_goal_;
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> preempt_requested_{false};
};

namespace {

void SetupCommonBlackboardKeys(
    const BT::Blackboard::Ptr& blackboard,
    const std::shared_ptr<TaskContext>& /*task_context*/,
    const proto::TaskOptions& options, const FeedbackUtils& feedback) {
    const auto bt_loop_ms = options.bt_loop_duration() > 0
                                ? std::chrono::milliseconds(
                                      options.bt_loop_duration())
                                : std::chrono::milliseconds(10);
    const auto server_timeout_ms =
        options.default_server_timeout() > 0
            ? std::chrono::milliseconds(options.default_server_timeout())
            : std::chrono::milliseconds(20);
    const auto wait_for_service_ms =
        options.wait_for_service_timeout() > 0
            ? std::chrono::milliseconds(options.wait_for_service_timeout())
            : std::chrono::milliseconds(1000);

    blackboard->set("bt_loop_duration", bt_loop_ms);              // NOLINT
    blackboard->set("server_timeout", server_timeout_ms);       // NOLINT
    blackboard->set("default_server_timeout", server_timeout_ms);  // NOLINT
    blackboard->set("wait_for_service_timeout", wait_for_service_ms);  // NOLINT

    blackboard->set("global_frame", feedback.global_frame);       // NOLINT
    blackboard->set("robot_base_frame", feedback.robot_frame);    // NOLINT
    blackboard->set("transform_tolerance", 0.1);                  // NOLINT
    blackboard->set("local_survival_timeout",
                    feedback.local_survival_timeout);  // NOLINT

    const std::string controller_id =
        !options.default_controller_id().empty()
            ? options.default_controller_id()
            : "FollowPath";
    const std::string planner_id =
        !options.default_planner_id().empty()
            ? options.default_planner_id()
            : "navfn_planner";

    blackboard->set("selected_controller", controller_id);  // NOLINT
    blackboard->set("selected_planner", planner_id);        // NOLINT
    blackboard->set("selected_smoother", "simple_smoother");  // NOLINT
    if (!options.default_goal_checker_id().empty()) {
        blackboard->set("selected_goal_checker",  // NOLINT
                        options.default_goal_checker_id());
    }

    if (options.goal_reached_tolerance() > 0.0) {
        blackboard->set("goal_reached_tol",  // NOLINT
                        options.goal_reached_tolerance());
    }

    commsgs::planning_msgs::Path empty_path;
    blackboard->set("path", empty_path);  // NOLINT

    blackboard->set("compute_path_error_code", int32_t{0});  // NOLINT
    blackboard->set("compute_path_error_msg", std::string{});  // NOLINT
    blackboard->set("follow_path_error_code", int32_t{0});     // NOLINT
    blackboard->set("follow_path_error_msg", std::string{});   // NOLINT
    blackboard->set("initial_pose_received", false);  // NOLINT
    blackboard->set("number_recoveries", 0);          // NOLINT
}

void SetupNavigateToPoseBlackboard(
    const BT::Blackboard::Ptr& blackboard,
    const std::shared_ptr<TaskContext>& task_context,
    const proto::TaskOptions& options, const FeedbackUtils& feedback) {
    if (!blackboard) {
        return;
    }
    SetupCommonBlackboardKeys(blackboard, task_context, options, feedback);
    blackboard->set("goal", commsgs::geometry_msgs::PoseStamped{});  // NOLINT
}

void SetupNavigatorBlackboard(
    const std::string& navigator_id, const BT::Blackboard::Ptr& blackboard,
    const std::shared_ptr<TaskContext>& task_context,
    const proto::TaskOptions& options, const FeedbackUtils& feedback) {
    if (!blackboard) {
        return;
    }

    if (navigator_id == "navigate_to_pose") {
        SetupNavigateToPoseBlackboard(blackboard, task_context, options,
                                      feedback);
        return;
    }

    SetupCommonBlackboardKeys(blackboard, task_context, options, feedback);

    if (navigator_id == "navigate_through_poses") {
        blackboard->set("goals", commsgs::planning_msgs::Goals{});  // NOLINT
        blackboard->set("compute_path_through_poses_error_code",
                        int32_t{0});  // NOLINT
        return;
    }

    if (navigator_id == "track_to_target") {
        blackboard->set("target_pose",
                        commsgs::geometry_msgs::PoseStamped{});  // NOLINT
        return;
    }

    if (navigator_id == "explore_to_anywhere") {
        blackboard->set("explore_goal",
                        commsgs::geometry_msgs::PoseStamped{});  // NOLINT
        blackboard->set("goal", commsgs::geometry_msgs::PoseStamped{});  // NOLINT
        return;
    }

    if (navigator_id == "navigate_to_docking") {
        blackboard->set("dock_id", std::string{});  // NOLINT
        blackboard->set("dock_pose",
                        commsgs::geometry_msgs::PoseStamped{});  // NOLINT
        return;
    }

    if (navigator_id == "teleop_drive") {
        double max_linear = 0.5;
        double max_angular = 1.5;
        double stale_timeout = 0.5;
        double projection_time = 1.5;
        double simulation_step = 0.1;
        if (options.has_teleop_drive_options()) {
            const auto& opts = options.teleop_drive_options();
            max_linear = opts.default_max_linear_vel();
            max_angular = opts.default_max_angular_vel();
            stale_timeout = opts.cmd_stale_timeout_sec();
            if (opts.projection_time_sec() > 0.0) {
                projection_time = opts.projection_time_sec();
            }
            if (opts.simulation_step_sec() > 0.0) {
                simulation_step = opts.simulation_step_sec();
            }
        }
        blackboard->set("teleop_mode",
                        static_cast<int>(proto::TELEOP_MOTION_VELOCITY));  // NOLINT
        blackboard->set("teleop_time_allowance", 0.0);  // NOLINT
        blackboard->set("teleop_max_linear_vel", max_linear);  // NOLINT
        blackboard->set("teleop_max_angular_vel", max_angular);  // NOLINT
        blackboard->set("teleop_cmd_stale_timeout_sec", stale_timeout);  // NOLINT
        blackboard->set("teleop_linear_distance", 0.0);  // NOLINT
        blackboard->set("teleop_linear_signed", 0.0);  // NOLINT
        blackboard->set("teleop_linear_speed", max_linear);  // NOLINT
        blackboard->set("teleop_rotation_angle", 0.0);  // NOLINT
        blackboard->set("teleop_angular_speed", max_angular);  // NOLINT
        blackboard->set("teleop_disable_collision_checks", false);  // NOLINT
        blackboard->set("teleop_projection_time_sec", projection_time);  // NOLINT
        blackboard->set("teleop_simulation_step_sec", simulation_step);  // NOLINT
        return;
    }

    SetupNavigateToPoseBlackboard(blackboard, task_context, options, feedback);
}

}  // namespace

template <class ActionT>
BtNavigator<ActionT>::BtNavigator(
    std::string navigator_name, std::string default_bt_filepath,
    const autonomy::tasks::proto::TaskOptions& options,
    const std::shared_ptr<TaskContext>& task_context,
    const std::vector<std::string>& plugin_lib_names,
    const FeedbackUtils& feedback_utils,
    const std::shared_ptr<NavigatorMuxer>& plugin_muxer,
    std::shared_ptr<OdomSmoother> odom_smoother)
    : NavigatorBase(),
      plugin_muxer_(plugin_muxer.get()),
      navigator_name_(std::move(navigator_name)),
      default_bt_filepath_(std::move(default_bt_filepath)),
      feedback_utils_(feedback_utils) {
    std::string default_bt_xml =
        feedback_utils.default_bt_xml_filename.empty()
            ? default_bt_filepath_
            : feedback_utils.default_bt_xml_filename;
    if (feedback_utils.bt_xml_path_resolver && !default_bt_xml.empty()) {
        default_bt_xml = feedback_utils.bt_xml_path_resolver(default_bt_xml);
    }
    default_bt_xml_filename_ = default_bt_xml;
    current_bt_xml_filename_ = default_bt_xml;

    const std::string plugin_lib_path = options.plugin_lib_path();
    bt_engine_ = std::make_unique<behavior_tree::BtEngine>(
        plugin_lib_names, plugin_lib_path);
    blackboard_ = BT::Blackboard::create();  // NOLINT
    blackboard_->set("bt_loop_duration", bt_loop_duration_);  // NOLINT
    blackboard_->set("wait_for_service_timeout", wait_for_service_timeout_);  // NOLINT
    blackboard_->set("task_context", task_context);  // NOLINT
    blackboard_->set("tf_buffer", feedback_utils.tf);  // NOLINT
    blackboard_->set("odom_smoother", odom_smoother);  // NOLINT
    SetupNavigatorBlackboard(navigator_name_, blackboard_, task_context, options,
                             feedback_utils);

    const auto bt_loop_ms = options.bt_loop_duration() > 0
                                ? std::chrono::milliseconds(options.bt_loop_duration())
                                : std::chrono::milliseconds(10);
    SetLoopDuration(bt_loop_ms);

    if (!default_bt_xml.empty() && !LoadBehaviorTree(default_bt_xml)) {
        AWARN << navigator_name_
              << ": failed to preload behavior tree: " << default_bt_xml;
    }
}

template <class ActionT>
bool BtNavigator<ActionT>::LoadBehaviorTree(
    const std::string& bt_xml_filename) {
    std::string filename =
        bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

    if (filename.empty()) {
        AERROR << "No behavior tree file specified";
        return false;
    }

    if (!always_reload_bt_xml_ && filename == current_bt_xml_filename_ &&
        tree_.rootNode()) {
        return true;
    }

    try {
        if (tree_.rootNode()) {
            tree_.haltTree();
        }
        tree_ = bt_engine_->CreateTreeFromFile(filename, blackboard_);
        current_bt_xml_filename_ = filename;
        return true;
    } catch (const std::exception& e) {
        AERROR << "Failed to load behavior tree from " << filename << ": "
               << e.what();
        return false;
    }
}

template <class ActionT>
BtStatus BtNavigator<ActionT>::Run(
    std::shared_ptr<const Goal> goal, std::function<bool()> is_canceling) {
    cancel_requested_.store(false);
    current_goal_ = goal;
    pending_goal_.reset();
    preempt_requested_.store(false);

    if (!goal || !OnGoalReceived(goal)) {
        auto result = std::make_shared<Result>();
        PopulateErrorCode(result);
        PopulateInternalError(result);
        OnCompletion(result, BtStatus::FAILED);
        CleanErrorCodes();
        current_goal_.reset();
        return BtStatus::FAILED;
    }

    if (!tree_.rootNode()) {
        AERROR << "Behavior tree not initialized for " << navigator_name_;
        current_goal_.reset();
        return BtStatus::FAILED;
    }

    CleanErrorCodes();
    ResetInternalError();

    auto is_canceling_fn = [this, is_canceling]() {
        if (cancel_requested_.load()) {
            return true;
        }
        return is_canceling && is_canceling();
    };

    if (blackboard_) {
        blackboard_->set("cancel_checker", is_canceling_fn);  // NOLINT
    }

    auto on_loop = [this]() {
        if (preempt_requested_.load() && pending_goal_) {
            OnPreempt(pending_goal_);
        }
        OnLoop();
    };

    const BtStatus status =
        bt_engine_->Run(&tree_, on_loop, is_canceling_fn, bt_loop_duration_);

    if (tree_.rootNode()) {
        tree_.haltTree();
    }

    auto result = std::make_shared<Result>();
    PopulateErrorCode(result);
    PopulateInternalError(result);
    OnCompletion(result, status);
    CleanErrorCodes();
    current_goal_.reset();
    return status;
}

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
