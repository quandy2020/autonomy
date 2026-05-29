/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autolink/action/simple_action_server.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/tasks/behavior_tree/bt_engine.hpp"
#include "autonomy/tasks/behavior_tree/bt_context.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief BT-backed action server (nav2 BtActionServer analogue, autolink transport).
 */
template <typename ActionT>
class BtActionServer
{
public:
    using Goal = typename ActionT::Goal;
    using Result = typename ActionT::Result;
    using Feedback = typename ActionT::Feedback;
    using GoalPtr = std::shared_ptr<const Goal>;
    using ResultPtr = std::shared_ptr<Result>;
    using FeedbackPtr = std::shared_ptr<Feedback>;
    using ActionServer = autolink::action::SimpleActionServer<ActionT>;

    using OnGoalReceivedCallback = std::function<bool(GoalPtr)>;
    using OnLoopCallback = std::function<void()>;
    using OnPreemptCallback = std::function<void(GoalPtr)>;
    using OnCompletionCallback =
        std::function<void(ResultPtr, RunStatus)>;

    BtActionServer(std::shared_ptr<BtEngine> engine,
                   std::shared_ptr<BtContext> ctx,
                   std::string action_name, std::string default_bt_xml,
                   OnGoalReceivedCallback on_goal_received,
                   OnLoopCallback on_loop, OnPreemptCallback on_preempt,
                   OnCompletionCallback on_completion);

    ~BtActionServer();

    bool OnConfigure();
    bool OnActivate(std::shared_ptr<autolink::Node> node);
    bool OnDeactivate();
    bool OnCleanup();

    bool LoadBehaviorTree(const std::string& bt_xml_filename = "");

    /** In-process navigation (Task API) without autolink action handles. */
    bool RunWithGoal(GoalPtr goal, const std::string& bt_xml_filename = "");

    bool Cancel();
    bool IsRunning() const;

    BT::Blackboard::Ptr GetBlackboard() const { return blackboard_; }
    std::string GetCurrentBTFilename() const { return current_bt_xml_; }
    std::string GetDefaultBTFilename() const { return default_bt_xml_; }

    GoalPtr AcceptPendingGoal();
    void TerminatePendingGoal();
    GoalPtr GetCurrentGoal() const;
    GoalPtr GetPendingGoal() const;
    void PublishFeedback(FeedbackPtr feedback);

    void SetInternalError(uint32_t error_code, const std::string& error_msg);
    bool PopulateInternalError(ResultPtr result) const;

private:
    void ExecuteCallback();
    void RunTree(GoalPtr goal, bool publish_action_results);
    bool PrepareTreeForGoal(GoalPtr goal, const std::string& bt_xml_filename);
    void FinishRun(RunStatus status, bool publish_action_results);

    std::shared_ptr<BtEngine> engine_;
    std::shared_ptr<BtContext> ctx_;
    std::string action_name_;
    std::string default_bt_xml_;
    std::string current_bt_xml_;

    OnGoalReceivedCallback on_goal_received_;
    OnLoopCallback on_loop_;
    OnPreemptCallback on_preempt_;
    OnCompletionCallback on_completion_;

    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<ActionServer> action_server_;

    BT::Blackboard::Ptr blackboard_;
    BT::Tree tree_;
    bool tree_loaded_{false};

    std::future<void> execution_future_;
    std::atomic<bool> stop_requested_{false};
    bool in_process_run_{false};
    mutable std::mutex mutex_;

    uint32_t internal_error_code_{0};
    std::string internal_error_msg_;
    std::chrono::milliseconds bt_loop_duration_{10};
};

template <typename ActionT>
BtActionServer<ActionT>::BtActionServer(
    std::shared_ptr<BtEngine> engine,
    std::shared_ptr<BtContext> ctx, std::string action_name,
    std::string default_bt_xml, OnGoalReceivedCallback on_goal_received,
    OnLoopCallback on_loop, OnPreemptCallback on_preempt,
    OnCompletionCallback on_completion)
    : engine_(std::move(engine)),
      ctx_(std::move(ctx)),
      action_name_(std::move(action_name)),
      default_bt_xml_(std::move(default_bt_xml)),
      on_goal_received_(std::move(on_goal_received)),
      on_loop_(std::move(on_loop)),
      on_preempt_(std::move(on_preempt)),
      on_completion_(std::move(on_completion)) {}

template <typename ActionT>
BtActionServer<ActionT>::~BtActionServer() {
    Cancel();
    OnCleanup();
}

template <typename ActionT>
bool BtActionServer<ActionT>::OnConfigure() {
    blackboard_ = BT::Blackboard::create();
    PopulateBlackboardDefaults(ctx_, blackboard_);
    if (ctx_ && ctx_->options.bt_loop_duration() > 0) {
        bt_loop_duration_ =
            std::chrono::milliseconds(ctx_->options.bt_loop_duration());
    }
    return true;
}

template <typename ActionT>
bool BtActionServer<ActionT>::OnActivate(std::shared_ptr<autolink::Node> node) {
    if (!node) {
        return false;
    }
    node_ = node;
    if (!LoadBehaviorTree(default_bt_xml_)) {
        AERROR << "BtActionServer: failed to load default BT: " << default_bt_xml_;
        return false;
    }
    action_server_ = std::make_shared<ActionServer>(
        node_, action_name_, [this]() { ExecuteCallback(); }, nullptr,
        std::chrono::milliseconds(500));
    AINFO << "BtActionServer active on '" << action_name_ << "'";
    return true;
}

template <typename ActionT>
bool BtActionServer<ActionT>::OnDeactivate() {
    Cancel();
    if (action_server_) {
        action_server_->Deactivate();
        action_server_.reset();
    }
    node_.reset();
    return true;
}

template <typename ActionT>
bool BtActionServer<ActionT>::OnCleanup() {
    OnDeactivate();
    blackboard_.reset();
    tree_loaded_ = false;
    current_bt_xml_.clear();
    return true;
}

template <typename ActionT>
bool BtActionServer<ActionT>::LoadBehaviorTree(
    const std::string& bt_xml_filename) {
    std::string file =
        bt_xml_filename.empty() ? default_bt_xml_ : bt_xml_filename;
    file = ResolveBehaviorTreeXmlPath(file);
    if (file.empty()) {
        AERROR << "BtActionServer: empty behavior tree file.";
        return false;
    }
    if (tree_loaded_ && current_bt_xml_ == file) {
        return true;
    }
    try {
        tree_ = engine_->CreateTreeFromFile(file, blackboard_);
        tree_loaded_ = true;
        current_bt_xml_ = file;
        return true;
    } catch (const std::exception& ex) {
        AERROR << "BtActionServer: failed to load BT " << file << ": "
               << ex.what();
        tree_loaded_ = false;
        return false;
    }
}

template <typename ActionT>
bool BtActionServer<ActionT>::PrepareTreeForGoal(
    GoalPtr goal, const std::string& bt_xml_filename) {
    if (!goal) {
        return false;
    }
    stop_requested_ = false;
    if (ctx_) {
        ctx_->cancel_requested = false;
        ctx_->pause_requested = false;
        ctx_->preempt_requested = false;
        ctx_->navigation_start = std::chrono::steady_clock::now();
        ctx_->number_recoveries = 0;
        blackboard_->set(kBlackboardNumberRecoveriesKey, 0);
    }
    internal_error_code_ = 0;
    internal_error_msg_.clear();
    const std::string bt_file = bt_xml_filename;
    if (!LoadBehaviorTree(bt_file)) {
        return false;
    }
    return on_goal_received_ && on_goal_received_(goal);
}

template <typename ActionT>
void BtActionServer<ActionT>::RunTree(GoalPtr /*goal*/,
                                      bool publish_action_results) {
    if (!tree_loaded_) {
        FinishRun(RunStatus::FAILED, publish_action_results);
        return;
    }

    auto is_canceling = [&]() {
        if (stop_requested_.load()) {
            return true;
        }
        if (publish_action_results && action_server_) {
            if (!action_server_->IsServerActive()) {
                return true;
            }
            return action_server_->IsCancelRequested();
        }
        return ctx_ && ctx_->cancel_requested.load();
    };

    auto on_loop = [&]() {
        if (publish_action_results && action_server_ &&
            action_server_->IsPreemptRequested() && on_preempt_) {
            on_preempt_(action_server_->GetPendingGoal());
        } else if (ctx_ && ctx_->preempt_requested.exchange(false) &&
                   on_preempt_) {
            on_preempt_(nullptr);
        }
        if (on_loop_) {
            on_loop_();
        }
    };

    const RunStatus status =
        engine_->Run(&tree_, on_loop, is_canceling, bt_loop_duration_);
    engine_->HaltAllActions(tree_);
    FinishRun(status, publish_action_results);
}

template <typename ActionT>
void BtActionServer<ActionT>::FinishRun(RunStatus status,
                                        bool publish_action_results) {
    auto result = std::make_shared<Result>();
    PopulateInternalError(result);
    if (on_completion_) {
        on_completion_(result, status);
    }
    if (publish_action_results && action_server_) {
        switch (status) {
            case RunStatus::SUCCEEDED:
                action_server_->SucceededCurrent(result);
                break;
            case RunStatus::CANCELED:
                action_server_->TerminateCurrent(result);
                break;
            default:
                action_server_->TerminateCurrent(result);
                break;
        }
    }
    in_process_run_ = false;
}

template <typename ActionT>
void BtActionServer<ActionT>::ExecuteCallback() {
    GoalPtr goal = GetCurrentGoal();
    if (!PrepareTreeForGoal(goal, goal ? goal->behavior_tree() : "")) {
        auto result = std::make_shared<Result>();
        PopulateInternalError(result);
        if (action_server_) {
            action_server_->TerminateCurrent(result);
        }
        return;
    }
    RunTree(goal, true);
}

template <typename ActionT>
bool BtActionServer<ActionT>::RunWithGoal(GoalPtr goal,
                                          const std::string& bt_xml_filename) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (IsRunning()) {
        AWARN << "BtActionServer: already running.";
        return false;
    }
    const std::string bt =
        bt_xml_filename.empty() && goal ? goal->behavior_tree() : bt_xml_filename;
    if (!PrepareTreeForGoal(goal, bt)) {
        return false;
    }
    in_process_run_ = true;
    execution_future_ =
        std::async(std::launch::async, [this, goal]() { RunTree(goal, false); });
    return true;
}

template <typename ActionT>
bool BtActionServer<ActionT>::Cancel() {
    stop_requested_ = true;
    if (ctx_) {
        ctx_->cancel_requested = true;
    }
    if (tree_loaded_) {
        engine_->HaltAllActions(tree_);
    }
    if (execution_future_.valid()) {
        execution_future_.wait();
    }
    return true;
}

template <typename ActionT>
bool BtActionServer<ActionT>::IsRunning() const {
    if (in_process_run_ && execution_future_.valid()) {
        return execution_future_.wait_for(std::chrono::milliseconds(0)) ==
               std::future_status::timeout;
    }
    if (action_server_ && action_server_->IsRunning()) {
        return true;
    }
    if (execution_future_.valid()) {
        return execution_future_.wait_for(std::chrono::milliseconds(0)) ==
               std::future_status::timeout;
    }
    return false;
}

template <typename ActionT>
typename BtActionServer<ActionT>::GoalPtr
BtActionServer<ActionT>::AcceptPendingGoal() {
    return action_server_ ? action_server_->AcceptPendingGoal() : nullptr;
}

template <typename ActionT>
void BtActionServer<ActionT>::TerminatePendingGoal() {
    if (action_server_) {
        action_server_->TerminatePendingGoal();
    }
}

template <typename ActionT>
typename BtActionServer<ActionT>::GoalPtr
BtActionServer<ActionT>::GetCurrentGoal() const {
    return action_server_ ? action_server_->GetCurrentGoal() : nullptr;
}

template <typename ActionT>
typename BtActionServer<ActionT>::GoalPtr
BtActionServer<ActionT>::GetPendingGoal() const {
    return action_server_ ? action_server_->GetPendingGoal() : nullptr;
}

template <typename ActionT>
void BtActionServer<ActionT>::PublishFeedback(FeedbackPtr feedback) {
    if (!action_server_ || !feedback) {
        return;
    }
    // In-process navigation (Task API / autonomy_ros) has no autolink goal handle.
    if (in_process_run_) {
        return;
    }
    action_server_->PublishFeedback(feedback);
}

template <typename ActionT>
void BtActionServer<ActionT>::SetInternalError(uint32_t error_code,
                                               const std::string& error_msg) {
    internal_error_code_ = error_code;
    internal_error_msg_ = error_msg;
}

template <typename ActionT>
bool BtActionServer<ActionT>::PopulateInternalError(ResultPtr result) const {
    if (!result || internal_error_code_ == 0) {
        return false;
    }
    using ErrorCode =
        std::decay_t<decltype(std::declval<Result>().error_code())>;
    result->set_error_code(static_cast<ErrorCode>(internal_error_code_));
    if (!internal_error_msg_.empty()) {
        result->set_error_msg(internal_error_msg_);
    }
    return true;
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
