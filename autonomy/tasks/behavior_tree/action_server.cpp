/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/action_server.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

BehaviorTreeActionServer::BehaviorTreeActionServer(
    std::shared_ptr<BehaviorTreeEngine> engine,
    std::shared_ptr<BehaviorTreeContext> ctx, std::string default_bt_xml,
    OnGoalReceivedCallback on_goal_received, OnLoopCallback on_loop,
    OnPreemptCallback on_preempt, OnCompletionCallback on_completion)
    : engine_(std::move(engine)),
      ctx_(std::move(ctx)),
      default_bt_xml_(std::move(default_bt_xml)),
      on_goal_received_(std::move(on_goal_received)),
      on_loop_(std::move(on_loop)),
      on_preempt_(std::move(on_preempt)),
      on_completion_(std::move(on_completion)) {
    blackboard_ = BT::Blackboard::create();
    PopulateBlackboardDefaults(ctx_, blackboard_);
}

BehaviorTreeActionServer::~BehaviorTreeActionServer() {
    Cancel();
}

bool BehaviorTreeActionServer::LoadBehaviorTree(const std::string& bt_xml_filename) {
    std::string file = bt_xml_filename.empty() ? default_bt_xml_ : bt_xml_filename;
    file = ResolveBehaviorTreeXmlPath(file);
    if (file.empty()) {
        AERROR << "BehaviorTreeActionServer: empty behavior tree file.";
        return false;
    }
    try {
        tree_ = engine_->CreateTreeFromFile(file, blackboard_);
        tree_loaded_ = true;
        current_bt_xml_ = file;
        return true;
    } catch (const std::exception& ex) {
        AERROR << "BehaviorTreeActionServer: failed to load BT XML " << file << ": "
               << ex.what();
        tree_loaded_ = false;
        return false;
    }
}

bool BehaviorTreeActionServer::Start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (IsRunning()) {
        AWARN << "BehaviorTreeActionServer: already running.";
        return false;
    }
    if (!tree_loaded_ && !LoadBehaviorTree()) {
        return false;
    }
    stop_requested_ = false;
  if (ctx_) {
        ctx_->cancel_requested = false;
        ctx_->pause_requested = false;
        ctx_->navigation_start = std::chrono::steady_clock::now();
        ctx_->number_recoveries = 0;
        blackboard_->set(kBlackboardNumberRecoveriesKey, 0);
    }
    if (on_goal_received_ && !on_goal_received_()) {
        return false;
    }
    execution_future_ =
        std::async(std::launch::async, [this]() { ExecuteCallback(); });
    return true;
}

bool BehaviorTreeActionServer::Cancel() {
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

bool BehaviorTreeActionServer::IsRunning() const {
    if (!execution_future_.valid()) {
        return false;
    }
    return execution_future_.wait_for(std::chrono::milliseconds(0)) ==
           std::future_status::timeout;
}

void BehaviorTreeActionServer::ExecuteCallback() {
    if (!tree_loaded_) {
        if (on_completion_) {
            on_completion_(RunStatus::FAILED);
        }
        return;
    }

    const auto loop_ms = std::chrono::milliseconds(
        ctx_ && ctx_->options.bt_loop_duration() > 0
            ? ctx_->options.bt_loop_duration()
            : 10);

    auto cancel_checker = [this]() {
        return stop_requested_.load() ||
               (ctx_ && ctx_->cancel_requested.load());
    };

    auto on_loop = [this]() {
        if (ctx_ && ctx_->preempt_requested.exchange(false) && on_preempt_) {
            on_preempt_();
        }
        if (on_loop_) {
            on_loop_();
        }
    };

    const RunStatus status =
        engine_->Run(&tree_, on_loop, cancel_checker, loop_ms);

    if (on_completion_) {
        on_completion_(status);
    }
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
