/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/navigator/navigator.hpp"

#include <chrono>

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

bool Navigator::Configure(std::shared_ptr<BehaviorTreeEngine> engine,
                          std::shared_ptr<BehaviorTreeContext> ctx,
                          const std::string& default_behavior_tree_file) {
    engine_ = std::move(engine);
    ctx_ = std::move(ctx);
    if (!default_behavior_tree_file.empty()) {
        default_bt_xml_ =
            ResolveBehaviorTreeXmlPath(default_behavior_tree_file);
    }
    action_server_ = std::make_unique<BehaviorTreeActionServer>(
        engine_, ctx_, default_bt_xml_,
        [this]() { return OnGoalReceived(); },
        [this]() { OnLoop(); }, [this]() { OnPreempt(); },
        [this](RunStatus status) { OnCompletion(status); });
    return true;
}

bool Navigator::Start(const std::string& bt_xml_file) {
    if (!action_server_) {
        return false;
    }
    if (IsRunning()) {
        AWARN << GetName() << ": navigation already running.";
        return false;
    }
    if (ctx_) {
        ctx_->cancel_requested = false;
        ctx_->pause_requested = false;
        ctx_->navigation_start = std::chrono::steady_clock::now();
    }
    std::string bt = bt_xml_file;
    if (!bt.empty() && !action_server_->LoadBehaviorTree(bt)) {
        return false;
    }
    if (bt.empty() && !action_server_->LoadBehaviorTree(default_bt_xml_)) {
        return false;
    }
    return action_server_->Start();
}

bool Navigator::Cancel() {
    return action_server_ ? action_server_->Cancel() : false;
}

bool Navigator::Pause() {
    if (ctx_) {
        ctx_->pause_requested = true;
    }
    return true;
}

bool Navigator::Resume() {
    if (ctx_) {
        ctx_->pause_requested = false;
    }
    return true;
}

bool Navigator::IsRunning() const {
    return action_server_ && action_server_->IsRunning();
}

void Navigator::OnCompletion(RunStatus status) {
    ADEBUG << GetName() << " BT finished with status "
           << static_cast<int>(status);
    if (completion_hook_) {
        completion_hook_(status);
    }
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
