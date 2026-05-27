/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/navigator/behavior_tree_navigator.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

bool BehaviorTreeNavigator::Configure(
    std::shared_ptr<BehaviorTreeEngine> engine,
    std::shared_ptr<BehaviorTreeContext> ctx,
    const std::string& default_behavior_tree_file) {
    engine_ = std::move(engine);
    ctx_ = std::move(ctx);
    if (!default_behavior_tree_file.empty()) {
        default_bt_xml_ =
            ResolveBehaviorTreeXmlPath(default_behavior_tree_file);
    }
    bt_server_ = std::make_unique<BtActionServer>(
        engine_, ctx_, default_bt_xml_,
        [this]() { return OnGoalReceived(); },
        [this]() { OnLoop(); }, [this]() { OnPreempt(); },
        [this](BtStatus status) { OnCompletion(status); });
    return true;
}

bool BehaviorTreeNavigator::Start(const std::string& bt_xml_file) {
    if (!bt_server_) {
        return false;
    }
    if (IsRunning()) {
        AWARN << GetName() << ": navigation already running.";
        return false;
    }
    std::string bt = bt_xml_file;
    if (!bt.empty() && !bt_server_->LoadBehaviorTree(bt)) {
        return false;
    }
    if (bt.empty() && !bt_server_->LoadBehaviorTree(default_bt_xml_)) {
        return false;
    }
    return bt_server_->Start();
}

bool BehaviorTreeNavigator::Cancel() {
    return bt_server_ ? bt_server_->Cancel() : false;
}

bool BehaviorTreeNavigator::Pause() {
    if (ctx_) {
        ctx_->pause_requested = true;
    }
    return true;
}

bool BehaviorTreeNavigator::Resume() {
    if (ctx_) {
        ctx_->pause_requested = false;
    }
    return true;
}

bool BehaviorTreeNavigator::IsRunning() const {
    return bt_server_ && bt_server_->IsRunning();
}

void BehaviorTreeNavigator::OnCompletion(BtStatus status) {
    ADEBUG << GetName() << " BT finished with status "
           << static_cast<int>(status);
    if (completion_hook_) {
        completion_hook_(status);
    }
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
