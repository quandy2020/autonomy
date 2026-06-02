/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/navigator/behavior_tree/bt_action_server.hpp"
#include "autonomy/navigator/behavior_tree/bt_context.hpp"
#include "autonomy/navigator/behavior_tree/bt_engine.hpp"

namespace autonomy {
namespace navigator {

enum class NavigationMode {
    NONE,
    NAVIGATE_TO_POSE,
    NAVIGATE_THROUGH_POSES,
};

/** Only one navigator may run at a time (nav2_core::NavigatorMuxer). */
class NavigatorMuxer
{
public:
    bool IsNavigating() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return !active_name_.empty();
    }

    void StartNavigating(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_name_.empty()) {
            AWARN << "Navigation requested while '" << active_name_ << "' is active.";
        }
        active_name_ = name;
    }

    void StopNavigating(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_name_ == name) {
            active_name_.clear();
        }
    }

private:
    mutable std::mutex mutex_;
    std::string active_name_;
};

/**
 * @brief BT navigator base (nav2_core::BehaviorTreeNavigator analogue).
 */
template <typename ActionT>
class BehaviorTreeNavigator
{
public:
    using Goal = typename ActionT::Goal;
    using Result = typename ActionT::Result;
    using GoalPtr = std::shared_ptr<const Goal>;
    using ResultPtr = std::shared_ptr<Result>;

    BehaviorTreeNavigator(std::string name,
                          std::shared_ptr<autolink::Node> node,
                          std::shared_ptr<behavior_tree::BtEngine> engine,
                          std::shared_ptr<behavior_tree::BtContext> context,
                          NavigatorMuxer* muxer,
                          const std::string& default_tree_xml);

    virtual ~BehaviorTreeNavigator() = default;

    const std::string& GetName() const { return name_; }

    bool StartWithGoal(GoalPtr goal, const std::string& tree_xml = "");

    bool IsRunning() const;
    bool IsInitialized() const { return static_cast<bool>(action_server_); }
    bool Cancel();

    void SetCompletionHook(std::function<void(behavior_tree::RunStatus)> hook) {
        completion_hook_ = std::move(hook);
    }

protected:
    virtual bool OnGoalReceived(GoalPtr goal) = 0;
    virtual void OnLoop() = 0;
    virtual void OnPreempt(GoalPtr goal) = 0;
    virtual void OnCompleted(ResultPtr result, behavior_tree::RunStatus status) = 0;

    std::shared_ptr<behavior_tree::BtEngine> engine_;
    std::shared_ptr<behavior_tree::BtContext> context_;
    NavigatorMuxer* muxer_{nullptr};
    std::unique_ptr<behavior_tree::BtActionServer<ActionT>> action_server_;
    std::string default_tree_xml_;
    std::function<void(behavior_tree::RunStatus)> completion_hook_;

private:
    bool HandleGoalReceived(GoalPtr goal);
    void HandleCompleted(ResultPtr result, behavior_tree::RunStatus status);

    std::string name_;
};

template <typename ActionT>
BehaviorTreeNavigator<ActionT>::BehaviorTreeNavigator(
    std::string name, std::shared_ptr<autolink::Node> node,
    std::shared_ptr<behavior_tree::BtEngine> engine,
    std::shared_ptr<behavior_tree::BtContext> context, NavigatorMuxer* muxer,
    const std::string& default_tree_xml)
    : engine_(std::move(engine)),
      context_(std::move(context)),
      muxer_(muxer),
      default_tree_xml_(default_tree_xml),
      name_(std::move(name)) {
    action_server_ = std::make_unique<behavior_tree::BtActionServer<ActionT>>(
        engine_, context_, name_, default_tree_xml_,
        [this](GoalPtr goal) { return HandleGoalReceived(goal); },
        [this]() { OnLoop(); }, [this](GoalPtr goal) { OnPreempt(goal); },
        [this](ResultPtr result, behavior_tree::RunStatus status) {
            HandleCompleted(result, status);
        });
    if (!action_server_->OnConfigure()) {
        AERROR << name_ << ": failed to set up behavior tree action server.";
        action_server_.reset();
        return;
    }
    if (!node) {
        return;
    }
    if (context_) {
        context_->autolink_node = node;
    }
    if (auto bb = action_server_->GetBlackboard()) {
        bb->set(behavior_tree::kBlackboardAutolinkNodeKey, node);
    }
    if (!action_server_->OnActivate(node)) {
        AERROR << name_ << ": failed to activate autolink action server.";
        action_server_.reset();
    }
}

template <typename ActionT>
bool BehaviorTreeNavigator<ActionT>::HandleGoalReceived(GoalPtr goal) {
    if (muxer_ && muxer_->IsNavigating()) {
        AERROR << name_ << ": rejected goal, another navigator is active.";
        return false;
    }
    if (!OnGoalReceived(goal)) {
        return false;
    }
    if (muxer_) {
        muxer_->StartNavigating(name_);
    }
    return true;
}

template <typename ActionT>
void BehaviorTreeNavigator<ActionT>::HandleCompleted(
    ResultPtr result, behavior_tree::RunStatus status) {
    if (muxer_) {
        muxer_->StopNavigating(name_);
    }
    OnCompleted(result, status);
    if (completion_hook_) {
        completion_hook_(status);
    }
}

template <typename ActionT>
bool BehaviorTreeNavigator<ActionT>::StartWithGoal(
    GoalPtr goal, const std::string& tree_xml) {
    if (!action_server_ || IsRunning()) {
        return false;
    }
    std::string xml = tree_xml;
    if (xml.empty() && goal) {
        xml = goal->behavior_tree();
    }
    return action_server_->RunWithGoal(goal, xml);
}

template <typename ActionT>
bool BehaviorTreeNavigator<ActionT>::IsRunning() const {
    return action_server_ && action_server_->IsRunning();
}

template <typename ActionT>
bool BehaviorTreeNavigator<ActionT>::Cancel() {
    return action_server_ ? action_server_->Cancel() : false;
}

}  // namespace navigator
}  // namespace autonomy
