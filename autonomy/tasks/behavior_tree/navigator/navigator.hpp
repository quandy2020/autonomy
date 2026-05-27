/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "autonomy/tasks/behavior_tree/action_server.hpp"
#include "autonomy/tasks/behavior_tree/context.hpp"
#include "autonomy/tasks/behavior_tree/engine.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief Base class for one BT navigation session (e.g. navigate_to_pose).
 *
 * Owns a BehaviorTreeActionServer and implements Start/Cancel/Pause/Resume for a single
 * mission type. BehaviorTreeNavigationEngine composes multiple Navigator instances.
 */
class Navigator
{
public:
    virtual ~Navigator() = default;

    virtual std::string GetName() const = 0;

    bool Configure(std::shared_ptr<BehaviorTreeEngine> engine,
                   std::shared_ptr<BehaviorTreeContext> ctx,
                   const std::string& default_behavior_tree_file);

    bool Start(const std::string& bt_xml_file);
    bool Cancel();
    bool Pause();
    bool Resume();
    bool IsRunning() const;

    void SetCompletionHook(std::function<void(RunStatus)> hook) {
        completion_hook_ = std::move(hook);
    }

protected:
    virtual bool OnGoalReceived() = 0;
    virtual void OnLoop() {}
    virtual void OnPreempt() {}
    virtual void OnCompletion(RunStatus status);

    std::shared_ptr<BehaviorTreeEngine> engine_;
    std::shared_ptr<BehaviorTreeContext> ctx_;
    std::unique_ptr<BehaviorTreeActionServer> action_server_;
    std::string default_bt_xml_;
    std::function<void(RunStatus)> completion_hook_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
