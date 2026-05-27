/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "autonomy/tasks/behavior_tree/bt_action_server.hpp"
#include "autonomy/tasks/behavior_tree/navigator/navigator_interface.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief Base navigator: owns BtActionServer and default BT path.
 */
class BehaviorTreeNavigator : public NavigatorInterface
{
public:
    bool Configure(std::shared_ptr<BehaviorTreeEngine> engine,
                   std::shared_ptr<BehaviorTreeContext> ctx,
                   const std::string& default_behavior_tree_file) override;

    bool Start(const std::string& bt_xml_file) override;
    bool Cancel() override;
    bool Pause() override;
    bool Resume() override;
    bool IsRunning() const override;

    void SetCompletionHook(std::function<void(BtStatus)> hook) {
        completion_hook_ = std::move(hook);
    }

protected:
    virtual bool OnGoalReceived() = 0;
    virtual void OnLoop() {}
    virtual void OnPreempt() {}
    virtual void OnCompletion(BtStatus status);

    std::shared_ptr<BehaviorTreeEngine> engine_;
    std::shared_ptr<BehaviorTreeContext> ctx_;
    std::unique_ptr<BtActionServer> bt_server_;
    std::string default_bt_xml_;
    std::function<void(BtStatus)> completion_hook_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
