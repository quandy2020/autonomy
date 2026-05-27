/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <memory>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/tasks/behavior_tree/servers/traits.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class BehaviorTreeNavigationEngine;

/** Autolink action server for navigate_through_poses (wraps BehaviorTreeNavigationEngine). */
class NavigateThroughPosesServer
{
public:
    explicit NavigateThroughPosesServer(BehaviorTreeNavigationEngine* engine);

    bool Activate(std::shared_ptr<autolink::Node> node);
    void Deactivate();

    bool IsActive() const;

private:
    void ExecuteCallback();

    BehaviorTreeNavigationEngine* engine_{nullptr};
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<
        autolink::action::SimpleActionServer<NavigateThroughPosesActionTraits>>
        server_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
