/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <memory>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/tasks/behavior_tree/action/navigation_action_traits.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class BtNavigator;

/** Autolink action server for navigate_to_pose (wraps BtNavigator). */
class NavigateToPoseActionServer
{
public:
    explicit NavigateToPoseActionServer(BtNavigator* navigator);

    bool Activate(std::shared_ptr<autolink::Node> node);
    void Deactivate();

    bool IsActive() const;

private:
    void ExecuteCallback();

    BtNavigator* navigator_{nullptr};
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<
        autolink::action::SimpleActionServer<NavigateToPoseActionTraits>>
        server_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
