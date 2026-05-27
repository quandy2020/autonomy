/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include "autonomy/control/controller_server.hpp"
#include "autonomy/tasks/behavior_tree/action_node.hpp"
#include "autonomy/tasks/behavior_tree/node_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/** BT action base for ControllerServer open-loop recovery motions. */
class RecoveryActionNode : public ActionNode
{
public:
    using ActionNode::ActionNode;

protected:
    bool BeginRecovery(
        const control::ControllerServer::RecoveryMotionCommand& cmd) {
        auto ctx = Context();
        if (!ctx || !ctx->controller) {
            return false;
        }
        started_ = ctx->controller->BeginRecoveryMotion(cmd);
        return started_;
    }

    BT::NodeStatus TickRecovery() {
        auto ctx = Context();
        if (!ctx || !started_) {
            return BT::NodeStatus::FAILURE;
        }
        return MapRecoveryTick(
            ctx->controller->TickRecoveryMotion(ctx->CancelChecker()));
    }

    void EndRecovery() {
        if (auto ctx = Context()) {
            if (ctx->controller) {
                ctx->controller->EndRecoveryMotion();
            }
        }
        started_ = false;
    }

    void onHalted() override { EndRecovery(); }

    bool started_{false};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
