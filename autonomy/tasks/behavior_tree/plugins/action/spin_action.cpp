/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/behavior_tree_action_node.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class SpinAction : public BehaviorTreeActionNode
{
public:
    SpinAction(const std::string& name, const BT::NodeConfiguration& conf)
        : BehaviorTreeActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("spin_dist", 1.57, "Yaw radians"),
            BT::InputPort<double>("time_allowance", 10.0, "Seconds"),
            BT::InputPort<bool>("is_recovery", false, "Recovery flag"),
        };
    }

protected:
    void onStart() override {
        auto ctx = GetContext(config());
        if (!ctx || !ctx->controller) {
            return;
        }
        double spin_dist = 1.57;
        double time_allowance = 10.0;
        bool is_recovery = false;
        getInput("spin_dist", spin_dist);
        getInput("time_allowance", time_allowance);
        getInput("is_recovery", is_recovery);
        if (is_recovery) {
            IncrementRecoveryCount(config());
        }
        control::ControllerServer::RecoveryMotionCommand cmd;
        cmd.type = control::ControllerServer::RecoveryMotionType::Spin;
        cmd.distance = spin_dist;
        cmd.speed = 0.5;
        cmd.time_allowance_sec = time_allowance;
        started_ = ctx->controller->BeginRecoveryMotion(cmd);
    }

    BT::NodeStatus onRunning() override {
        auto ctx = GetContext(config());
        if (!ctx || !started_) {
            return BT::NodeStatus::FAILURE;
        }
        const auto r =
            ctx->controller->TickRecoveryMotion(ctx->CancelChecker());
        switch (r) {
            case control::ControllerServer::RecoveryTickResult::Running:
                return BT::NodeStatus::RUNNING;
            case control::ControllerServer::RecoveryTickResult::Succeeded:
                return BT::NodeStatus::SUCCESS;
            case control::ControllerServer::RecoveryTickResult::Cancelled:
                return BT::NodeStatus::SUCCESS;
            default:
                return BT::NodeStatus::FAILURE;
        }
    }

    void onHalted() override {
        if (auto ctx = GetContext(config())) {
            if (ctx->controller) {
                ctx->controller->EndRecoveryMotion();
            }
        }
        started_ = false;
    }

private:
    bool started_{false};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::SpinAction>("Spin");
}
