/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/behavior_tree_action_node.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class DriveOnHeadingAction : public BehaviorTreeActionNode
{
public:
    DriveOnHeadingAction(const std::string& name,
                         const BT::NodeConfiguration& conf)
        : BehaviorTreeActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("dist_to_travel", 0.5, "Distance m"),
            BT::InputPort<double>("speed", 0.2, "Speed m/s"),
            BT::InputPort<double>("time_allowance", 10.0, "Seconds"),
            BT::InputPort<bool>("disable_collision_checks", false, "No collide"),
        };
    }

protected:
    void onStart() override {
        auto ctx = GetContext(config());
        if (!ctx || !ctx->controller) {
            return;
        }
        double dist = 0.5;
        double speed = 0.2;
        double time_allowance = 10.0;
        bool disable_checks = false;
        getInput("dist_to_travel", dist);
        getInput("speed", speed);
        getInput("time_allowance", time_allowance);
        getInput("disable_collision_checks", disable_checks);
        control::ControllerServer::RecoveryMotionCommand cmd;
        cmd.type =
            control::ControllerServer::RecoveryMotionType::DriveOnHeading;
        cmd.distance = dist;
        cmd.speed = speed;
        cmd.time_allowance_sec = time_allowance;
        cmd.disable_collision_checks = disable_checks;
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
    factory.registerNodeType<autonomy::tasks::behavior_tree::DriveOnHeadingAction>(
        "DriveOnHeading");
}
