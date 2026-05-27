/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/behavior_tree_action_node.hpp"
#include "autonomy/tasks/behavior_tree/error_codes.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class FollowPathAction : public BehaviorTreeActionNode
{
public:
    FollowPathAction(const std::string& name, const BT::NodeConfiguration& conf)
        : BehaviorTreeActionNode(name, conf), started_(false) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::planning_msgs::Path>("path"),
            BT::InputPort<std::string>("controller_id", "", "Controller"),
            BT::OutputPort<uint16_t>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    void onStart() override {
        started_ = false;
        auto ctx = GetContext(config());
        if (!ctx || !ctx->controller) {
            return;
        }
        commsgs::planning_msgs::Path path;
        if (!getInput("path", path)) {
            getInput(kBlackboardPathKey, path);
        }
        std::string controller_id;
        getInput("controller_id", controller_id);
        if (controller_id.empty()) {
            config().blackboard->get("selected_controller", controller_id);
        }
        if (controller_id.empty()) {
            controller_id = ctx->options.default_controller_id();
        }
        std::string goal_checker = ctx->options.default_goal_checker_id();
        if (goal_checker.empty()) {
            goal_checker = "goal_checker";
        }
        std::string progress_checker = "progress_checker";
        started_ = ctx->controller->BeginFollowPath(path, controller_id,
                                                      goal_checker,
                                                      progress_checker);
    }

    BT::NodeStatus onRunning() override {
        auto ctx = GetContext(config());
        if (!ctx || !ctx->controller || !started_) {
            return BT::NodeStatus::FAILURE;
        }
        const auto result =
            ctx->controller->TickFollowPath(ctx->CancelChecker());
        switch (result) {
            case control::ControllerServer::FollowPathTickResult::Running:
                return BT::NodeStatus::RUNNING;
            case control::ControllerServer::FollowPathTickResult::Succeeded:
                setOutput("error_code_id", BtErrorCode::NONE);
                setOutput("error_msg", std::string());
                return BT::NodeStatus::SUCCESS;
            case control::ControllerServer::FollowPathTickResult::Cancelled:
                setOutput("error_code_id", BtErrorCode::CANCELED);
                return BT::NodeStatus::SUCCESS;
            case control::ControllerServer::FollowPathTickResult::Failed:
            default:
                setOutput("error_code_id", BtErrorCode::CONTROLLER_FAILED);
                setOutput("error_msg", "FollowPath failed.");
                return BT::NodeStatus::FAILURE;
        }
    }

    void onHalted() override {
        if (auto ctx = GetContext(config())) {
            if (ctx->controller) {
                ctx->controller->EndFollowPath();
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
    factory.registerNodeType<autonomy::tasks::behavior_tree::FollowPathAction>(
        "FollowPath");
}
