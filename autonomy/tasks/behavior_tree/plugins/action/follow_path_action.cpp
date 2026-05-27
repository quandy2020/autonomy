/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/action_node.hpp"
#include "autonomy/tasks/behavior_tree/node_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class FollowPathAction : public ActionNode
{
public:
    FollowPathAction(const std::string& name, const BT::NodeConfiguration& conf)
        : ActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = {
            BT::InputPort<commsgs::planning_msgs::Path>("path"),
            BT::InputPort<std::string>("controller_id", "", "Controller"),
        };
        return AppendErrorOutcomePorts(ports);
    }

protected:
    void onStart() override {
        started_ = false;
        auto ctx = Context();
        if (!ctx || !ctx->controller) {
            return;
        }
        commsgs::planning_msgs::Path path;
        if (!GetInputOrBB(*this, "path", kBlackboardPathKey, path)) {
            return;
        }
        const std::string controller_id = ResolveControllerId(*this, *ctx);
        std::string goal_checker = ctx->options.default_goal_checker_id();
        if (goal_checker.empty()) {
            goal_checker = "goal_checker";
        }
        started_ = ctx->controller->BeginFollowPath(path, controller_id,
                                                    goal_checker,
                                                    "progress_checker");
    }

    BT::NodeStatus onRunning() override {
        auto ctx = Context();
        if (!ctx || !ctx->controller || !started_) {
            return BT::NodeStatus::FAILURE;
        }
        return MapFollowPathTick(
            *this, ctx->controller->TickFollowPath(ctx->CancelChecker()));
    }

    void onHalted() override {
        if (auto ctx = Context()) {
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

REGISTER_BEHAVIOR_TREE_NODE(FollowPathAction, "FollowPath")
