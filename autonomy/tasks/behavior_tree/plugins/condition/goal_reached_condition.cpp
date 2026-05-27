/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <cmath>

#include "autonomy/tasks/behavior_tree/bt_plugin_common.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class GoalReachedCondition : public BT::ConditionNode
{
public:
    GoalReachedCondition(const std::string& name,
                         const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("goal"),
            BT::InputPort<double>("goal_reached_tol", 0.25, "Tolerance m"),
            BT::InputPort<double>("transform_tolerance", 0.1, "TF tol"),
        };
    }

    BT::NodeStatus tick() override {
        auto ctx = GetContext(config());
        if (!ctx) {
            return BT::NodeStatus::FAILURE;
        }
        commsgs::geometry_msgs::PoseStamped goal;
        if (!getInput("goal", goal)) {
            getInput(kBlackboardGoalKey, goal);
        }
        double tol = 0.25;
        getInput("goal_reached_tol", tol);
        if (tol <= 0.0) {
            config().blackboard->get(kBlackboardGoalReachedTolKey, tol);
        }
        commsgs::geometry_msgs::PoseStamped current;
        if (!utils::getGlobalRobotPose(
                current, ctx->tf_buffer, ctx->controller->GetOdomSmoother(),
                ctx->options.global_frame(), ctx->options.robot_base_frame())) {
            return BT::NodeStatus::FAILURE;
        }
        const double dx = current.pose.position.x - goal.pose.position.x;
        const double dy = current.pose.position.y - goal.pose.position.y;
        const double dist = std::hypot(dx, dy);
        return dist <= tol ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::GoalReachedCondition>(
        "GoalReached");
}
