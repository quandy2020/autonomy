/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <cmath>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class RemovePassedGoalsAction : public BT::SyncActionNode
{
public:
    RemovePassedGoalsAction(const std::string& name,
                            const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "input_goals"),
            BT::InputPort<double>("radius", 0.5, "Passed radius"),
            BT::OutputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "output_goals"),
        };
    }

    BT::NodeStatus tick() override {
        std::vector<commsgs::geometry_msgs::PoseStamped> goals;
        if (!getInput("input_goals", goals)) {
            GetInputOrBlackboard(*this, config(), "goals", kBlackboardGoalsKey,
                                 goals);
        }
        double radius = 0.5;
        getInput("radius", radius);
        const double radius_sq = radius * radius;

        auto ctx = GetContext(config());
        commsgs::geometry_msgs::PoseStamped current;
        if (!ctx || !GetGlobalStartPose(*ctx, current)) {
            return BT::NodeStatus::FAILURE;
        }

        std::vector<commsgs::geometry_msgs::PoseStamped> remaining;
        remaining.reserve(goals.size());
        for (const auto& goal : goals) {
            const double dx = goal.pose.position.x - current.pose.position.x;
            const double dy = goal.pose.position.y - current.pose.position.y;
            if ((dx * dx + dy * dy) > radius_sq) {
                remaining.push_back(goal);
            }
        }
        setOutput("output_goals", remaining);
        if (config().blackboard) {
            config().blackboard->set(kBlackboardGoalsKey, remaining);
            if (!remaining.empty()) {
                config().blackboard->set(kBlackboardGoalKey, remaining.back());
            }
        }
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(RemovePassedGoalsAction, "RemovePassedGoals")
