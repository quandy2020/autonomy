/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class RemoveInCollisionGoalsAction : public BT::SyncActionNode
{
public:
    RemoveInCollisionGoalsAction(const std::string& name,
                                 const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("service_name", "", "Costmap id"),
            BT::InputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "input_goals"),
            BT::InputPort<int>("cost_threshold", 253, "Max cost"),
            BT::InputPort<bool>("consider_unknown_as_obstacle", false,
                                "Treat unknown as obstacle"),
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
        int cost_threshold = 253;
        bool consider_unknown = false;
        std::string service_name;
        getInput("cost_threshold", cost_threshold);
        getInput("consider_unknown_as_obstacle", consider_unknown);
        getInput("service_name", service_name);

        auto ctx = GetContext(config());
        auto wrapper = ResolveCostmap(ctx, service_name);
        if (!wrapper) {
            return BT::NodeStatus::FAILURE;
        }
        auto* map = wrapper->getCostmap();
        if (!map) {
            return BT::NodeStatus::FAILURE;
        }

        std::vector<commsgs::geometry_msgs::PoseStamped> valid;
        valid.reserve(goals.size());
        for (const auto& goal : goals) {
            unsigned int mx = 0;
            unsigned int my = 0;
            if (!map->worldToMap(goal.pose.position.x, goal.pose.position.y, mx,
                                 my)) {
                continue;
            }
            const unsigned char cost = map->getCost(mx, my);
            if (cost == map::costmap_2d::NO_INFORMATION) {
                if (!consider_unknown) {
                    valid.push_back(goal);
                }
                continue;
            }
            if (static_cast<int>(cost) < cost_threshold) {
                valid.push_back(goal);
            }
        }
        setOutput("output_goals", valid);
        if (config().blackboard) {
            config().blackboard->set(kBlackboardGoalsKey, valid);
            if (!valid.empty()) {
                config().blackboard->set(kBlackboardGoalKey, valid.back());
            }
        }
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(RemoveInCollisionGoalsAction,
                            "RemoveInCollisionGoals")
