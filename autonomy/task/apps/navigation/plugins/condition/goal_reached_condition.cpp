/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

class GoalReachedCondition : public BtCondition
{
public:
    GoalReachedCondition(const std::string& name, const BT::NodeConfig& config)
        : BtCondition(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("goal_reached_tol", 0.25, "m"),
        };
    }

protected:
    BT::NodeStatus OnEvaluate() override
    {
        double tolerance = 0.25;
        getInput("goal_reached_tol", tolerance);
        return ResolveClient(*this)->IsGoalReached(tolerance)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::GoalReachedCondition>("NavGoalReached");
}
