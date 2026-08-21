/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

class PathValidCondition : public BtCondition
{
public:
    PathValidCondition(const std::string& name, const BT::NodeConfig& config)
        : BtCondition(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<automsgs::msgs::nav_msgs::Path>("path"),
            BT::InputPort<int>("max_cost", 253, "costmap threshold"),
            BT::InputPort<bool>("consider_unknown_as_obstacle", false, ""),
        };
    }

protected:
    BT::NodeStatus OnEvaluate() override
    {
        automsgs::msgs::nav_msgs::Path path;
        int max_cost = 253;
        bool consider_unknown = false;
        if (!getInput("path", path)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("max_cost", max_cost);
        getInput("consider_unknown_as_obstacle", consider_unknown);

        return ResolveClient(*this)
                       ->IsPathValid(path, static_cast<uint8_t>(max_cost),
                                     consider_unknown)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::navigation::PathValidCondition>(
        "NavPathValid");
}
