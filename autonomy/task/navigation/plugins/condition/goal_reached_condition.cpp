/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>

namespace autonomy::task::plugins::navigation {

class GoalReachedCondition : public BtCondition
{
public:
    GoalReachedCondition(const std::string& name, const BT::NodeConfig& config)
        : BtCondition(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<automsgs::msgs::geometry_msgs::PoseStamped>(
                "goal", "navigation goal pose"),
            BT::InputPort<std::string>("robot_base_frame", "base_link",
                                       "robot frame (unused, TF via client)"),
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
