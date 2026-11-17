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
                                       "robot frame for current-pose lookup"),
            BT::InputPort<double>("goal_reached_tol", 0.25, "m"),
            BT::InputPort<double>("goal_reached_yaw_tol", 0.10, "rad"),
            BT::InputPort<std::string>("goal_mode", "POINT_GOAL", "execution goal mode"),
            BT::InputPort<bool>("allow_alternative_completion", false,
                                "task explicitly accepts a region/approximate goal"),
            BT::InputPort<double>("transform_tolerance", 0.1,
                                  "TF lookup tolerance (s)"),
        };
    }

protected:
    BT::NodeStatus OnEvaluate() override
    {
        int epoch_before = 0, epoch_after = 0;
        if (config().blackboard) config().blackboard->get("goal_epoch", epoch_before);
        double tolerance = 0.25;
        double yaw_tolerance = 0.10, max_pose_age = 0.1;
        std::string robot_frame = "base_link", mode;
        bool allow_alternative = false;
        automsgs::msgs::geometry_msgs::PoseStamped goal;
        if (!getInput("goal", goal)) return BT::NodeStatus::FAILURE;
        getInput("goal_reached_tol", tolerance);
        getInput("goal_reached_yaw_tol", yaw_tolerance);
        getInput("transform_tolerance", max_pose_age);
        getInput("robot_base_frame", robot_frame);
        getInput("goal_mode", mode);
        getInput("allow_alternative_completion", allow_alternative);
        const bool exact = mode == "POINT_GOAL" || mode == "EXACT";
        const bool alternative = mode == "REGION_FALLBACK" || mode == "APPROXIMATE";
        if (!exact && !(alternative && allow_alternative))
            return BT::NodeStatus::FAILURE;
        const bool reached = ResolveClient(*this)->IsGoalReached(
            goal, tolerance, yaw_tolerance, robot_frame, max_pose_age);
        if (config().blackboard) config().blackboard->get("goal_epoch", epoch_after);
        return reached && epoch_before == epoch_after
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::GoalReachedCondition>("GoalReached");
}
