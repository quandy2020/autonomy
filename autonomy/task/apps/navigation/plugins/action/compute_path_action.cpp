/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

class ComputePathAction : public BtSyncAction
{
public:
    ComputePathAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("goal"),
            BT::OutputPort<commsgs::planning_msgs::Path>("path"),
            BT::InputPort<std::string>("planner_id"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        commsgs::geometry_msgs::PoseStamped goal;
        std::string planner_id;
        if (!getInput("goal", goal)) {
            SetErrorPorts(*this, 1, "ComputePath: missing goal");
            return BT::NodeStatus::FAILURE;
        }
        getInput("planner_id", planner_id);

        commsgs::planning_msgs::Path path;
        int error_code = 0;
        std::string error_msg;
        if (!client->ComputePathToPose(goal, planner_id, path, &error_code,
                                       &error_msg)) {
            SetErrorPorts(*this, error_code, error_msg);
            return BT::NodeStatus::FAILURE;
        }

        setOutput("path", path);
        ClearErrorPorts(*this);
        return BT::NodeStatus::SUCCESS;
    }
};

class ComputePathThroughPosesAction : public BtSyncAction
{
public:
    ComputePathThroughPosesAction(const std::string& name,
                                  const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "goals"),
            BT::OutputPort<commsgs::planning_msgs::Path>("path"),
            BT::InputPort<std::string>("planner_id"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        std::vector<commsgs::geometry_msgs::PoseStamped> goals;
        std::string planner_id;
        if (!getInput("goals", goals) || goals.empty()) {
            SetErrorPorts(*this, 1, "ComputePathThroughPoses: missing goals");
            return BT::NodeStatus::FAILURE;
        }
        getInput("planner_id", planner_id);

        commsgs::planning_msgs::Path path;
        int error_code = 0;
        std::string error_msg;
        if (!client->ComputePathThroughPoses(goals, planner_id, path, &error_code,
                                             &error_msg)) {
            SetErrorPorts(*this, error_code, error_msg);
            return BT::NodeStatus::FAILURE;
        }

        setOutput("path", path);
        ClearErrorPorts(*this);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::ComputePathAction>("NavComputePath");
    factory.registerNodeType<
        autonomy::task::plugins::navigation::ComputePathThroughPosesAction>(
        "NavComputePathThroughPoses");
}
