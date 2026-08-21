/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

class ComputePathThroughPosesAction : public BtSyncAction
{
public:
    ComputePathThroughPosesAction(const std::string& name,
                                  const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<automsgs::msgs::geometry_msgs::PoseStamped>>(
                "goals"),
            BT::OutputPort<automsgs::msgs::nav_msgs::Path>("path"),
            BT::InputPort<std::string>("planner_id"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        std::vector<automsgs::msgs::geometry_msgs::PoseStamped> goals;
        std::string planner_id;
        if (!getInput("goals", goals) || goals.empty()) {
            SetErrorPorts(*this, 1, "ComputePathThroughPoses: missing goals");
            return BT::NodeStatus::FAILURE;
        }
        getInput("planner_id", planner_id);

        automsgs::msgs::nav_msgs::Path path;
        int error_code = 0;
        std::string error_message;
        if (!client->ComputePathThroughPoses(goals, planner_id, path, &error_code,
                                             &error_message)) {
            SetErrorPorts(*this, error_code, error_message);
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
        autonomy::task::plugins::navigation::ComputePathThroughPosesAction>(
        "PlanPoses");
}
