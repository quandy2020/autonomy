/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/common/logging.hpp"
#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

class ComputePathAction : public BtSyncAction
{
public:
    ComputePathAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<automsgs::msgs::geometry_msgs::PoseStamped>("goal"),
            BT::BidirectionalPort<automsgs::msgs::nav_msgs::Path>("path"),
            BT::InputPort<std::string>("planner_id"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        automsgs::msgs::geometry_msgs::PoseStamped goal;
        std::string planner_id;
        if (!getInput("goal", goal)) {
            SetErrorPorts(*this, 1, "ComputePath: missing goal");
            return BT::NodeStatus::FAILURE;
        }
        getInput("planner_id", planner_id);

        automsgs::msgs::nav_msgs::Path path;
        int error_code = 0;
        std::string error_message;
        if (!client->ComputePathToPose(goal, planner_id, path, &error_code,
                                       &error_message)) {
            // Keep following the last good path instead of aborting FollowPath
            // / thrashing RecoveryNode on transient costmap collisions.
            automsgs::msgs::nav_msgs::Path existing;
            if (getInput("path", existing) && !existing.poses().empty()) {
                AWARN_EVERY(20)
                    << "ComputePath failed (" << error_message
                    << "); keeping previous path (" << existing.poses_size()
                    << " poses)";
                ClearErrorPorts(*this);
                return BT::NodeStatus::SUCCESS;
            }
            SetErrorPorts(*this, error_code, error_message);
            return BT::NodeStatus::FAILURE;
        }
        try {
            setOutput("path", path);
        } catch (const std::exception& ex) {
            AERROR << "ComputePathToPose: setOutput(path) failed: " << ex.what()
                   << " poses=" << path.poses_size();
            SetErrorPorts(*this, 1, ex.what());
            return BT::NodeStatus::FAILURE;
        }
        ClearErrorPorts(*this);
        AINFO_EVERY(10) << "ComputePathToPose OK (" << path.poses_size()
                        << " poses)";
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::ComputePathAction>("PlanPath");
    factory.registerNodeType<
        autonomy::task::plugins::navigation::ComputePathAction>(
        "ComputePathToPose");
}
