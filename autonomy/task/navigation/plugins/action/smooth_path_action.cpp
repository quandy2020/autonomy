/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

class SmoothPathAction : public BtSyncAction
{
public:
    SmoothPathAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<automsgs::msgs::nav_msgs::Path>("unsmoothed_path"),
            BT::OutputPort<automsgs::msgs::nav_msgs::Path>("smoothed_path"),
            BT::InputPort<std::string>("smoother_id", "simple_smoother",
                                       "plugin id"),
            BT::OutputPort<double>("smoothing_duration"),
            BT::OutputPort<bool>("was_completed"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        automsgs::msgs::nav_msgs::Path unsmoothed;
        std::string smoother_id;
        if (!getInput("unsmoothed_path", unsmoothed)) {
            SetErrorPorts(*this, 1, "SmoothPath: missing unsmoothed_path");
            return BT::NodeStatus::FAILURE;
        }
        getInput("smoother_id", smoother_id);

        automsgs::msgs::nav_msgs::Path smoothed;
        int error_code = 0;
        std::string error_message;
        if (!client->SmoothPath(unsmoothed, smoother_id, smoothed, &error_code,
                                &error_message)) {
            SetErrorPorts(*this, error_code, error_message);
            return BT::NodeStatus::FAILURE;
        }

        setOutput("smoothed_path", smoothed);
        setOutput("smoothing_duration", 0.0);
        setOutput("was_completed", true);
        ClearErrorPorts(*this);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::navigation::SmoothPathAction>(
        "NavSmoothPath");
}
