/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/common/logging.hpp"

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
            BT::InputPort<double>("max_smoothing_duration", 1.0,
                                  "smoother wall-time budget (s)"),
            BT::InputPort<bool>("check_for_collisions", false,
                                "reject smoothed path on collision"),
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
        double max_smoothing_duration = 1.0;
        bool check_for_collisions = false;
        if (!getInput("unsmoothed_path", unsmoothed)) {
            SetErrorPorts(*this, 1, "SmoothPath: missing unsmoothed_path");
            return BT::NodeStatus::FAILURE;
        }
        getInput("smoother_id", smoother_id);
        getInput("max_smoothing_duration", max_smoothing_duration);
        getInput("check_for_collisions", check_for_collisions);

        automsgs::msgs::nav_msgs::Path smoothed;
        int error_code = 0;
        std::string error_message;
        if (!client->SmoothPath(unsmoothed, smoother_id, max_smoothing_duration,
                                check_for_collisions, smoothed, &error_code,
                                &error_message)) {
            // Optional stage: keep navigating on the planned path if smoothing
            // fails (timeout / abort / empty). Avoids BT recovery thrashing.
            AWARN << "SmoothPath failed (" << error_message
                  << "); using unsmoothed path (" << unsmoothed.poses_size()
                  << " poses)";
            setOutput("smoothed_path", unsmoothed);
            setOutput("smoothing_duration", 0.0);
            setOutput("was_completed", false);
            SetErrorPorts(*this, error_code, error_message);
            return BT::NodeStatus::SUCCESS;
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
        "SmoothPath");
}
