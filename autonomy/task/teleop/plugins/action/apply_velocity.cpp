/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/teleop/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::teleop {

/**
 * @class teleop::ApplyTeleopVelocityAction
 * @brief BT action TrackCommand: publish /cmd_vel via TeleopClient
 */
class ApplyTeleopVelocityAction : public BtSyncAction
{
public:
    /**
     * @brief Construct TrackCommand BT action node
     */
    ApplyTeleopVelocityAction(const std::string& name,
                              const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    /**
     * @brief BT action ports (error outputs only)
     */
    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    /**
     * @brief Publish /cmd_vel via TeleopClient
     */
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        if (!client) {
            setOutput("error_code_id", 1);
            setOutput("error_msg", std::string("TrackCommand: missing teleop client"));
            return BT::NodeStatus::FAILURE;
        }
        if (!client->PublishVelocity()) {
            setOutput("error_code_id", 2);
            setOutput("error_msg", std::string("TrackCommand: publish failed"));
            return BT::NodeStatus::FAILURE;
        }

        setOutput("error_code_id", 0);
        setOutput("error_msg", std::string{});
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::teleop

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::teleop::ApplyTeleopVelocityAction>(
        "TrackCommand");
}
