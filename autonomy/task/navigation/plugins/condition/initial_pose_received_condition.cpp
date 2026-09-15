/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"

namespace autonomy::task::plugins::navigation {

class InitialPoseReceivedCondition : public BtCondition
{
public:
    InitialPoseReceivedCondition(const std::string& name,
                                 const BT::NodeConfig& config)
        : BtCondition(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<bool>("initial_pose_received", true,
                                    "blackboard / port pose-init flag")};
    }

protected:
    BT::NodeStatus OnEvaluate() override
    {
        bool received = true;
        if (!getInput("initial_pose_received", received)) {
            if (config().blackboard) {
                (void)config().blackboard->get("initial_pose_received",
                                               received);
            }
        }
        return received ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::InitialPoseReceivedCondition>(
        "InitialPoseReceived");
}
