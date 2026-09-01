/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/teleop/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::teleop {

/**
 * @class teleop::TeleopWatchdogOkCondition
 * @brief BT condition CommandValid: true while teleop watchdog has not expired
 */
class TeleopWatchdogOkCondition : public BtCondition
{
public:
    /**
     * @brief Construct CommandValid BT condition node
     */
    TeleopWatchdogOkCondition(const std::string& name,
                              const BT::NodeConfig& config)
        : BtCondition(name, config) {}

protected:
    /**
     * @brief Evaluate CommandValid BT condition (watchdog)
     */
    BT::NodeStatus OnEvaluate() override
    {
        auto client = ResolveClient(*this);
        if (!client) {
            return BT::NodeStatus::FAILURE;
        }
        return client->IsWatchdogOk() ? BT::NodeStatus::SUCCESS
                                      : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::teleop

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::teleop::TeleopWatchdogOkCondition>(
        "CommandValid");
}
