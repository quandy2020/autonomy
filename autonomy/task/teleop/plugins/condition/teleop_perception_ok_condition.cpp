/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/teleop/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::teleop {

class TeleopPerceptionOkCondition : public BtCondition
{
public:
    TeleopPerceptionOkCondition(const std::string& name,
                                const BT::NodeConfig& config)
        : BtCondition(name, config) {}

protected:
    BT::NodeStatus OnEvaluate() override
    {
        auto client = ResolveClient(*this);
        if (!client) {
            return BT::NodeStatus::FAILURE;
        }
        return client->IsPerceptionOk() ? BT::NodeStatus::SUCCESS
                                        : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::teleop

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::teleop::TeleopPerceptionOkCondition>(
        "PerceptionValid");
}
