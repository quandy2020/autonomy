/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::exploration {
class UpdateExplorationAction final : public BtSyncAction {
public:
    using BtSyncAction::BtSyncAction;
    static BT::PortsList providedPorts() { return ContextPorts(); }
protected:
    BT::NodeStatus OnExecute() override {
        try {
            const auto context = ResolveContext(*this);
            if (!context || !context->session || !context->clock ||
                context->session->driver() != task::exploration::ExplorationDriver::kBehaviorTree)
                return BT::NodeStatus::FAILURE;
            PublishSnapshot(*this, context->session->Tick(context->clock()));
            return BT::NodeStatus::SUCCESS;
        } catch (...) {
            return BT::NodeStatus::FAILURE;
        }
    }
};
}  // namespace autonomy::task::plugins::exploration

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::task::plugins::exploration::UpdateExplorationAction>(
        "UpdateExploration");
}
