/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::exploration {
class SensorReadyCondition final : public BtCondition {
public:
    using BtCondition::BtCondition;
    static BT::PortsList providedPorts() { return ContextPorts(); }
protected:
    BT::NodeStatus OnEvaluate() override {
        try {
            const auto context = ResolveContext(*this);
            return context && context->session && context->session->Snapshot().motion_ready
                ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        } catch (...) { return BT::NodeStatus::FAILURE; }
    }
};
}  // namespace autonomy::task::plugins::exploration

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::task::plugins::exploration::SensorReadyCondition>(
        "SensorReady");
}
