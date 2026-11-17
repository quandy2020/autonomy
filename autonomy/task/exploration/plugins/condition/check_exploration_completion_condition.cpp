/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::exploration {
class CheckExplorationCompletionCondition final : public BtCondition {
public:
    using BtCondition::BtCondition;
    static BT::PortsList providedPorts() { return ContextPorts(); }
protected:
    BT::NodeStatus OnEvaluate() override {
        try {
            const auto context = ResolveContext(*this);
            if (!context || !context->session) return BT::NodeStatus::FAILURE;
            const auto snapshot = context->session->Snapshot();
            return snapshot.completion_confirmed && snapshot.result.exploration ==
                task::exploration::ExplorationOutcome::kCompleted
                ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        } catch (...) { return BT::NodeStatus::FAILURE; }
    }
};
}  // namespace autonomy::task::plugins::exploration

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::task::plugins::exploration::CheckExplorationCompletionCondition>(
        "CheckExplorationCompletion");
}
