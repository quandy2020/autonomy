/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/plugins/phase_action.hpp"

namespace autonomy::task::plugins::exploration {
class SelectObservationPoseAction final : public PhaseAction {
public:
    SelectObservationPoseAction(const std::string& name, const BT::NodeConfig& config)
        : PhaseAction(name, config, task::exploration::ExplorationPhase::kSelect) {}
    static BT::PortsList providedPorts() {
        auto ports = PhaseAction::providedPorts();
        ports.insert(BT::OutputPort<task::exploration::ObservationGoal>("observation_goal"));
        return ports;
    }
protected:
    bool PublishSuccess(const task::exploration::ExplorationSnapshot& snapshot) override {
        if (!snapshot.selection) return false;
        setOutput("observation_goal", *snapshot.selection);
        return true;
    }
};
}  // namespace autonomy::task::plugins::exploration

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::task::plugins::exploration::SelectObservationPoseAction>(
        "SelectObservationPose");
}
