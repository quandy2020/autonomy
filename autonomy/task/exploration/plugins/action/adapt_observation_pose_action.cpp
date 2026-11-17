/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/plugins/phase_action.hpp"

namespace autonomy::task::plugins::exploration {
class AdaptObservationPoseAction final : public PhaseAction {
public:
    AdaptObservationPoseAction(const std::string& name, const BT::NodeConfig& config)
        : PhaseAction(name, config, task::exploration::ExplorationPhase::kAdapt) {}
    static BT::PortsList providedPorts() {
        auto ports = PhaseAction::providedPorts();
        ports.insert(BT::OutputPort<task::exploration::AdaptedObservationGoal>("adapted_goal"));
        return ports;
    }
protected:
    bool PublishSuccess(const task::exploration::ExplorationSnapshot& snapshot) override {
        if (!snapshot.goal) return false;
        setOutput("adapted_goal", *snapshot.goal);
        return true;
    }
};
}  // namespace autonomy::task::plugins::exploration

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::task::plugins::exploration::AdaptObservationPoseAction>(
        "AdaptObservationPose");
}
