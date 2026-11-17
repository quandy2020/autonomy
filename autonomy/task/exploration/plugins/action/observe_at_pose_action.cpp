/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/plugins/phase_action.hpp"

namespace autonomy::task::plugins::exploration {
class ObserveAtPoseAction final : public PhaseAction {
public:
    ObserveAtPoseAction(const std::string& name, const BT::NodeConfig& config)
        : PhaseAction(name, config, task::exploration::ExplorationPhase::kObserve) {}
};
}  // namespace autonomy::task::plugins::exploration

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::task::plugins::exploration::ObserveAtPoseAction>(
        "ObserveAtPose");
}
