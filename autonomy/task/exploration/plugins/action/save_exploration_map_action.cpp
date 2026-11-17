/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/plugins/phase_action.hpp"

namespace autonomy::task::plugins::exploration {
class SaveExplorationMapAction final : public PhaseAction {
public:
    SaveExplorationMapAction(const std::string& name, const BT::NodeConfig& config)
        : PhaseAction(name, config, task::exploration::ExplorationPhase::kSave) {}
};
}  // namespace autonomy::task::plugins::exploration

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::task::plugins::exploration::SaveExplorationMapAction>(
        "SaveExplorationMap");
}
