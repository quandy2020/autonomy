/* Copyright 2026 The Openbot Authors. */
/** @file startup_observation_action.cpp
 * @brief BT gate for the deployment-confirmed, stationary startup observation phase.
 */
#include "autonomy/task/exploration/plugins/phase_action.hpp"

namespace autonomy::task::plugins::exploration {
/** @brief Keeps normal exploration selection behind a bounded native yaw sweep. */
class StartupObservationAction final : public PhaseAction {
public:
    /** @brief Bind the action to the owner-session startup phase. */
    StartupObservationAction(const std::string& name, const BT::NodeConfig& config)
        : PhaseAction(name, config,
                      task::exploration::ExplorationPhase::kStartupObservation) {}
    /** @brief Expose the common exploration context only; no fabricated result is accepted. */
    static BT::PortsList providedPorts() { return PhaseAction::providedPorts(); }
};
}  // namespace autonomy::task::plugins::exploration

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::task::plugins::exploration::StartupObservationAction>(
        "StartupObservation");
}
