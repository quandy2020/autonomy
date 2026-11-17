/* Copyright 2026 The Openbot Authors. */
#pragma once

#include <optional>
#include "autonomy/task/exploration/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::exploration {

// A native stateful action owns exactly one session ticket. Repeated ticks
// never submit new work; halting cancels only that ticket, not the task.
class PhaseAction : public BtStatefulAction {
public:
    PhaseAction(const std::string& name, const BT::NodeConfig& config,
                task::exploration::ExplorationPhase phase);
    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus OnFirstTick() override;
    BT::NodeStatus OnExecute() override;
    void OnHalted() override;
    bool LatchAfterFirstTick() const override;
    virtual bool PublishSuccess(const task::exploration::ExplorationSnapshot& snapshot);

private:
    BT::NodeStatus Poll();
    BT::NodeStatus Failure(const std::string& reason);
    void CancelOwnedTicket() noexcept;

    task::exploration::ExplorationPhase phase_;
    std::shared_ptr<ExplorationBtContext> context_;
    std::shared_ptr<task::exploration::ExplorationSession> session_;
    std::optional<task::exploration::PhaseTicket> ticket_;
};

}  // namespace autonomy::task::plugins::exploration
