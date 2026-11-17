/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/plugins/phase_action.hpp"

#include <exception>

namespace autonomy::task::plugins::exploration {
namespace core = ::autonomy::task::exploration;

PhaseAction::PhaseAction(const std::string& name, const BT::NodeConfig& config,
                         core::ExplorationPhase phase)
    : BtStatefulAction(name, config), phase_(phase) {}

BT::PortsList PhaseAction::providedPorts() {
    auto ports = ContextPorts();
    ports.insert(BT::OutputPort<std::string>("phase_status"));
    ports.insert(BT::OutputPort<std::string>("error_msg"));
    return ports;
}

BT::NodeStatus PhaseAction::Failure(const std::string& reason) {
    setOutput("phase_status", std::string("FAILED"));
    setOutput("error_msg", reason);
    return BT::NodeStatus::FAILURE;
}

bool PhaseAction::LatchAfterFirstTick() const { return ticket_.has_value(); }

BT::NodeStatus PhaseAction::OnFirstTick() {
    try {
        context_ = ResolveContext(*this);
        if (!context_ || !context_->session || !context_->clock)
            return Failure("exploration context, session or clock is unavailable");
        session_ = context_->session;
        if (session_->driver() != core::ExplorationDriver::kBehaviorTree)
            return Failure("exploration BT requires the mutually exclusive BT driver");
        // A prior successful tick is reset by the parent without invoking halt.
        // Its ticket must not be mistaken for this new invocation.
        ticket_.reset();
        ticket_ = session_->RequestPhase(phase_, context_->clock());
        if (!ticket_) {
            setOutput("phase_status", std::string("PENDING"));
            setOutput("error_msg", std::string("another phase owns the session"));
            return BT::NodeStatus::RUNNING;
        }
        return Poll();
    } catch (const std::exception& error) {
        CancelOwnedTicket();
        return Failure(error.what());
    } catch (...) {
        CancelOwnedTicket();
        return Failure("non-standard exception while requesting an exploration phase");
    }
}

BT::NodeStatus PhaseAction::OnExecute() {
    try {
        const auto binding = ResolveContext(*this);
        if (binding != context_ || !binding || binding->session != session_) {
            CancelOwnedTicket();
            return Failure("exploration context changed during an active phase");
        }
        return Poll();
    } catch (const std::exception& error) {
        CancelOwnedTicket();
        return Failure(error.what());
    } catch (...) {
        CancelOwnedTicket();
        return Failure("non-standard exception while polling an exploration phase");
    }
}

BT::NodeStatus PhaseAction::Poll() {
    if (!session_ || !ticket_) return Failure("phase request is unavailable");
    const auto result = session_->PhaseState(*ticket_);
    setOutput("phase_status", std::string(PhaseStatusName(result.status)));
    setOutput("error_msg", result.reason);
    const auto snapshot = session_->Snapshot();
    PublishSnapshot(*this, snapshot);
    if (result.status == core::PhaseStatus::kPending) return BT::NodeStatus::RUNNING;
    if (result.status == core::PhaseStatus::kSucceeded) {
        if (!PublishSuccess(snapshot))
            return Failure("successful phase no longer has its required output");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

bool PhaseAction::PublishSuccess(const core::ExplorationSnapshot&) { return true; }

void PhaseAction::CancelOwnedTicket() noexcept {
    try {
        if (session_ && ticket_) session_->CancelPhase(*ticket_);
    } catch (...) {
        // Native halt/destruction must not throw across the tree boundary.
    }
    ticket_.reset();
    session_.reset();
    context_.reset();
}

void PhaseAction::OnHalted() { CancelOwnedTicket(); }

}  // namespace autonomy::task::plugins::exploration
