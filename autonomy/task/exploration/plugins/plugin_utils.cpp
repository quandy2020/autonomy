/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::exploration {
namespace core = ::autonomy::task::exploration;

std::shared_ptr<ExplorationBtContext> ResolveContext(const BT::TreeNode& node) {
    std::shared_ptr<ExplorationBtContext> context;
    if (node.getInput("context", context) && context) return context;
    if (node.config().blackboard)
        (void)node.config().blackboard->get("exploration_context", context);
    return context;
}

BT::PortsList ContextPorts() {
    return {BT::InputPort<std::shared_ptr<ExplorationBtContext>>(
        "context", "task-owned exploration context")};
}

void PublishSnapshot(const BT::TreeNode& node, const core::ExplorationSnapshot& snapshot) {
    if (!node.config().blackboard) return;
    auto& board = *node.config().blackboard;
    board.set("exploration_snapshot", snapshot);
    board.set("exploration_finishing", snapshot.finishing);
    board.set("exploration_completed", snapshot.completion_confirmed);
    board.set("exploration_state", static_cast<int>(snapshot.state));
    board.set("exploration_reason", snapshot.reason);
}

const char* PhaseStatusName(core::PhaseStatus status) {
    switch (status) {
        case core::PhaseStatus::kPending: return "PENDING";
        case core::PhaseStatus::kSucceeded: return "SUCCEEDED";
        case core::PhaseStatus::kRetryable: return "RETRYABLE";
        case core::PhaseStatus::kInvalidated: return "INVALIDATED";
        case core::PhaseStatus::kFailed: return "FAILED";
        case core::PhaseStatus::kCanceled: return "CANCELED";
    }
    return "FAILED";
}

}  // namespace autonomy::task::plugins::exploration
