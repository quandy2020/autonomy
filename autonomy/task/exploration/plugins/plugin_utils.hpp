/* Copyright 2026 The Openbot Authors. */
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/exploration/exploration_session.hpp"

namespace autonomy::task::plugins::exploration {

// Owned by the task. All tree ticks and phase requests run on its owner thread.
// Sensor and remote-service callbacks enqueue inputs in the session instead.
struct ExplorationBtContext {
    std::shared_ptr<task::exploration::ExplorationSession> session;
    std::function<double()> clock = [] {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };
};

std::shared_ptr<ExplorationBtContext> ResolveContext(const BT::TreeNode& node);
BT::PortsList ContextPorts();
void PublishSnapshot(const BT::TreeNode& node,
                     const task::exploration::ExplorationSnapshot& snapshot);
const char* PhaseStatusName(task::exploration::PhaseStatus status);

}  // namespace autonomy::task::plugins::exploration
