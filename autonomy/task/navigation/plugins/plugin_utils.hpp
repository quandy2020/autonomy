/*
 * Copyright 2026 The Openbot Authors
 * @file plugin_utils.hpp
 * @brief Resolve native navigation clients and invocation-owned dispatch gates.
 */

#pragma once

#include "autonomy/task/behavior_tree/plugin_utils.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
#include "autonomy/task/common/motion_dispatch_gate.hpp"

namespace autonomy::task::plugins::navigation {

/**
 * @brief Resolve the invocation gate without silently bypassing native ownership.
 * @return Gate, or null only for an explicitly legacy non-NavigationTask tree.
 * @throws std::runtime_error When a native invocation has no valid gate.
 */
inline std::shared_ptr<common::MotionDispatchGate> ResolveMotionGate(const BT::TreeNode& node) {
    std::shared_ptr<common::MotionDispatchGate> gate;
    const auto board = node.config().blackboard;
    if (board) board->get(common::kMotionDispatchGateBlackboardKey, gate);
    uint64_t epoch = 0;
    if (!gate && board && board->get("navigation_invocation_epoch", epoch) && epoch != 0)
        throw std::runtime_error("native navigation invocation has no motion dispatch gate");
    return gate;
}

/** @brief Resolve the native NavigationClient associated with the node's invocation. */
inline ::autonomy::task::navigation::NavigationClient::Ptr ResolveClient(
    const BT::TreeNode& node) {
    return ::autonomy::task::plugins::ResolveClient<
        ::autonomy::task::navigation::NavigationClient>(node);
}

using ::autonomy::task::plugins::SetErrorPorts;
using ::autonomy::task::plugins::ClearErrorPorts;

[[nodiscard]] inline automsgs::msgs::builtin_interfaces::Duration
ToProtoDuration(double seconds) {
    return ::autonomy::task::plugins::DurationFromSeconds(seconds);
}

}  // namespace autonomy::task::plugins::navigation
