/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/behavior_tree/plugin_utils.hpp"
#include "autonomy/task/exploration/exploration_client.hpp"

namespace autonomy::task::plugins::exploration {

inline ::autonomy::task::exploration::ExplorationClient::Ptr ResolveClient(
    const BT::TreeNode& node) {
    return ::autonomy::task::plugins::ResolveClient<
        ::autonomy::task::exploration::ExplorationClient>(node);
}

using ::autonomy::task::plugins::SetErrorPorts;
using ::autonomy::task::plugins::ClearErrorPorts;

}  // namespace autonomy::task::plugins::exploration
