/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/behavior_tree/plugin_utils.hpp"
#include "autonomy/task/teleop/teleop_client.hpp"

namespace autonomy::task::plugins::teleop {

inline ::autonomy::task::teleop::TeleopClient::Ptr ResolveClient(
    const BT::TreeNode& node) {
    return ::autonomy::task::plugins::ResolveClient<
        ::autonomy::task::teleop::TeleopClient>(node);
}

}  // namespace autonomy::task::plugins::teleop
