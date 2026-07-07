/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/apps/teleop/teleop_client.hpp"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy::task::plugins::teleop {

inline ::autonomy::task::teleop::TeleopClient::Ptr ResolveClient(
    const BT::TreeNode& node)
{
    return ::autonomy::task::teleop::TeleopClient::FromNode(node);
}

}  // namespace autonomy::task::plugins::teleop
