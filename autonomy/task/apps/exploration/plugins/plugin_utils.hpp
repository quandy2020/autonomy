/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/apps/exploration/exploration_client.hpp"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy::task::plugins::exploration {

inline ::autonomy::task::exploration::ExplorationClient::Ptr ResolveClient(
    const BT::TreeNode& node)
{
    return ::autonomy::task::exploration::ExplorationClient::FromNode(node);
}

inline void SetErrorPorts(BT::TreeNode& node, int code, const std::string& message)
{
    node.setOutput("error_code_id", code);
    node.setOutput("error_msg", message);
}

inline void ClearErrorPorts(BT::TreeNode& node)
{
    node.setOutput("error_code_id", 0);
    node.setOutput("error_msg", std::string{});
}

}  // namespace autonomy::task::plugins::exploration
