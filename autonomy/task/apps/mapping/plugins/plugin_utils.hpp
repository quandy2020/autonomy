/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/apps/mapping/mapping_client.hpp"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy::task::plugins::mapping {

inline ::autonomy::task::mapping::MappingClient::Ptr ResolveClient(
    const BT::TreeNode& node)
{
    return ::autonomy::task::mapping::MappingClient::FromNode(node);
}

}  // namespace autonomy::task::plugins::mapping
