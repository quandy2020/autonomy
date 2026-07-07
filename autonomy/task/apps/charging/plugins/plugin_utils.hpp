/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/apps/charging/charging_client.hpp"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy::task::plugins::charging {

inline ::autonomy::task::charging::ChargingClient::Ptr ResolveClient(
    const BT::TreeNode& node)
{
    return ::autonomy::task::charging::ChargingClient::FromNode(node);
}

}  // namespace autonomy::task::plugins::charging
