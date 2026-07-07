/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/apps/tracking/tracking_client.hpp"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy::task::plugins::tracking {

inline ::autonomy::task::tracking::TrackingClient::Ptr ResolveClient(
    const BT::TreeNode& node)
{
    return ::autonomy::task::tracking::TrackingClient::FromNode(node);
}

}  // namespace autonomy::task::plugins::tracking
