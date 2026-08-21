/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/behavior_tree/plugin_utils.hpp"
#include "autonomy/task/tracking/tracking_client.hpp"

namespace autonomy::task::plugins::tracking {

inline ::autonomy::task::tracking::TrackingClient::Ptr ResolveClient(
    const BT::TreeNode& node) {
    return ::autonomy::task::plugins::ResolveClient<
        ::autonomy::task::tracking::TrackingClient>(node);
}

}  // namespace autonomy::task::plugins::tracking
