/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/behavior_tree/plugin_utils.hpp"
#include "autonomy/task/charging/charging_client.hpp"

namespace autonomy::task::plugins::charging {

inline ::autonomy::task::charging::ChargingClient::Ptr ResolveClient(
    const BT::TreeNode& node) {
    return ::autonomy::task::plugins::ResolveClient<
        ::autonomy::task::charging::ChargingClient>(node);
}

}  // namespace autonomy::task::plugins::charging
