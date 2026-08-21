/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/behavior_tree/plugin_utils.hpp"
#include "autonomy/task/mapping/mapping_client.hpp"

namespace autonomy::task::plugins::mapping {

inline ::autonomy::task::mapping::MappingClient::Ptr ResolveClient(
    const BT::TreeNode& node) {
    return ::autonomy::task::plugins::ResolveClient<
        ::autonomy::task::mapping::MappingClient>(node);
}

}  // namespace autonomy::task::plugins::mapping
