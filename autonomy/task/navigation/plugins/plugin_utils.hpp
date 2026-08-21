/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/behavior_tree/plugin_utils.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"

namespace autonomy::task::plugins::navigation {

inline ::autonomy::task::navigation::NavigationClient::Ptr ResolveClient(
    const BT::TreeNode& node) {
    return ::autonomy::task::plugins::ResolveClient<
        ::autonomy::task::navigation::NavigationClient>(node);
}

using ::autonomy::task::plugins::SetErrorPorts;
using ::autonomy::task::plugins::ClearErrorPorts;

[[nodiscard]] inline automsgs::msgs::builtin_interfaces::Duration
ToProtoDuration(double seconds) {
    return ::autonomy::task::plugins::DurationFromSeconds(seconds);
}

}  // namespace autonomy::task::plugins::navigation
