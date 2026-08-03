/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/task/apps/navigation/navigation_client.hpp"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy::task::plugins::navigation {

inline ::autonomy::task::navigation::NavigationClient::Ptr ResolveClient(
    const BT::TreeNode& node)
{
    return ::autonomy::task::navigation::NavigationClient::FromNode(node);
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

inline automsgs::msgs::builtin_interfaces::Duration ToProtoDuration(double seconds)
{
    return 
        automsgs::msgs::builtin_interfaces::DurationFromSeconds(seconds);
}

}  // namespace autonomy::task::plugins::navigation
