/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/apps/localization/localization_client.hpp"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy::task::plugins::localization {

inline ::autonomy::task::localization::LocalizationClient::Ptr ResolveClient(
    const BT::TreeNode& node)
{
    return ::autonomy::task::localization::LocalizationClient::FromNode(node);
}

}  // namespace autonomy::task::plugins::localization
