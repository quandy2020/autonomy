/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/task/behavior_tree/plugin_utils.hpp"
#include "autonomy/task/localization/localization_client.hpp"

namespace autonomy::task::plugins::localization {

inline ::autonomy::task::localization::LocalizationClient::Ptr ResolveClient(
    const BT::TreeNode& node) {
    return ::autonomy::task::plugins::ResolveClient<
        ::autonomy::task::localization::LocalizationClient>(node);
}

}  // namespace autonomy::task::plugins::localization
