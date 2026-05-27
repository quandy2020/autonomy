/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

inline bool IsCancelRequested(const BT::NodeConfiguration& conf) {
    auto ctx = GetContext(conf);
    return ctx && ctx->cancel_requested.load();
}

inline bool WaitIfPaused(const BT::NodeConfiguration& conf) {
    auto ctx = GetContext(conf);
    return ctx && ctx->IsPaused();
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
