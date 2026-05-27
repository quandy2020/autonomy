/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/context.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/** Resolve BT XML path from basename or absolute path. */
std::string ResolveBehaviorTreeXmlPath(const std::string& basename_or_path);

/** Search directories for plugin shared libraries. */
std::vector<std::string> ResolvePluginLibraryPaths(
    const proto::TaskOptions& options);

std::shared_ptr<BehaviorTreeContext> GetContextFromBlackboard(
    const BT::Blackboard::Ptr& bb);

std::shared_ptr<BehaviorTreeContext> GetContext(
    const BT::NodeConfiguration& conf);

inline bool IsCancelRequested(const BT::NodeConfiguration& conf) {
    auto ctx = GetContext(conf);
    return ctx && ctx->cancel_requested.load();
}

inline bool WaitIfPaused(const BT::NodeConfiguration& conf) {
    auto ctx = GetContext(conf);
    return ctx && ctx->IsPaused();
}

void IncrementRecoveryCount(const BT::NodeConfiguration& conf);

void PopulateBlackboardDefaults(const std::shared_ptr<BehaviorTreeContext>& ctx,
                                const BT::Blackboard::Ptr& bb);

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
