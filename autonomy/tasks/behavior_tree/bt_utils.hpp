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
#include "autonomy/tasks/behavior_tree/behavior_tree_context.hpp"
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

template <typename T>
bool GetInputOrBlackboard(const BT::NodeConfiguration& conf,
                          const std::string& port_name, T& value) {
    if (!conf.blackboard) {
        return false;
    }
    return conf.blackboard->get(port_name, value);
}

void IncrementRecoveryCount(const BT::NodeConfiguration& conf);

void PopulateBlackboardDefaults(const std::shared_ptr<BehaviorTreeContext>& ctx,
                                const BT::Blackboard::Ptr& bb);

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
