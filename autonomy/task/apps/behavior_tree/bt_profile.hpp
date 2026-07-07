/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/task/proto/task_options.pb.h"

namespace autonomy {
namespace task {

/** Behavior-tree profile for one task type (mirrors TaskBehaviorTreeProfile). */
struct BtProfile
{
    std::string default_tree_file;
    std::string alternate_tree_file;
    std::vector<std::string> plugin_libraries;
    std::string plugin_lib_path;
    uint32_t loop_period_ms{10};
    uint32_t server_timeout_ms{20000};

    static BtProfile FromProto(
        const ::autonomy::task::proto::TaskBehaviorTreeProfile& proto);

    std::string ResolvePath(const std::string& config_directory,
                            const std::string& relative_or_absolute) const;

    std::string DefaultTreePath(const std::string& config_directory) const;

    std::string AlternateTreePath(const std::string& config_directory) const;
};

}  // namespace task
}  // namespace autonomy
