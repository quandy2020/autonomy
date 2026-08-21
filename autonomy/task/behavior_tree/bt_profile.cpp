/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/bt_profile.hpp"

#include <filesystem>

namespace autonomy {
namespace task {
namespace {

std::string JoinPath(const std::string& base, const std::string& rel)
{
    if (rel.empty()) {
        return base;
    }
    if (rel.front() == '/') {
        return rel;
    }
    std::filesystem::path path(base);
    path /= rel;
    return path.lexically_normal().string();
}

}  // namespace

BtProfile BtProfile::FromProto(
    const ::autonomy::task::proto::TaskBehaviorTreeProfile& proto)
{
    BtProfile profile;
    profile.default_tree_file = proto.default_tree_file();
    profile.alternate_tree_file = proto.alternate_tree_file();
    profile.plugin_libraries = {proto.plugin_lib_names().begin(),
                                proto.plugin_lib_names().end()};
    profile.plugin_lib_path = proto.plugin_lib_path();
    if (proto.bt_loop_duration_ms() > 0) {
        profile.loop_period_ms = proto.bt_loop_duration_ms();
    }
    if (proto.default_server_timeout_ms() > 0) {
        profile.server_timeout_ms = proto.default_server_timeout_ms();
    }
    return profile;
}

std::string BtProfile::ResolvePath(
    const std::string& config_directory,
    const std::string& relative_or_absolute) const
{
    if (relative_or_absolute.empty()) {
        return {};
    }
    return JoinPath(config_directory, relative_or_absolute);
}

std::string BtProfile::DefaultTreePath(
    const std::string& config_directory) const
{
    return ResolvePath(config_directory, default_tree_file);
}

std::string BtProfile::AlternateTreePath(
    const std::string& config_directory) const
{
    return ResolvePath(config_directory, alternate_tree_file);
}

}  // namespace task
}  // namespace autonomy
