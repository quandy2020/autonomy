/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

#include <cstdlib>
#include <filesystem>
#include <sstream>

#include "autonomy/common/config.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/constants.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace {

namespace fs = std::filesystem;

bool FileExists(const std::string& path) {
    return !path.empty() && fs::is_regular_file(fs::path(path));
}

void AppendUnique(std::vector<std::string>* dirs, const std::string& dir) {
    if (dir.empty()) {
        return;
    }
    for (const auto& existing : *dirs) {
        if (existing == dir) {
            return;
        }
    }
    dirs->push_back(dir);
}

}  // namespace

std::string ResolveBehaviorTreeXmlPath(const std::string& basename_or_path) {
    if (basename_or_path.empty()) {
        return {};
    }
    if (FileExists(basename_or_path)) {
        return basename_or_path;
    }

    std::vector<std::string> bases;
    if (const char* env = std::getenv("AUTONOMY_CONFIG_PATH")) {
        AppendUnique(&bases, std::string(env) + "/tasks/behavior_tree");
        AppendUnique(&bases, std::string(env));
    }
    AppendUnique(&bases, std::string(common::kConfigurationFilesDirectory) +
                              "/tasks/behavior_tree");
    AppendUnique(&bases, std::string(common::kSourceDirectory) +
                              "/config/tasks/behavior_tree");

    for (const auto& base : bases) {
        const fs::path candidate = fs::path(base) / basename_or_path;
        if (FileExists(candidate.string())) {
            return candidate.string();
        }
    }
    return basename_or_path;
}

std::vector<std::string> ResolvePluginLibraryPaths(
    const proto::TaskOptions& options) {
    std::vector<std::string> dirs;
    if (!options.plugin_lib_path().empty()) {
        AppendUnique(&dirs, options.plugin_lib_path());
    }
    if (const char* env = std::getenv("AUTONOMY_BT_PLUGIN_PATH")) {
        std::stringstream ss(env);
        std::string token;
        while (std::getline(ss, token, ':')) {
            AppendUnique(&dirs, token);
        }
    }
    AppendUnique(&dirs, std::string(common::kLibraryBuildDir) + "/lib");
    AppendUnique(&dirs, std::string(common::kLibraryInstallDir) + "/lib");
    return dirs;
}

std::shared_ptr<BehaviorTreeContext> GetContextFromBlackboard(
    const BT::Blackboard::Ptr& bb) {
    if (!bb) {
        return nullptr;
    }
    std::shared_ptr<BehaviorTreeContext> ctx;
    if (!bb->get(kBlackboardContextKey, ctx)) {
        return nullptr;
    }
    return ctx;
}

std::shared_ptr<BehaviorTreeContext> GetContext(
    const BT::NodeConfiguration& conf) {
    return GetContextFromBlackboard(conf.blackboard);
}

void IncrementRecoveryCount(const BT::NodeConfiguration& conf) {
    int count = 0;
    [[maybe_unused]] auto res =
        conf.blackboard->get(kBlackboardNumberRecoveriesKey, count);
    count += 1;
    conf.blackboard->set(kBlackboardNumberRecoveriesKey, count);
    if (auto ctx = GetContext(conf)) {
        ctx->number_recoveries = count;
    }
}

void PopulateBlackboardDefaults(const std::shared_ptr<BehaviorTreeContext>& ctx,
                                const BT::Blackboard::Ptr& bb) {
    if (!ctx || !bb) {
        return;
    }
    bb->set(kBlackboardContextKey, ctx);
    bb->set(kBlackboardGlobalFrameKey, ctx->options.global_frame());
    bb->set(kBlackboardRobotBaseFrameKey, ctx->options.robot_base_frame());
    bb->set(
        kBlackboardBtLoopDurationKey,
        std::chrono::milliseconds(ctx->options.bt_loop_duration() > 0
                                      ? ctx->options.bt_loop_duration()
                                      : 10));
  bb->set(
        kBlackboardServerTimeoutKey,
        std::chrono::milliseconds(ctx->options.default_server_timeout() > 0
                                      ? ctx->options.default_server_timeout()
                                      : 20000));
    bb->set(kBlackboardNumberRecoveriesKey, 0);
    bb->set(kBlackboardLocalSurvivalTimeoutKey,
            ctx->options.local_survival_timeout() > 0.0
                ? ctx->options.local_survival_timeout()
                : 120.0);
    bb->set(kBlackboardGoalReachedTolKey,
            ctx->options.goal_reached_tolerance() > 0.0
                ? ctx->options.goal_reached_tolerance()
                : 0.25);
    bb->set(kBlackboardInitialPoseReceivedKey, true);
    bb->set("default_planner_id", ctx->options.default_planner_id());
    bb->set("default_controller_id", ctx->options.default_controller_id());
    bb->set("default_goal_checker_id", ctx->options.default_goal_checker_id());
    const std::string default_planner =
        ctx->options.default_planner_id().empty() ? "navfn_planner"
                                                  : ctx->options.default_planner_id();
    const std::string default_controller =
        ctx->options.default_controller_id().empty() ? "FollowPath"
                                                     : ctx->options.default_controller_id();
    const std::string default_smoother = "simple_smoother";
    bb->set("default_planner_id", default_planner);
    bb->set("default_controller_id", default_controller);
    bb->set("default_smoother_id", default_smoother);
    bb->set("selected_planner", default_planner);
    bb->set("selected_controller", default_controller);
    bb->set("selected_smoother", default_smoother);
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
