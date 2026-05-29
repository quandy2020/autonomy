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

std::shared_ptr<BtContext> GetContextFromBlackboard(
    const BT::Blackboard::Ptr& bb) {
    if (!bb) {
        return nullptr;
    }
    std::shared_ptr<BtContext> ctx;
    if (!bb->get(kBlackboardContextKey, ctx)) {
        return nullptr;
    }
    return ctx;
}

std::shared_ptr<BtContext> GetContext(
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

void PopulateBlackboardDefaults(const std::shared_ptr<BtContext>& ctx,
                                const BT::Blackboard::Ptr& bb) {
    if (!ctx || !bb) {
        return;
    }
    bb->set(kBlackboardContextKey, ctx);
    if (ctx->autolink_node) {
        bb->set(kBlackboardAutolinkNodeKey, ctx->autolink_node);
    }
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
    bb->set("default_goal_checker_id", ctx->options.default_goal_checker_id());

    const std::string default_planner =
        ctx->options.default_planner_id().empty() ? "navfn_planner"
                                                  : ctx->options.default_planner_id();
    const std::string default_controller =
        ctx->options.default_controller_id().empty() ? "FollowPath"
                                                     : ctx->options.default_controller_id();
    const std::string default_smoother =
        ctx->options.default_smoother_id().empty() ? "simple_smoother"
                                                   : ctx->options.default_smoother_id();
    bb->set("default_planner_id", default_planner);
    bb->set("default_controller_id", default_controller);
    bb->set("default_smoother_id", default_smoother);
    bb->set("selected_planner", default_planner);
    bb->set("selected_controller", default_controller);
    bb->set("selected_smoother", default_smoother);
}

namespace {

task_proto::NavigateToPoseErrorCode MapGetPathError(
    task_proto::GetPathErrorCode outcome) {
    switch (outcome) {
        case task_proto::GET_PATH_SUCCESS:
            return task_proto::NAV_TO_POSE_NONE;
        case task_proto::GET_PATH_CANCELED:
        case task_proto::GET_PATH_STOPPED:
            return task_proto::NAV_TO_POSE_CANCELED;
        case task_proto::GET_PATH_INVALID_GOAL:
        case task_proto::GET_PATH_INVALID_START:
            return task_proto::NAV_TO_POSE_INVALID_GOAL;
        case task_proto::GET_PATH_TF_ERROR:
            return task_proto::NAV_TO_POSE_TF_ERROR;
        case task_proto::GET_PATH_NOT_INITIALIZED:
        case task_proto::GET_PATH_INVALID_PLUGIN:
            return task_proto::NAV_TO_POSE_NOT_INITIALIZED;
        case task_proto::GET_PATH_NO_PATH_FOUND:
        case task_proto::GET_PATH_EMPTY_PATH:
        case task_proto::GET_PATH_BLOCKED_START:
        case task_proto::GET_PATH_BLOCKED_GOAL:
        case task_proto::GET_PATH_OUT_OF_MAP:
            return task_proto::NAV_TO_POSE_NO_VALID_PATH;
        case task_proto::GET_PATH_PAT_EXCEEDED:
            return task_proto::NAV_TO_POSE_TIMEOUT;
        case task_proto::GET_PATH_MAP_ERROR:
            return task_proto::NAV_TO_POSE_PATH_INVALID;
        case task_proto::GET_PATH_INTERNAL_ERROR:
            return task_proto::NAV_TO_POSE_SMOOTHER_FAILED;
        case task_proto::GET_PATH_FAILURE:
        default:
            return task_proto::NAV_TO_POSE_PLANNER_FAILED;
    }
}

task_proto::NavigateToPoseErrorCode MapExePathError(
    task_proto::ExePathErrorCode outcome) {
    switch (outcome) {
        case task_proto::EXE_PATH_SUCCESS:
            return task_proto::NAV_TO_POSE_NONE;
        case task_proto::EXE_PATH_CANCELED:
        case task_proto::EXE_PATH_STOPPED:
            return task_proto::NAV_TO_POSE_CANCELED;
        case task_proto::EXE_PATH_TF_ERROR:
            return task_proto::NAV_TO_POSE_TF_ERROR;
        case task_proto::EXE_PATH_NOT_INITIALIZED:
        case task_proto::EXE_PATH_INVALID_PLUGIN:
            return task_proto::NAV_TO_POSE_NOT_INITIALIZED;
        case task_proto::EXE_PATH_INVALID_PATH:
        case task_proto::EXE_PATH_MISSED_PATH:
        case task_proto::EXE_PATH_BLOCKED_PATH:
        case task_proto::EXE_PATH_OUT_OF_MAP:
        case task_proto::EXE_PATH_MAP_ERROR:
            return task_proto::NAV_TO_POSE_PATH_INVALID;
        case task_proto::EXE_PATH_PAT_EXCEEDED:
            return task_proto::NAV_TO_POSE_TIMEOUT;
        case task_proto::EXE_PATH_MISSED_GOAL:
        case task_proto::EXE_PATH_BLOCKED_GOAL:
            return task_proto::NAV_TO_POSE_GOAL_CHECKER_FAILED;
        case task_proto::EXE_PATH_NO_VALID_CMD:
        case task_proto::EXE_PATH_ROBOT_STUCK:
        case task_proto::EXE_PATH_OSCILLATION:
        case task_proto::EXE_PATH_COLLISION:
        case task_proto::EXE_PATH_FAILURE:
        default:
            return task_proto::NAV_TO_POSE_CONTROLLER_FAILED;
    }
}

task_proto::NavigateToPoseErrorCode MapRecoveryError(
    task_proto::RecoveryErrorCode outcome) {
    switch (outcome) {
        case task_proto::RECOVERY_SUCCESS:
            return task_proto::NAV_TO_POSE_NONE;
        case task_proto::RECOVERY_CANCELED:
        case task_proto::RECOVERY_STOPPED:
            return task_proto::NAV_TO_POSE_CANCELED;
        case task_proto::RECOVERY_TF_ERROR:
            return task_proto::NAV_TO_POSE_TF_ERROR;
        case task_proto::RECOVERY_NOT_INITIALIZED:
        case task_proto::RECOVERY_INVALID_PLUGIN:
            return task_proto::NAV_TO_POSE_NOT_INITIALIZED;
        case task_proto::RECOVERY_PAT_EXCEEDED:
            return task_proto::NAV_TO_POSE_TIMEOUT;
        case task_proto::RECOVERY_IMPASSABLE:
            return task_proto::NAV_TO_POSE_PATH_INVALID;
        case task_proto::RECOVERY_FAILURE:
        case task_proto::RECOVERY_INTERNAL_ERROR:
        default:
            return task_proto::NAV_TO_POSE_UNKNOWN;
    }
}

task_proto::NavigateToPoseErrorCode MapMoveBaseError(
    task_proto::MoveBaseErrorCode outcome) {
    switch (outcome) {
        case task_proto::MOVE_BASE_SUCCESS:
            return task_proto::NAV_TO_POSE_NONE;
        case task_proto::MOVE_BASE_CANCELED:
            return task_proto::NAV_TO_POSE_CANCELED;
        case task_proto::MOVE_BASE_TF_ERROR:
            return task_proto::NAV_TO_POSE_TF_ERROR;
        case task_proto::MOVE_BASE_START_BLOCKED:
        case task_proto::MOVE_BASE_GOAL_BLOCKED:
            return task_proto::NAV_TO_POSE_NO_VALID_PATH;
        case task_proto::MOVE_BASE_COLLISION:
        case task_proto::MOVE_BASE_OSCILLATION:
            return task_proto::NAV_TO_POSE_CONTROLLER_FAILED;
        case task_proto::MOVE_BASE_PLAN_FAILURE:
            return task_proto::NAV_TO_POSE_PLANNER_FAILED;
        case task_proto::MOVE_BASE_CTRL_FAILURE:
            return task_proto::NAV_TO_POSE_CONTROLLER_FAILED;
        case task_proto::MOVE_BASE_FAILURE:
        case task_proto::MOVE_BASE_INTERNAL_ERROR:
        default:
            return task_proto::NAV_TO_POSE_UNKNOWN;
    }
}

task_proto::NavigateThroughPosesErrorCode MapNavToThrough(
    task_proto::NavigateToPoseErrorCode code) {
    switch (code) {
        case task_proto::NAV_TO_POSE_NONE:
            return task_proto::NAV_THROUGH_POSES_NONE;
        case task_proto::NAV_TO_POSE_TF_ERROR:
            return task_proto::NAV_THROUGH_POSES_TF_ERROR;
        case task_proto::NAV_TO_POSE_TIMEOUT:
            return task_proto::NAV_THROUGH_POSES_TIMEOUT;
        case task_proto::NAV_TO_POSE_NO_VALID_PATH:
            return task_proto::NAV_THROUGH_POSES_NO_VALID_PATH;
        case task_proto::NAV_TO_POSE_CANCELED:
            return task_proto::NAV_THROUGH_POSES_CANCELED;
        case task_proto::NAV_TO_POSE_PREEMPTED:
            return task_proto::NAV_THROUGH_POSES_PREEMPTED;
        case task_proto::NAV_TO_POSE_INVALID_GOAL:
            return task_proto::NAV_THROUGH_POSES_INVALID_GOAL;
        case task_proto::NAV_TO_POSE_NOT_INITIALIZED:
            return task_proto::NAV_THROUGH_POSES_NOT_INITIALIZED;
        case task_proto::NAV_TO_POSE_PLANNER_FAILED:
            return task_proto::NAV_THROUGH_POSES_PLANNER_FAILED;
        case task_proto::NAV_TO_POSE_CONTROLLER_FAILED:
            return task_proto::NAV_THROUGH_POSES_CONTROLLER_FAILED;
        case task_proto::NAV_TO_POSE_SMOOTHER_FAILED:
            return task_proto::NAV_THROUGH_POSES_SMOOTHER_FAILED;
        case task_proto::NAV_TO_POSE_PATH_INVALID:
            return task_proto::NAV_THROUGH_POSES_PATH_INVALID;
        case task_proto::NAV_TO_POSE_GOAL_CHECKER_FAILED:
            return task_proto::NAV_THROUGH_POSES_GOAL_CHECKER_FAILED;
        case task_proto::NAV_TO_POSE_FAILED_TO_LOAD_BEHAVIOR_TREE:
            return task_proto::NAV_THROUGH_POSES_FAILED_TO_LOAD_BEHAVIOR_TREE;
        case task_proto::NAV_TO_POSE_UNKNOWN:
        default:
            return task_proto::NAV_THROUGH_POSES_UNKNOWN;
    }
}

}  // namespace

task_proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    task_proto::GetPathErrorCode outcome) {
    return MapGetPathError(outcome);
}

task_proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    task_proto::ExePathErrorCode outcome) {
    return MapExePathError(outcome);
}

task_proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    task_proto::RecoveryErrorCode outcome) {
    return MapRecoveryError(outcome);
}

task_proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    task_proto::MoveBaseErrorCode outcome) {
    return MapMoveBaseError(outcome);
}

task_proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    const uint32_t outcome) {
    if (outcome >= 9000 && outcome < 9200) {
        return static_cast<task_proto::NavigateToPoseErrorCode>(outcome);
    }
    if (outcome >= 150 && outcome <= 199) {
        return MapRecoveryError(static_cast<task_proto::RecoveryErrorCode>(outcome));
    }
    if (outcome >= 100 && outcome <= 149) {
        return MapExePathError(static_cast<task_proto::ExePathErrorCode>(outcome));
    }
    if (outcome >= 50 && outcome <= 99) {
        return MapGetPathError(static_cast<task_proto::GetPathErrorCode>(outcome));
    }
    if (outcome >= 10 && outcome <= 49) {
        return MapMoveBaseError(static_cast<task_proto::MoveBaseErrorCode>(outcome));
    }
    if (outcome == 0) {
        return task_proto::NAV_TO_POSE_NONE;
    }
    return task_proto::NAV_TO_POSE_UNKNOWN;
}

task_proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    task_proto::GetPathErrorCode outcome) {
    return MapNavToThrough(MapGetPathError(outcome));
}

task_proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    task_proto::ExePathErrorCode outcome) {
    return MapNavToThrough(MapExePathError(outcome));
}

task_proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    task_proto::RecoveryErrorCode outcome) {
    return MapNavToThrough(MapRecoveryError(outcome));
}

task_proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    task_proto::MoveBaseErrorCode outcome) {
    return MapNavToThrough(MapMoveBaseError(outcome));
}

task_proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    const uint32_t outcome) {
    return MapNavToThrough(ToNavigateToPoseErrorCode(outcome));
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
