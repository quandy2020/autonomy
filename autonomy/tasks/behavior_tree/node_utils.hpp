/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/tasks/behavior_tree/context.hpp"
#include "autonomy/tasks/behavior_tree/utils.hpp"
#include "autonomy/tasks/behavior_tree/error_codes.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/** Standard MBF-style error ports for planner/controller BT nodes. */
inline BT::PortsList ErrorOutcomePorts() {
    return {
        BT::OutputPort<uint16_t>("error_code_id"),
        BT::OutputPort<std::string>("error_msg"),
    };
}

inline BT::PortsList& AppendErrorOutcomePorts(BT::PortsList& ports) {
    ports.insert(ErrorOutcomePorts().begin(), ErrorOutcomePorts().end());
    return ports;
}

/** Read an input port, then fall back to a blackboard key. */
template <typename T, typename Node>
bool GetInputOrBB(const Node& node, const std::string& port,
                  const char* bb_key, T& value) {
    if (node.getInput(port, value)) {
        return true;
    }
    if (bb_key == nullptr || !node.config().blackboard) {
        return false;
    }
    return node.config().blackboard->get(bb_key, value);
}

/** Resolve plugin id: port → selected_* blackboard → TaskOptions default. */
inline std::string ResolvePluginId(const BT::TreeNode& node,
                                   const BehaviorTreeContext& ctx,
                                   const char* port, const char* selected_key,
                                   const std::string& default_id) {
    std::string id;
    if (node.getInput(port, id) && !id.empty()) {
        return id;
    }
    if (node.config().blackboard &&
        node.config().blackboard->get(selected_key, id) && !id.empty()) {
        return id;
    }
    return default_id;
}

inline std::string ResolvePlannerId(const BT::TreeNode& node,
                                    const BehaviorTreeContext& ctx) {
    return ResolvePluginId(node, ctx, "planner_id", "selected_planner",
                         ctx.options.default_planner_id());
}

inline std::string ResolveControllerId(const BT::TreeNode& node,
                                       const BehaviorTreeContext& ctx) {
    return ResolvePluginId(node, ctx, "controller_id", "selected_controller",
                         ctx.options.default_controller_id());
}

inline std::string ResolveSmootherId(const BT::TreeNode& node,
                                     const BehaviorTreeContext& ctx) {
    const std::string fallback = ctx.options.default_smoother_id().empty()
                                     ? "simple_smoother"
                                     : ctx.options.default_smoother_id();
    return ResolvePluginId(node, ctx, "smoother_id", "selected_smoother",
                           fallback);
}

inline bool GetGlobalStartPose(const BehaviorTreeContext& ctx,
                               commsgs::geometry_msgs::PoseStamped& start) {
    if (!ctx.controller) {
        return false;
    }
    return utils::getGlobalRobotPose(
        start, ctx.tf_buffer, ctx.controller->GetOdomSmoother(),
        ctx.options.global_frame(), ctx.options.robot_base_frame());
}

inline void SetGetPathError(BT::TreeNode& node, uint16_t code,
                            const std::string& msg = {}) {
    node.setOutput("error_code_id", code);
    node.setOutput("error_msg", msg);
}

inline void SetExePathError(BT::TreeNode& node, uint16_t code,
                            const std::string& msg = {}) {
    node.setOutput("error_code_id", code);
    node.setOutput("error_msg", msg);
}

inline void PublishPath(BT::TreeNode& node,
                        const commsgs::planning_msgs::Path& path) {
    node.setOutput("path", path);
    node.setOutput(kBlackboardPathKey, path);
    SetGetPathError(node, GetPathOutcome::SUCCESS);
}

inline BT::NodeStatus MapRecoveryTick(
    control::ControllerServer::RecoveryTickResult result) {
    switch (result) {
        case control::ControllerServer::RecoveryTickResult::Running:
            return BT::NodeStatus::RUNNING;
        case control::ControllerServer::RecoveryTickResult::Succeeded:
        case control::ControllerServer::RecoveryTickResult::Cancelled:
            return BT::NodeStatus::SUCCESS;
        default:
            return BT::NodeStatus::FAILURE;
    }
}

inline BT::NodeStatus MapFollowPathTick(
    BT::TreeNode& node,
    control::ControllerServer::FollowPathTickResult result,
    const std::string& failure_msg = "FollowPath failed.") {
    switch (result) {
        case control::ControllerServer::FollowPathTickResult::Running:
            return BT::NodeStatus::RUNNING;
        case control::ControllerServer::FollowPathTickResult::Succeeded:
            SetExePathError(node, ExePathOutcome::SUCCESS);
            return BT::NodeStatus::SUCCESS;
        case control::ControllerServer::FollowPathTickResult::Cancelled:
            SetExePathError(node, ExePathOutcome::CANCELED);
            return BT::NodeStatus::SUCCESS;
        case control::ControllerServer::FollowPathTickResult::Failed:
        default:
            SetExePathError(node, ExePathOutcome::FAILURE, failure_msg);
            return BT::NodeStatus::FAILURE;
    }
}

/**
 * @brief One-shot get-path BT tick: resolve start, call planner, write path ports.
 */
template <typename Node, typename GoalT, typename ComputeFn>
BT::NodeStatus RunGetPath(Node& node, const std::shared_ptr<BehaviorTreeContext>& ctx,
                          const std::string& goal_port, const char* goal_bb_key,
                          ComputeFn&& compute) {
    if (!ctx || !ctx->planner) {
        return BT::NodeStatus::FAILURE;
    }
    GoalT goal{};
    if (!GetInputOrBB(node, goal_port, goal_bb_key, goal)) {
        SetGetPathError(node, GetPathOutcome::INVALID_GOAL, "Goal unavailable.");
        return BT::NodeStatus::FAILURE;
    }
    if constexpr (std::is_same_v<GoalT,
                                 std::vector<commsgs::geometry_msgs::PoseStamped>>) {
        if (goal.empty()) {
            SetGetPathError(node, GetPathOutcome::INVALID_GOAL, "Empty goals.");
            return BT::NodeStatus::FAILURE;
        }
    }
    const std::string planner_id = ResolvePlannerId(node, *ctx);
    commsgs::geometry_msgs::PoseStamped start;
    if (!GetGlobalStartPose(*ctx, start)) {
        SetGetPathError(node, GetPathOutcome::TF_ERROR, "Robot pose unavailable.");
        return BT::NodeStatus::FAILURE;
    }
    try {
        auto path = compute(*ctx, start, goal, planner_id);
        PublishPath(node, path);
        return BT::NodeStatus::SUCCESS;
    } catch (const planning::common::PlannerException& ex) {
        SetGetPathError(node, GetPathOutcome::NO_PATH_FOUND, ex.what());
        node.setOutput("path", commsgs::planning_msgs::Path{});
        return BT::NodeStatus::FAILURE;
    }
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"

#define REGISTER_BEHAVIOR_TREE_NODE(Type, XmlName)          \
    BT_REGISTER_NODES(factory) {                          \
        factory.registerNodeType<autonomy::tasks::behavior_tree::Type>( \
            XmlName);                                     \
    }
