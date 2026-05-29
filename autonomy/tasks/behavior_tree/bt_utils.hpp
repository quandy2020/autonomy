/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/task_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/tasks/behavior_tree/bt_context.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

namespace task_proto {
using namespace commsgs::proto::task_msgs;
}  // namespace task_proto

inline std::shared_ptr<BtContext> GetBtContext(
    const BT::Blackboard::Ptr& blackboard) {
    std::shared_ptr<BtContext> ctx;
    if (blackboard) {
        blackboard->get(kBlackboardContextKey, ctx);
    }
    return ctx;
}

/** Robot pose from blackboard (updated before each BT tick) or TF fallback. */
inline bool TryGetRobotPose(const BT::Blackboard::Ptr& blackboard,
                            commsgs::geometry_msgs::PoseStamped& pose) {
    if (blackboard && blackboard->get("current_pose", pose)) {
        return true;
    }
    const auto ctx = GetBtContext(blackboard);
    if (!ctx || !ctx->tf_buffer || !ctx->controller) {
        return false;
    }
    return utils::getGlobalRobotPose(
        pose, ctx->tf_buffer, ctx->controller->GetOdomSmoother(),
        ctx->options.global_frame(), ctx->options.robot_base_frame());
}

/** Publishes path to ROS /plan via Autonomy::NotifyPath when bt_context::on_path is set. */
inline void NotifyGlobalPath(const BT::Blackboard::Ptr& blackboard,
                             const commsgs::planning_msgs::Path& path) {
    const auto ctx = GetBtContext(blackboard);
    if (ctx && ctx->on_path) {
        ctx->on_path(path);
    }
}

/** MBF GetPath outcome constants for BT blackboard error_code_id. */
struct GetPathOutcome {
    static constexpr uint16_t SUCCESS =
        static_cast<uint16_t>(task_proto::GET_PATH_SUCCESS);
    static constexpr uint16_t FAILURE =
        static_cast<uint16_t>(task_proto::GET_PATH_FAILURE);
    static constexpr uint16_t CANCELED =
        static_cast<uint16_t>(task_proto::GET_PATH_CANCELED);
    static constexpr uint16_t INVALID_START =
        static_cast<uint16_t>(task_proto::GET_PATH_INVALID_START);
    static constexpr uint16_t INVALID_GOAL =
        static_cast<uint16_t>(task_proto::GET_PATH_INVALID_GOAL);
    static constexpr uint16_t BLOCKED_START =
        static_cast<uint16_t>(task_proto::GET_PATH_BLOCKED_START);
    static constexpr uint16_t BLOCKED_GOAL =
        static_cast<uint16_t>(task_proto::GET_PATH_BLOCKED_GOAL);
    static constexpr uint16_t NO_PATH_FOUND =
        static_cast<uint16_t>(task_proto::GET_PATH_NO_PATH_FOUND);
    static constexpr uint16_t PAT_EXCEEDED =
        static_cast<uint16_t>(task_proto::GET_PATH_PAT_EXCEEDED);
    static constexpr uint16_t EMPTY_PATH =
        static_cast<uint16_t>(task_proto::GET_PATH_EMPTY_PATH);
    static constexpr uint16_t TF_ERROR =
        static_cast<uint16_t>(task_proto::GET_PATH_TF_ERROR);
    static constexpr uint16_t NOT_INITIALIZED =
        static_cast<uint16_t>(task_proto::GET_PATH_NOT_INITIALIZED);
    static constexpr uint16_t INVALID_PLUGIN =
        static_cast<uint16_t>(task_proto::GET_PATH_INVALID_PLUGIN);
    static constexpr uint16_t INTERNAL_ERROR =
        static_cast<uint16_t>(task_proto::GET_PATH_INTERNAL_ERROR);
    static constexpr uint16_t OUT_OF_MAP =
        static_cast<uint16_t>(task_proto::GET_PATH_OUT_OF_MAP);
    static constexpr uint16_t MAP_ERROR =
        static_cast<uint16_t>(task_proto::GET_PATH_MAP_ERROR);
    static constexpr uint16_t STOPPED =
        static_cast<uint16_t>(task_proto::GET_PATH_STOPPED);
};

/** MBF ExePath outcome constants for BT blackboard error_code_id. */
struct ExePathOutcome {
    static constexpr uint16_t SUCCESS =
        static_cast<uint16_t>(task_proto::EXE_PATH_SUCCESS);
    static constexpr uint16_t FAILURE =
        static_cast<uint16_t>(task_proto::EXE_PATH_FAILURE);
    static constexpr uint16_t CANCELED =
        static_cast<uint16_t>(task_proto::EXE_PATH_CANCELED);
    static constexpr uint16_t NO_VALID_CMD =
        static_cast<uint16_t>(task_proto::EXE_PATH_NO_VALID_CMD);
    static constexpr uint16_t PAT_EXCEEDED =
        static_cast<uint16_t>(task_proto::EXE_PATH_PAT_EXCEEDED);
    static constexpr uint16_t COLLISION =
        static_cast<uint16_t>(task_proto::EXE_PATH_COLLISION);
    static constexpr uint16_t OSCILLATION =
        static_cast<uint16_t>(task_proto::EXE_PATH_OSCILLATION);
    static constexpr uint16_t ROBOT_STUCK =
        static_cast<uint16_t>(task_proto::EXE_PATH_ROBOT_STUCK);
    static constexpr uint16_t MISSED_GOAL =
        static_cast<uint16_t>(task_proto::EXE_PATH_MISSED_GOAL);
    static constexpr uint16_t MISSED_PATH =
        static_cast<uint16_t>(task_proto::EXE_PATH_MISSED_PATH);
    static constexpr uint16_t BLOCKED_GOAL =
        static_cast<uint16_t>(task_proto::EXE_PATH_BLOCKED_GOAL);
    static constexpr uint16_t BLOCKED_PATH =
        static_cast<uint16_t>(task_proto::EXE_PATH_BLOCKED_PATH);
    static constexpr uint16_t INVALID_PATH =
        static_cast<uint16_t>(task_proto::EXE_PATH_INVALID_PATH);
    static constexpr uint16_t TF_ERROR =
        static_cast<uint16_t>(task_proto::EXE_PATH_TF_ERROR);
    static constexpr uint16_t NOT_INITIALIZED =
        static_cast<uint16_t>(task_proto::EXE_PATH_NOT_INITIALIZED);
    static constexpr uint16_t INVALID_PLUGIN =
        static_cast<uint16_t>(task_proto::EXE_PATH_INVALID_PLUGIN);
    static constexpr uint16_t INTERNAL_ERROR =
        static_cast<uint16_t>(task_proto::EXE_PATH_INTERNAL_ERROR);
    static constexpr uint16_t OUT_OF_MAP =
        static_cast<uint16_t>(task_proto::EXE_PATH_OUT_OF_MAP);
    static constexpr uint16_t MAP_ERROR =
        static_cast<uint16_t>(task_proto::EXE_PATH_MAP_ERROR);
    static constexpr uint16_t STOPPED =
        static_cast<uint16_t>(task_proto::EXE_PATH_STOPPED);
};

/** MBF Recovery outcome constants. */
struct RecoveryOutcome {
    static constexpr uint16_t SUCCESS =
        static_cast<uint16_t>(task_proto::RECOVERY_SUCCESS);
    static constexpr uint16_t FAILURE =
        static_cast<uint16_t>(task_proto::RECOVERY_FAILURE);
    static constexpr uint16_t CANCELED =
        static_cast<uint16_t>(task_proto::RECOVERY_CANCELED);
    static constexpr uint16_t TF_ERROR =
        static_cast<uint16_t>(task_proto::RECOVERY_TF_ERROR);
    static constexpr uint16_t IMPASSABLE =
        static_cast<uint16_t>(task_proto::RECOVERY_IMPASSABLE);
};

task_proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    task_proto::GetPathErrorCode outcome);
task_proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    task_proto::ExePathErrorCode outcome);
task_proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    task_proto::RecoveryErrorCode outcome);
task_proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(
    task_proto::MoveBaseErrorCode outcome);
task_proto::NavigateToPoseErrorCode ToNavigateToPoseErrorCode(uint32_t outcome);

task_proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    task_proto::GetPathErrorCode outcome);
task_proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    task_proto::ExePathErrorCode outcome);
task_proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    task_proto::RecoveryErrorCode outcome);
task_proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    task_proto::MoveBaseErrorCode outcome);
task_proto::NavigateThroughPosesErrorCode ToNavigateThroughPosesErrorCode(
    uint32_t outcome);

std::string ResolveBehaviorTreeXmlPath(const std::string& basename_or_path);

std::vector<std::string> ResolvePluginLibraryPaths(
    const proto::TaskOptions& options);

std::shared_ptr<BtContext> GetContextFromBlackboard(
    const BT::Blackboard::Ptr& bb);

std::shared_ptr<BtContext> GetContext(
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

void PopulateBlackboardDefaults(const std::shared_ptr<BtContext>& ctx,
                                const BT::Blackboard::Ptr& bb);

inline map::costmap_2d::Costmap2DWrapper::SharedPtr ResolveCostmap(
    const std::shared_ptr<BtContext>& ctx,
    const std::string& service_name) {
    if (!ctx) {
        return nullptr;
    }
    if (service_name.find("global") != std::string::npos) {
        return ctx->planner ? ctx->planner->GetCostmapWrapper() : nullptr;
    }
    if (ctx->controller) {
        if (auto local = ctx->controller->GetCostmapWrapper()) {
            return local;
        }
    }
    return ctx->planner ? ctx->planner->GetCostmapWrapper() : nullptr;
}

inline bool GetRobotMapPose(map::costmap_2d::Costmap2DWrapper* wrapper,
                            double& rx, double& ry) {
    if (!wrapper) {
        return false;
    }
    commsgs::geometry_msgs::PoseStamped pose;
    if (!wrapper->getRobotPose(pose)) {
        return false;
    }
    rx = pose.pose.position.x;
    ry = pose.pose.position.y;
    return true;
}

inline void ClearCostmapByDistance(map::costmap_2d::Costmap2D* map, double rx,
                                   double ry, double radius,
                                   bool clear_outside) {
    if (!map || radius <= 0.0) {
        return;
    }
    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(
        *(map->getMutex()));
    const unsigned int sx = map->getSizeInCellsX();
    const unsigned int sy = map->getSizeInCellsY();
    const double radius_sq = radius * radius;
    for (unsigned int my = 0; my < sy; ++my) {
        for (unsigned int mx = 0; mx < sx; ++mx) {
            double wx = 0.0;
            double wy = 0.0;
            map->mapToWorld(mx, my, wx, wy);
            const double dx = wx - rx;
            const double dy = wy - ry;
            const bool in_disk = (dx * dx + dy * dy) <= radius_sq;
            if (clear_outside ? !in_disk : in_disk) {
                map->setCost(mx, my, map::costmap_2d::FREE_SPACE);
            }
        }
    }
}

inline bool PoseStampedEqual(const commsgs::geometry_msgs::PoseStamped& a,
                             const commsgs::geometry_msgs::PoseStamped& b,
                             double pos_tol = 1e-4) {
    if (a.header.frame_id != b.header.frame_id) {
        return false;
    }
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    const double dz = a.pose.position.z - b.pose.position.z;
    if ((dx * dx + dy * dy + dz * dz) > pos_tol * pos_tol) {
        return false;
    }
    const double dot =
        a.pose.orientation.w * b.pose.orientation.w +
        a.pose.orientation.x * b.pose.orientation.x +
        a.pose.orientation.y * b.pose.orientation.y +
        a.pose.orientation.z * b.pose.orientation.z;
    return std::abs(dot) > 0.999;
}

inline bool GoalsEqual(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& a,
    const std::vector<commsgs::geometry_msgs::PoseStamped>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (!PoseStampedEqual(a[i], b[i])) {
            return false;
        }
    }
    return true;
}

inline bool PathEqual(const commsgs::planning_msgs::Path& a,
                      const commsgs::planning_msgs::Path& b) {
    if (a.poses.size() != b.poses.size()) {
        return false;
    }
    if (a.poses.empty()) {
        return true;
    }
    return PoseStampedEqual(a.poses.front(), b.poses.front()) &&
           PoseStampedEqual(a.poses.back(), b.poses.back());
}

inline BT::PortsList AppendErrorOutcomePorts(BT::PortsList ports) {
    ports.insert(BT::OutputPort<uint16_t>("error_code_id"));
    ports.insert(BT::OutputPort<std::string>("error_msg"));
    return ports;
}

template <typename NodeT, typename T>
bool GetInputOrBlackboard(NodeT& node, const BT::NodeConfiguration& conf,
                          const std::string& port_name,
                          const char* blackboard_key, T& value) {
    if (node.getInput(port_name, value)) {
        return true;
    }
    if (!conf.blackboard) {
        return false;
    }
    return conf.blackboard->get(blackboard_key, value);
}

inline bool GetGlobalStartPose(BtContext& ctx,
                               commsgs::geometry_msgs::PoseStamped& pose) {
    if (!ctx.controller || !ctx.tf_buffer) {
        return false;
    }
    const std::string global_frame = ctx.options.global_frame().empty()
                                         ? "map"
                                         : ctx.options.global_frame();
    const std::string base_frame = ctx.options.robot_base_frame().empty()
                                       ? "base_link"
                                       : ctx.options.robot_base_frame();
    return utils::getGlobalRobotPose(pose, ctx.tf_buffer,
                                     ctx.controller->GetOdomSmoother(),
                                     global_frame, base_frame);
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

namespace BT {

template <typename T>
inline bool getInputPortOrBlackboard(const BT::TreeNode& bt_node,
                                     const BT::Blackboard& blackboard,
                                     const std::string& param_name, T& value) {
    if (bt_node.getInput<T>(param_name, value)) {
        return true;
    }
    if (blackboard.get<T>(param_name, value)) {
        return true;
    }
    return false;
}

#define getInputOrBlackboard(name, value) \
    getInputPortOrBlackboard(*this, *(this->config().blackboard), name, value)

}  // namespace BT

/** Place at the bottom of each BT plugin .cpp (BehaviorTree.CPP registration). */
#define REGISTER_BEHAVIOR_TREE_NODE(ClassName, ID)                          \
    BT_REGISTER_NODES(factory) {                                            \
        factory.registerNodeType<autonomy::tasks::behavior_tree::ClassName>( \
            ID);                                                            \
    }
