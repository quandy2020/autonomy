/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/smoother_server.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace scheduler {

/** Shared planner / controller / context owned by tasks::Task. */
struct SharedSystem {
    std::shared_ptr<planning::PlannerServer> planner;
    std::shared_ptr<planning::SmootherServer> smoother;
    std::shared_ptr<control::ControllerServer> controller;
    std::shared_ptr<common::TaskContext> task_context;
};

class TaskScheduler
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskScheduler)

    TaskScheduler() = default;

    /** Creates and owns planner / smoother / controller (standalone tools). */
    void Initialize(const std::string& configuration_directory);

    /**
     * @brief Registers BT navigators using existing servers (tasks::Task / ROS facade).
     * Does not Start/Shutdown planner or controller.
     */
    void InitializeAttached(const std::string& configuration_directory,
                            const SharedSystem& system);

    void Shutdown();

    bool IsInitialized() const { return initialized_; }

    /** True when Initialize() created servers; false when InitializeAttached(). */
    bool OwnsServers() const { return owns_servers_; }

    /** @brief True if @p id was registered at init (config enable + factory). */
    bool HasNavigator(const std::string& id) const;

    /** @brief True if @p id appears in tasks.lua navigators list with enable=true. */
    bool IsNavigatorConfigured(const std::string& id) const;

    /** @brief Ids successfully registered in @ref SetupNavigators(). */
    std::vector<std::string> RegisteredNavigatorIds() const;

    // ---- A→B 单点导航 ----
    behavior_tree::BtStatus NavigateToPose(
        std::shared_ptr<const behavior_tree::proto::NavigateToPoseAction::Goal>
            goal);

    // ---- 多点 / 途经点导航 ----
    behavior_tree::BtStatus NavigateThroughPoses(
        std::shared_ptr<const behavior_tree::proto::NavigateThroughPosesAction::Goal>
            goal);

    behavior_tree::BtStatus NavigateThroughPoses(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& poses,
        const std::string& behavior_tree = "");

    // ---- 自动回充（导航至 dock 位姿） ----
    behavior_tree::BtStatus NavigateToDock(
        std::shared_ptr<const behavior_tree::proto::DockRobotAction::Goal> goal);

    behavior_tree::BtStatus NavigateToDockPose(
        const commsgs::geometry_msgs::PoseStamped& dock_pose,
        const std::string& dock_type = "default",
        const std::string& dock_id = "");

    // ---- 人体 / 目标跟随 ----
    behavior_tree::BtStatus TrackToTarget(
        std::shared_ptr<const behavior_tree::proto::TrackToTargetAction::Goal>
            goal);

    behavior_tree::BtStatus TrackToTarget(uint32_t target_id);

    /** @brief 运行中更新黑板 {target_pose}（跟踪模块周期调用）。 */
    bool UpdateTrackTargetPose(
        const commsgs::geometry_msgs::PoseStamped& target_pose);

    // ---- 环境探索 ----
    behavior_tree::BtStatus ExploreToAnywhere(
        std::shared_ptr<const behavior_tree::proto::ExploreToAnywhereAction::Goal>
            goal);

    behavior_tree::BtStatus ExploreToAnywhere(double time_allowance_sec = 0.0);

    /** @brief 运行中更新黑板 {explore_goal}（探索模块周期调用）。 */
    bool UpdateExploreGoal(
        const commsgs::geometry_msgs::PoseStamped& explore_goal);

    // ---- 遥操控制 ----
    behavior_tree::BtStatus TeleopDrive(
        std::shared_ptr<const behavior_tree::proto::AssistedTeleopAction::Goal>
            goal,
        double max_linear_vel = 0.0,
        double max_angular_vel = 0.0,
        std::function<bool()> cancel_checker = nullptr);

    behavior_tree::BtStatus TeleopDrive(
        double time_allowance_sec = 0.0,
        double max_linear_vel = 0.0,
        double max_angular_vel = 0.0,
        std::function<bool()> cancel_checker = nullptr);

    bool UpdateTeleopCommand(
        const commsgs::geometry_msgs::TwistStamped& cmd);

    void BeginTeleopSession(
        double max_linear_vel = 0.0, double max_angular_vel = 0.0);

    void EndTeleopSession();

    bool IsTeleopActive() const;

    void RequestCancel();

    std::shared_ptr<common::TaskContext> TaskContext() const {
        return task_context_;
    }

    template <typename NavigatorT>
    std::shared_ptr<NavigatorT> GetNavigator(const std::string& id) const {
        auto it = navigators_.find(id);
        if (it == navigators_.end()) {
            return nullptr;
        }
        return std::dynamic_pointer_cast<NavigatorT>(it->second);
    }

private:
    using CancelFn = std::function<bool()>;

    behavior_tree::BtStatus RunNavigator(
        const std::string& id,
        const std::function<behavior_tree::BtStatus(CancelFn)>& run);

    void SetupNavigators();

    std::string configuration_directory_;
    proto::TaskOptions task_options_;
    std::shared_ptr<common::TaskContext> task_context_;
    std::shared_ptr<planning::PlannerServer> planner_;
    std::shared_ptr<planning::SmootherServer> smoother_;
    std::shared_ptr<control::ControllerServer> controller_;
    std::shared_ptr<control::utils::OdomSmoother> odom_smoother_;
    std::unordered_map<std::string, std::shared_ptr<common::NavigatorBase>>
        navigators_;
    common::NavigatorMuxer muxer_;
    void ApplyTaskOptionsToContext();

    std::atomic<bool> cancel_requested_{false};
    bool initialized_{false};
    bool owns_servers_{false};
};

}  // namespace scheduler
}  // namespace tasks
}  // namespace autonomy
