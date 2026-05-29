/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_context.hpp"
#include "autonomy/tasks/behavior_tree/bt_engine.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autolink {
class Node;
}

namespace autonomy {
namespace control {
class ControllerServer;
}
namespace planning {
class PlannerServer;
class SmootherServer;
}
namespace transform {
class Buffer;
}
namespace tasks {

class NavigateToPoseNavigator;
class NavigateThroughPosesNavigator;

/**
 * @brief Top-level BT navigation orchestrator (nav2_bt_navigator::BtNavigator).
 */
class BtNavigator
{
public:
    BtNavigator(
        const proto::TaskOptions& options,
        std::shared_ptr<planning::PlannerServer> planner,
        std::shared_ptr<planning::SmootherServer> smoother,
        std::shared_ptr<control::ControllerServer> controller,
        std::shared_ptr<transform::Buffer> tf_buffer,
        std::shared_ptr<autolink::Node> node = nullptr);
    ~BtNavigator();

    bool StartNavigateToPose(
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& behavior_tree_file);

    bool StartNavigateThroughPoses(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
        const std::string& behavior_tree_file);

    bool Cancel();

    NavigationMode GetActiveMode() const;
    bool IsActive() const;

    std::shared_ptr<behavior_tree::BtContext> GetContext() const {
        return context_;
    }
    behavior_tree::RunStatus GetLastRunStatus() const {
        return last_run_status_.load();
    }

private:
    bool EnsureIdle() const;

    std::shared_ptr<behavior_tree::BtContext> context_;
    std::shared_ptr<behavior_tree::BtEngine> engine_;
    NavigatorMuxer muxer_;
    std::unique_ptr<NavigateToPoseNavigator> navigate_to_pose_;
    std::unique_ptr<NavigateThroughPosesNavigator> navigate_through_poses_;

    mutable std::mutex mutex_;
    std::atomic<behavior_tree::RunStatus> last_run_status_{
        behavior_tree::RunStatus::FAILED};
};

}  // namespace tasks
}  // namespace autonomy
