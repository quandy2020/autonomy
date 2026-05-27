/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/tasks/behavior_tree/behavior_tree_context.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/behavior_tree/navigator/navigate_through_poses_navigator.hpp"
#include "autonomy/tasks/behavior_tree/navigator/navigate_to_pose_navigator.hpp"
#include "autonomy/tasks/common/navigation_engine.hpp"

namespace autolink {
class Node;
}

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class BtNavigator : public common::NavigationEngine
{
public:
    BtNavigator();
    ~BtNavigator() override;

    bool Configure(
        const proto::TaskOptions& options,
        std::shared_ptr<planning::PlannerServer> planner,
        std::shared_ptr<planning::SmootherServer> smoother,
        std::shared_ptr<control::ControllerServer> controller,
        std::shared_ptr<transform::Buffer> tf_buffer) override;

    bool StartNavigateToPose(
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& behavior_tree_file) override;

    bool StartNavigateThroughPoses(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
        const std::string& behavior_tree_file) override;

    bool Cancel() override;
    bool Pause() override;
    bool Resume() override;

    common::NavigationMode GetActiveMode() const override;
    bool IsActive() const override;

    /** Register autolink SimpleActionServer endpoints (Phase 3). */
    bool AttachAutolinkNode(std::shared_ptr<autolink::Node> node) override;

    std::shared_ptr<BehaviorTreeContext> GetContext() const { return ctx_; }
    BtStatus GetLastBtStatus() const { return last_bt_status_.load(); }

private:
    std::shared_ptr<BehaviorTreeContext> ctx_;
    std::shared_ptr<BehaviorTreeEngine> engine_;
    std::unique_ptr<NavigateToPoseNavigator> navigate_to_pose_;
    std::unique_ptr<NavigateThroughPosesNavigator> navigate_through_poses_;
    NavigatorInterface* active_navigator_{nullptr};

    mutable std::mutex mutex_;
    std::atomic<common::NavigationMode> active_mode_{common::NavigationMode::NONE};
    std::atomic<BtStatus> last_bt_status_{BtStatus::FAILED};
    bool configured_{false};

    std::unique_ptr<class NavigateToPoseActionServer> pose_action_server_;
    std::unique_ptr<class NavigateThroughPosesActionServer>
        through_poses_action_server_;
    bool autolink_attached_{false};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
