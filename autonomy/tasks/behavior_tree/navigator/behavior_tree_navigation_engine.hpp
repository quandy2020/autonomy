/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/tasks/behavior_tree/context.hpp"
#include "autonomy/tasks/behavior_tree/engine.hpp"
#include "autonomy/tasks/behavior_tree/navigator/navigate_through_poses.hpp"
#include "autonomy/tasks/behavior_tree/navigator/navigate_to_pose.hpp"
#include "autonomy/tasks/common/navigation.hpp"

namespace autolink {
class Node;
}

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief NavigationEngine implementation: shared BT runtime + mission routing.
 */
class BehaviorTreeNavigationEngine : public NavigationEngine
{
public:
    BehaviorTreeNavigationEngine();
    ~BehaviorTreeNavigationEngine() override;

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

    NavigationMode GetActiveMode() const override;
    bool IsActive() const override;

    bool AttachAutolinkNode(std::shared_ptr<autolink::Node> node) override;

    std::shared_ptr<BehaviorTreeContext> GetContext() const { return ctx_; }
    RunStatus GetLastRunStatus() const { return last_run_status_.load(); }

private:
    bool StartNavigator(Navigator& navigator, const std::string& bt_xml_file,
                        NavigationMode mode);

    std::shared_ptr<BehaviorTreeContext> ctx_;
    std::shared_ptr<BehaviorTreeEngine> engine_;
    std::unique_ptr<NavigateToPoseNavigator> navigate_to_pose_;
    std::unique_ptr<NavigateThroughPosesNavigator> navigate_through_poses_;
    Navigator* active_navigator_{nullptr};

    mutable std::mutex mutex_;
    std::atomic<NavigationMode> active_mode_{NavigationMode::NONE};
    std::atomic<RunStatus> last_run_status_{RunStatus::FAILED};
    bool configured_{false};

    std::unique_ptr<class NavigateToPoseServer> navigate_to_pose_server_;
    std::unique_ptr<class NavigateThroughPosesServer>
        navigate_through_poses_server_;
    bool autolink_attached_{false};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
