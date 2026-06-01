/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/navigators/bt_navigator.hpp"

#include "autolink/node/node.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/tasks/navigators/navigate_through_poses.hpp"
#include "autonomy/tasks/navigators/navigate_to_pose.hpp"

namespace autonomy {
namespace tasks {

namespace task_proto = behavior_tree::task_proto;

BtNavigator::BtNavigator(
    const proto::TaskOptions& options,
    std::shared_ptr<planning::PlannerServer> planner,
    std::shared_ptr<planning::SmootherServer> smoother,
    std::shared_ptr<control::ControllerServer> controller,
    std::shared_ptr<transform::Buffer> tf_buffer,
    std::shared_ptr<autolink::Node> node) {
    if (!planner || !smoother || !controller || !tf_buffer) {
        AERROR << "BtNavigator: missing dependencies.";
        return;
    }

    context_ = std::make_shared<behavior_tree::BtContext>();
    context_->planner = std::move(planner);
    context_->smoother = std::move(smoother);
    context_->controller = std::move(controller);
    context_->tf_buffer = std::move(tf_buffer);
    context_->options = options;

    context_->controller->SetNavigationContext(
        context_->tf_buffer, options.global_frame(), options.robot_base_frame());
    if (auto global_costmap = context_->planner->GetCostmapWrapper()) {
        context_->controller->SetSharedCostmap(global_costmap);
    }

    if (node) {
        context_->autolink_node = node;
        context_->controller->AttachAutolinkNode(node);
        context_->smoother->AttachAutolinkNode(node);
    }

    engine_ = std::make_shared<behavior_tree::BtEngine>(options);

    std::string pose_tree = "navigate_to_pose.xml";
    if (options.has_navigate_to_pose() &&
        !options.navigate_to_pose().behavior_tree_file().empty()) {
        pose_tree = options.navigate_to_pose().behavior_tree_file();
    }
    std::string through_tree = "navigate_through_poses.xml";
    if (options.has_navigate_through_poses() &&
        !options.navigate_through_poses().behavior_tree_file().empty()) {
        through_tree = options.navigate_through_poses().behavior_tree_file();
    }

    navigate_to_pose_ = std::make_unique<NavigateToPoseNavigator>(
        node, engine_, context_, &muxer_, pose_tree);
    navigate_through_poses_ = std::make_unique<NavigateThroughPosesNavigator>(
        node, engine_, context_, &muxer_, through_tree);

    if (!navigate_to_pose_->IsInitialized() ||
        !navigate_through_poses_->IsInitialized()) {
        AERROR << "BtNavigator: failed to initialize navigators.";
        navigate_to_pose_.reset();
        navigate_through_poses_.reset();
        context_.reset();
        engine_.reset();
        return;
    }

    auto on_done = [this](behavior_tree::RunStatus status) {
        last_run_status_.store(status);
    };
    navigate_to_pose_->SetCompletionHook(on_done);
    navigate_through_poses_->SetCompletionHook(on_done);

    if (node) {
        AINFO << "Autolink navigation actions active (navigate_to_pose, "
                 "navigate_through_poses).";
    }
}

BtNavigator::~BtNavigator() {
    std::lock_guard<std::mutex> lock(mutex_);
    Cancel();
}

bool BtNavigator::EnsureIdle() const {
    if (!context_ || !navigate_to_pose_ || !navigate_through_poses_) {
        AERROR << "BtNavigator is not initialized.";
        return false;
    }
    if (IsActive()) {
        AWARN << "Cancel active navigation before starting another.";
        return false;
    }
    return true;
}

bool BtNavigator::StartNavigateToPose(
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& behavior_tree_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!EnsureIdle()) {
        return false;
    }
    last_run_status_.store(behavior_tree::RunStatus::FAILED);

    task_proto::NavigateToPoseAction_Goal action_goal;
    *action_goal.mutable_pose() = commsgs::geometry_msgs::ToProto(goal);
    if (!behavior_tree_file.empty()) {
        action_goal.set_behavior_tree(behavior_tree_file);
    }
    auto goal_ptr = std::make_shared<const task_proto::NavigateToPoseAction_Goal>(
        std::move(action_goal));
    return navigate_to_pose_->StartWithGoal(goal_ptr, behavior_tree_file);
}

bool BtNavigator::StartNavigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    const std::string& behavior_tree_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goals.empty() || !EnsureIdle()) {
        return false;
    }
    last_run_status_.store(behavior_tree::RunStatus::FAILED);

    task_proto::NavigateThroughPosesAction_Goal action_goal;
    for (const auto& g : goals) {
        *action_goal.add_poses() = commsgs::geometry_msgs::ToProto(g);
    }
    if (!behavior_tree_file.empty()) {
        action_goal.set_behavior_tree(behavior_tree_file);
    }
    auto goal_ptr =
        std::make_shared<const task_proto::NavigateThroughPosesAction_Goal>(
            std::move(action_goal));
    return navigate_through_poses_->StartWithGoal(goal_ptr, behavior_tree_file);
}

bool BtNavigator::Cancel() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (context_) {
        context_->cancel_requested = true;
    }
    bool ok = true;
    if (navigate_to_pose_ && navigate_to_pose_->IsRunning()) {
        ok = navigate_to_pose_->Cancel();
    }
    if (navigate_through_poses_ && navigate_through_poses_->IsRunning()) {
        ok = navigate_through_poses_->Cancel() && ok;
    }
    return ok;
}

NavigationMode BtNavigator::GetActiveMode() const {
    if (navigate_to_pose_ && navigate_to_pose_->IsRunning()) {
        return NavigationMode::NAVIGATE_TO_POSE;
    }
    if (navigate_through_poses_ && navigate_through_poses_->IsRunning()) {
        return NavigationMode::NAVIGATE_THROUGH_POSES;
    }
    return NavigationMode::NONE;
}

bool BtNavigator::IsActive() const {
    return muxer_.IsNavigating() ||
           (navigate_to_pose_ && navigate_to_pose_->IsRunning()) ||
           (navigate_through_poses_ && navigate_through_poses_->IsRunning());
}

}  // namespace tasks
}  // namespace autonomy
