/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/navigator/bt_navigator.hpp"

#include "autolink/node/node.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/action/navigate_through_poses_action_server.hpp"
#include "autonomy/tasks/behavior_tree/action/navigate_to_pose_action_server.hpp"
#include "autonomy/tasks/constants.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

BtNavigator::BtNavigator() = default;

BtNavigator::~BtNavigator() {
    if (pose_action_server_) {
        pose_action_server_->Deactivate();
    }
    if (through_poses_action_server_) {
        through_poses_action_server_->Deactivate();
    }
    Cancel();
}

bool BtNavigator::Configure(
    const proto::TaskOptions& options,
    std::shared_ptr<planning::PlannerServer> planner,
    std::shared_ptr<planning::SmootherServer> smoother,
    std::shared_ptr<control::ControllerServer> controller,
    std::shared_ptr<transform::Buffer> tf_buffer) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!planner || !smoother || !controller || !tf_buffer) {
        AERROR << "BtNavigator::Configure: missing server dependencies.";
        return false;
    }

    ctx_ = std::make_shared<BehaviorTreeContext>();
    ctx_->planner = std::move(planner);
    ctx_->smoother = std::move(smoother);
    ctx_->controller = std::move(controller);
    ctx_->tf_buffer = std::move(tf_buffer);
    ctx_->options = options;

    ctx_->controller->SetNavigationContext(
        ctx_->tf_buffer, options.global_frame(), options.robot_base_frame());
    if (auto global_costmap = ctx_->planner->GetCostmapWrapper()) {
        ctx_->controller->SetSharedCostmap(global_costmap);
    }

    engine_ = std::make_shared<BehaviorTreeEngine>(options);

    navigate_to_pose_ = std::make_unique<NavigateToPoseNavigator>();
    navigate_through_poses_ =
        std::make_unique<NavigateThroughPosesNavigator>();

    std::string pose_bt_file = "navigate_to_pose.xml";
    if (options.has_navigate_to_pose() &&
        !options.navigate_to_pose().default_behavior_tree_file().empty()) {
        pose_bt_file = options.navigate_to_pose().default_behavior_tree_file();
    }
    std::string through_poses_bt_file = "navigate_through_poses.xml";
    if (options.has_navigate_through_poses() &&
        !options.navigate_through_poses()
             .default_behavior_tree_file()
             .empty()) {
        through_poses_bt_file =
            options.navigate_through_poses().default_behavior_tree_file();
    }

    if (!navigate_to_pose_->Configure(engine_, ctx_, pose_bt_file) ||
        !navigate_through_poses_->Configure(engine_, ctx_,
                                            through_poses_bt_file)) {
        AERROR << "BtNavigator::Configure: navigator setup failed.";
        configured_ = false;
        return false;
    }

    auto completion_hook = [this](BtStatus status) {
        last_bt_status_.store(status);
        std::lock_guard<std::mutex> lock(mutex_);
        active_navigator_ = nullptr;
        active_mode_ = common::NavigationMode::NONE;
    };
    navigate_to_pose_->SetCompletionHook(completion_hook);
    navigate_through_poses_->SetCompletionHook(completion_hook);

    configured_ = true;
    return true;
}

bool BtNavigator::StartNavigateToPose(
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& behavior_tree_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
        AERROR << "BtNavigator not configured.";
        return false;
    }
    if (IsActive()) {
        AWARN << "BtNavigator: cancel active navigation before starting new.";
        return false;
    }
    navigate_to_pose_->SetGoal(goal);
    active_navigator_ = navigate_to_pose_.get();
    active_mode_ = common::NavigationMode::NAVIGATE_TO_POSE;
    last_bt_status_.store(BtStatus::FAILED);
    if (!navigate_to_pose_->Start(behavior_tree_file)) {
        active_navigator_ = nullptr;
        active_mode_ = common::NavigationMode::NONE;
        return false;
    }
    return true;
}

bool BtNavigator::StartNavigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    const std::string& behavior_tree_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
        return false;
    }
    if (IsActive()) {
        return false;
    }
    if (goals.empty()) {
        return false;
    }
    navigate_through_poses_->SetGoals(goals);
    active_navigator_ = navigate_through_poses_.get();
    active_mode_ = common::NavigationMode::NAVIGATE_THROUGH_POSES;
    last_bt_status_.store(BtStatus::FAILED);
    if (!navigate_through_poses_->Start(behavior_tree_file)) {
        active_navigator_ = nullptr;
        active_mode_ = common::NavigationMode::NONE;
        return false;
    }
    return true;
}

bool BtNavigator::Cancel() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ctx_) {
        ctx_->cancel_requested = true;
    }
    bool ok = true;
    if (navigate_to_pose_->IsRunning()) {
        ok = navigate_to_pose_->Cancel() && ok;
    }
    if (navigate_through_poses_->IsRunning()) {
        ok = navigate_through_poses_->Cancel() && ok;
    }
    active_navigator_ = nullptr;
    active_mode_ = common::NavigationMode::NONE;
    return ok;
}

bool BtNavigator::Pause() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ctx_) {
        ctx_->pause_requested = true;
    }
    if (active_navigator_) {
        return active_navigator_->Pause();
    }
    return false;
}

bool BtNavigator::Resume() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ctx_) {
        ctx_->pause_requested = false;
    }
    if (active_navigator_) {
        return active_navigator_->Resume();
    }
    return false;
}

common::NavigationMode BtNavigator::GetActiveMode() const {
    if (!IsActive()) {
        return common::NavigationMode::NONE;
    }
    return active_mode_.load();
}

bool BtNavigator::IsActive() const {
    return navigate_to_pose_->IsRunning() ||
           navigate_through_poses_->IsRunning();
}

bool BtNavigator::AttachAutolinkNode(std::shared_ptr<autolink::Node> node) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
        AERROR << "BtNavigator::AttachAutolinkNode: not configured.";
        return false;
    }
    if (!node) {
        AERROR << "BtNavigator::AttachAutolinkNode: null node.";
        return false;
    }
    if (autolink_attached_) {
        return true;
    }
    pose_action_server_ = std::make_unique<NavigateToPoseActionServer>(this);
    through_poses_action_server_ =
        std::make_unique<NavigateThroughPosesActionServer>(this);
    if (!pose_action_server_->Activate(node) ||
        !through_poses_action_server_->Activate(node)) {
        pose_action_server_.reset();
        through_poses_action_server_.reset();
        AERROR << "BtNavigator::AttachAutolinkNode: failed to activate servers.";
        return false;
    }
    autolink_attached_ = true;
    return true;
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
