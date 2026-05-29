/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/task.hpp"

#include <utility>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace tasks {
namespace {

using TaskState = TaskInterface::TaskState;

template <typename NavigatorOptions>
bool NavigatorEnabled(const NavigatorOptions& config) {
    return !config.has_enable() || config.enable();
}

}  // namespace

Task::Task(const proto::TaskOptions& options,
           std::shared_ptr<planning::PlannerServer> planner,
           std::shared_ptr<planning::SmootherServer> smoother,
           std::shared_ptr<control::ControllerServer> controller,
           std::shared_ptr<transform::Buffer> tf_buffer,
           std::shared_ptr<autolink::Node> autolink_node)
    : options_(options) {
    if (!planner || !smoother || !controller || !tf_buffer) {
        AERROR << "Task: planner, smoother, controller, and tf_buffer are required.";
        return;
    }

    const bool enable_actions =
        !options_.has_enable_autolink_action_servers() ||
        options_.enable_autolink_action_servers();
    std::shared_ptr<autolink::Node> node;
    if (enable_actions && autolink_node) {
        node = std::move(autolink_node);
    }

    navigator_ = std::make_shared<BtNavigator>(
        options_, std::move(planner), std::move(smoother), std::move(controller),
        std::move(tf_buffer), node);
    if (!navigator_->GetContext()) {
        AERROR << "Task: BtNavigator failed to initialize.";
        navigator_.reset();
        return;
    }
    configured_ = true;
    if (node) {
        AINFO << "Task: autolink navigation action servers attached.";
    }
}

std::string Task::DefaultBtFileForMode(const NavigationMode mode) const {
    if (mode == NavigationMode::NAVIGATE_TO_POSE) {
        if (options_.has_navigate_to_pose() &&
            !options_.navigate_to_pose().behavior_tree_file().empty()) {
            return options_.navigate_to_pose().behavior_tree_file();
        }
        return "navigate_to_pose.xml";
    }
    if (mode == NavigationMode::NAVIGATE_THROUGH_POSES) {
        if (options_.has_navigate_through_poses() &&
            !options_.navigate_through_poses().behavior_tree_file().empty()) {
            return options_.navigate_through_poses().behavior_tree_file();
        }
        return "navigate_through_poses.xml";
    }
    return {};
}

void Task::OnNavigationStarted(const NavigationMode mode) {
    mode_.store(mode);
    state_.store(TaskState::kRunning);
}

void Task::OnNavigationCanceled() {
    mode_.store(NavigationMode::NONE);
    state_.store(TaskState::kCanceled);
}

bool Task::StartNavigateToPose(
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& behavior_tree_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
        AERROR << "Task is not configured.";
        return false;
    }
    const TaskState state = state_.load();
    if (state == TaskState::kShutdown) {
        AERROR << "Task is shut down.";
        return false;
    }
    if (state == TaskState::kRunning) {
        AWARN << "A navigation task is already active; cancel it before starting "
                 "another.";
        return false;
    }
    if (options_.has_navigate_to_pose() &&
        !NavigatorEnabled(options_.navigate_to_pose())) {
        AWARN << "navigate_to_pose navigator is disabled in TaskOptions.";
        return false;
    }
    std::string bt_file = behavior_tree_file;
    if (bt_file.empty()) {
        bt_file = DefaultBtFileForMode(NavigationMode::NAVIGATE_TO_POSE);
    }
    if (!navigator_->StartNavigateToPose(goal, bt_file)) {
        return false;
    }
    OnNavigationStarted(NavigationMode::NAVIGATE_TO_POSE);
    return true;
}

bool Task::StartNavigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    const std::string& behavior_tree_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
        AERROR << "Task is not configured.";
        return false;
    }
    const TaskState state = state_.load();
    if (state == TaskState::kShutdown) {
        AERROR << "Task is shut down.";
        return false;
    }
    if (state == TaskState::kRunning) {
        AWARN << "A navigation task is already active; cancel it before starting "
                 "another.";
        return false;
    }
    if (goals.empty()) {
        AERROR << "StartNavigateThroughPoses: goals must not be empty.";
        return false;
    }
    if (options_.has_navigate_through_poses() &&
        !NavigatorEnabled(options_.navigate_through_poses())) {
        AWARN << "navigate_through_poses navigator is disabled in TaskOptions.";
        return false;
    }
    std::string bt_file = behavior_tree_file;
    if (bt_file.empty()) {
        bt_file = DefaultBtFileForMode(NavigationMode::NAVIGATE_THROUGH_POSES);
    }
    if (!navigator_->StartNavigateThroughPoses(goals, bt_file)) {
        return false;
    }
    OnNavigationStarted(NavigationMode::NAVIGATE_THROUGH_POSES);
    return true;
}

NavigationMode Task::GetNavigationMode() const {
    return mode_.load();
}

bool Task::IsNavigating() const {
    return state_.load() == TaskState::kRunning;
}

bool Task::IsNavigatorActive() const {
    return navigator_ && navigator_->IsActive();
}

void Task::SetPathCallback(
    std::function<void(const commsgs::planning_msgs::Path&)> callback) {
    if (!navigator_) {
        return;
    }
    if (auto ctx = navigator_->GetContext()) {
        ctx->on_path = std::move(callback);
    }
}

bool Task::LastNavigationSucceeded() const {
    if (!navigator_) {
        return false;
    }
    return navigator_->GetLastRunStatus() ==
           behavior_tree::RunStatus::SUCCEEDED;
}

void Task::FinalizeNavigation(const bool succeeded) {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_.store(NavigationMode::NONE);
    state_.store(succeeded ? TaskState::kCompleted : TaskState::kFailed);
}

bool Task::Cancel() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!IsNavigating()) {
        return false;
    }
    if (!navigator_->Cancel()) {
        return false;
    }
    OnNavigationCanceled();
    return true;
}

void Task::Shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (IsNavigating()) {
        navigator_->Cancel();
    }
    mode_.store(NavigationMode::NONE);
    state_.store(TaskState::kShutdown);
    configured_ = false;
}

TaskState Task::GetState() const { return state_.load(); }

}  // namespace tasks
}  // namespace autonomy
