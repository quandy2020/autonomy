/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/task.hpp"

#include <utility>

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/navigator/bt_navigator.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/smoother_server.hpp"
#include "autonomy/tasks/constants.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace {

using TaskState = common::TaskInterface::TaskState;

template <typename NavigatorOptions>
bool NavigatorEnabled(const NavigatorOptions& config) {
    return !config.has_enable() || config.enable();
}


}  // namespace

Task::Task() : engine_(common::CreateBtNavigationEngine()) {}

Task::Task(const proto::TaskOptions& options)
    : options_(options), engine_(common::CreateBtNavigationEngine()) {}

void Task::SetNavigationEngine(common::NavigationEngine::SharedPtr engine) {
    std::lock_guard<std::mutex> lock(mutex_);
    engine_ = std::move(engine);
    if (!engine_) {
        engine_ = common::CreateBtNavigationEngine();
    }
    configured_ = false;
}

bool Task::Configure(
    const proto::TaskOptions& options,
    std::shared_ptr<planning::PlannerServer> planner,
    std::shared_ptr<planning::SmootherServer> smoother,
    std::shared_ptr<control::ControllerServer> controller,
    std::shared_ptr<transform::Buffer> tf_buffer) {
    options_ = options;
    return Configure(std::move(planner), std::move(smoother), std::move(controller),
                     std::move(tf_buffer));
}

bool Task::Configure(
    std::shared_ptr<planning::PlannerServer> planner,
    std::shared_ptr<planning::SmootherServer> smoother,
    std::shared_ptr<control::ControllerServer> controller,
    std::shared_ptr<transform::Buffer> tf_buffer) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!planner || !smoother || !controller || !tf_buffer) {
        AERROR << "Task::Configure: planner, smoother, controller, and tf_buffer "
                  "are required.";
        return false;
    }
    if (!engine_) {
        engine_ = common::CreateBtNavigationEngine();
    }
    planner_ = std::move(planner);
    smoother_ = std::move(smoother);
    controller_ = std::move(controller);
    tf_buffer_ = std::move(tf_buffer);
    if (!engine_->Configure(options_, planner_, smoother_, controller_,
                            tf_buffer_)) {
        AERROR << "Task::Configure: navigation engine failed to configure.";
        configured_ = false;
        return false;
    }
    configured_ = true;
    return true;
}

bool Task::Configure(
    const proto::TaskOptions& options,
    std::shared_ptr<planning::PlannerServer> planner,
    std::shared_ptr<planning::SmootherServer> smoother,
    std::shared_ptr<control::ControllerServer> controller,
    std::shared_ptr<transform::Buffer> tf_buffer,
    std::shared_ptr<autolink::Node> autolink_node) {
    if (!Configure(options, std::move(planner), std::move(smoother),
                   std::move(controller), std::move(tf_buffer))) {
        return false;
    }
    if (!autolink_node) {
        return true;
    }
    const bool enable_actions =
        !options_.has_enable_autolink_action_servers() ||
        options_.enable_autolink_action_servers();
    if (!enable_actions) {
        return true;
    }
    return AttachAutolinkNode(std::move(autolink_node));
}

bool Task::AttachAutolinkNode(std::shared_ptr<autolink::Node> node) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_ || !engine_) {
        AERROR << "Task::AttachAutolinkNode: Task not configured.";
        return false;
    }
    if (!engine_->AttachAutolinkNode(std::move(node))) {
        AERROR << "Task::AttachAutolinkNode: engine rejected node.";
        return false;
    }
    AINFO << "Task: autolink navigation action servers attached.";
    return true;
}

std::string Task::DefaultBtFileForMode(const common::NavigationMode mode) const {
    if (mode == common::NavigationMode::NAVIGATE_TO_POSE) {
        if (options_.has_navigate_to_pose() &&
            !options_.navigate_to_pose().default_behavior_tree_file().empty()) {
            return options_.navigate_to_pose().default_behavior_tree_file();
        }
        return "navigate_to_pose.xml";
    }
    if (mode == common::NavigationMode::NAVIGATE_THROUGH_POSES) {
        if (options_.has_navigate_through_poses() &&
            !options_.navigate_through_poses()
                 .default_behavior_tree_file()
                 .empty()) {
            return options_.navigate_through_poses().default_behavior_tree_file();
        }
        return "navigate_through_poses.xml";
    }
    return {};
}

bool Task::EnsureConfigured() {
    if (!configured_) {
        AERROR << "Task is not configured; call Configure() first.";
        return false;
    }
    return true;
}

bool Task::EnsureIdleForStart() {
    const auto state = state_.load();
    if (state == TaskState::SHUTDOWN) {
        AERROR << "Task is shut down.";
        return false;
    }
    if (state == TaskState::RUNNING || state == TaskState::PAUSED) {
        AWARN << "A navigation task is already active; cancel it before starting "
                 "another.";
        return false;
    }
    return true;
}

void Task::OnNavigationStarted(const common::NavigationMode mode) {
    navigation_mode_.store(mode);
    state_.store(TaskState::RUNNING);
}

void Task::OnNavigationCanceled() {
    navigation_mode_.store(common::NavigationMode::NONE);
    state_.store(TaskState::CANCELED);
}

bool Task::StartNavigateToPoseLocked(
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& behavior_tree_file) {
    if (!EnsureConfigured() || !EnsureIdleForStart()) {
        return false;
    }
    if (options_.has_navigate_to_pose() &&
        !NavigatorEnabled(options_.navigate_to_pose())) {
        AWARN << "navigate_to_pose navigator is disabled in TaskOptions.";
        return false;
    }
    std::string bt_file = behavior_tree_file;
    if (bt_file.empty()) {
        bt_file = DefaultBtFileForMode(common::NavigationMode::NAVIGATE_TO_POSE);
    }
    if (!engine_->StartNavigateToPose(goal, bt_file)) {
        return false;
    }
    OnNavigationStarted(common::NavigationMode::NAVIGATE_TO_POSE);
    return true;
}

bool Task::StartNavigateToPose(
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& behavior_tree_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    return StartNavigateToPoseLocked(goal, behavior_tree_file);
}

bool Task::CancelNavigateToPose() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (navigation_mode_.load() != common::NavigationMode::NAVIGATE_TO_POSE) {
        return false;
    }
    if (!engine_->Cancel()) {
        return false;
    }
    OnNavigationCanceled();
    return true;
}

bool Task::StartNavigateThroughPosesLocked(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    const std::string& behavior_tree_file) {
    if (!EnsureConfigured() || !EnsureIdleForStart()) {
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
        bt_file =
            DefaultBtFileForMode(common::NavigationMode::NAVIGATE_THROUGH_POSES);
    }
    if (!engine_->StartNavigateThroughPoses(goals, bt_file)) {
        return false;
    }
    OnNavigationStarted(common::NavigationMode::NAVIGATE_THROUGH_POSES);
    return true;
}

bool Task::StartNavigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    const std::string& behavior_tree_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    return StartNavigateThroughPosesLocked(goals, behavior_tree_file);
}

bool Task::CancelNavigateThroughPoses() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (navigation_mode_.load() !=
        common::NavigationMode::NAVIGATE_THROUGH_POSES) {
        return false;
    }
    if (!engine_->Cancel()) {
        return false;
    }
    OnNavigationCanceled();
    return true;
}

bool Task::PauseNavigation() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!EnsureConfigured()) {
        return false;
    }
    if (state_.load() != TaskState::RUNNING) {
        return false;
    }
    if (!engine_->Pause()) {
        return false;
    }
    state_.store(TaskState::PAUSED);
    return true;
}

bool Task::ResumeNavigation() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!EnsureConfigured()) {
        return false;
    }
    if (state_.load() != TaskState::PAUSED) {
        return false;
    }
    if (!engine_->Resume()) {
        return false;
    }
    state_.store(TaskState::RUNNING);
    return true;
}

common::NavigationMode Task::GetNavigationMode() const {
    return navigation_mode_.load();
}

bool Task::IsNavigating() const {
    const auto state = state_.load();
    return state == TaskState::RUNNING || state == TaskState::PAUSED;
}

bool Task::IsNavigationEngineActive() const {
    return engine_ && engine_->IsActive();
}

bool Task::LastNavigationSucceeded() const {
    const auto bt_engine =
        std::dynamic_pointer_cast<behavior_tree::BtNavigator>(engine_);
    if (!bt_engine) {
        return false;
    }
    return bt_engine->GetLastBtStatus() ==
           behavior_tree::BtStatus::SUCCEEDED;
}

void Task::FinalizeNavigation(const bool succeeded) {
    std::lock_guard<std::mutex> lock(mutex_);
    navigation_mode_.store(common::NavigationMode::NONE);
    state_.store(succeeded ? TaskState::COMPLETED : TaskState::FAILED);
}

bool Task::Resume() { return ResumeNavigation(); }

bool Task::Cancel() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!IsNavigating()) {
        return false;
    }
    if (!engine_->Cancel()) {
        return false;
    }
    OnNavigationCanceled();
    return true;
}

bool Task::Stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (IsNavigating()) {
        engine_->Cancel();
    }
    navigation_mode_.store(common::NavigationMode::NONE);
    state_.store(TaskState::STOPPED);
    return true;
}

void Task::Shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (IsNavigating()) {
        engine_->Cancel();
    }
    navigation_mode_.store(common::NavigationMode::NONE);
    state_.store(TaskState::SHUTDOWN);
    configured_ = false;
}

TaskState Task::GetState() const { return state_.load(); }

std::string Task::GetName() const { return kTaskNodeName; }

bool Task::StartImpl(std::vector<std::any>&& args) {
    if (args.empty()) {
        AERROR << "Task::Start: expected NavigateToPose goal or "
                  "NavigateThroughPoses goal list.";
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        if (args.size() == 1) {
            if (args[0].type() ==
                typeid(commsgs::geometry_msgs::PoseStamped)) {
                return StartNavigateToPoseLocked(
                    std::any_cast<commsgs::geometry_msgs::PoseStamped>(args[0]),
                    "");
            }
            if (args[0].type() ==
                typeid(std::vector<commsgs::geometry_msgs::PoseStamped>)) {
                return StartNavigateThroughPosesLocked(
                    std::any_cast<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                        args[0]),
                    "");
            }
        }
        if (args.size() == 2 &&
            args[0].type() == typeid(commsgs::geometry_msgs::PoseStamped) &&
            args[1].type() == typeid(std::string)) {
            return StartNavigateToPoseLocked(
                std::any_cast<commsgs::geometry_msgs::PoseStamped>(args[0]),
                std::any_cast<std::string>(args[1]));
        }
        if (args.size() == 2 &&
            args[0].type() ==
                typeid(std::vector<commsgs::geometry_msgs::PoseStamped>) &&
            args[1].type() == typeid(std::string)) {
            return StartNavigateThroughPosesLocked(
                std::any_cast<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                    args[0]),
                std::any_cast<std::string>(args[1]));
        }
    } catch (const std::bad_any_cast&) {
        AERROR << "Task::Start: unsupported argument types.";
        return false;
    }
    AERROR << "Task::Start: unsupported argument types.";
    return false;
}

}  // namespace tasks
}  // namespace autonomy
