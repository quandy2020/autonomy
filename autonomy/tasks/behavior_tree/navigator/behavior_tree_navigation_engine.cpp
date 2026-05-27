/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/navigator/behavior_tree_navigation_engine.hpp"

#include "autolink/node/node.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/servers/navigate_through_poses_server.hpp"
#include "autonomy/tasks/behavior_tree/servers/navigate_to_pose_server.hpp"
#include "autonomy/tasks/constants.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

BehaviorTreeNavigationEngine::BehaviorTreeNavigationEngine() = default;

BehaviorTreeNavigationEngine::~BehaviorTreeNavigationEngine() {
    if (navigate_to_pose_server_) {
        navigate_to_pose_server_->Deactivate();
    }
    if (navigate_through_poses_server_) {
        navigate_through_poses_server_->Deactivate();
    }
    Cancel();
}

bool BehaviorTreeNavigationEngine::Configure(
    const proto::TaskOptions& options,
    std::shared_ptr<planning::PlannerServer> planner,
    std::shared_ptr<planning::SmootherServer> smoother,
    std::shared_ptr<control::ControllerServer> controller,
    std::shared_ptr<transform::Buffer> tf_buffer) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!planner || !smoother || !controller || !tf_buffer) {
        AERROR << "BehaviorTreeNavigationEngine::Configure: missing server dependencies.";
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
        !options.navigate_to_pose().behavior_tree_file().empty()) {
        pose_bt_file = options.navigate_to_pose().behavior_tree_file();
    }
    std::string through_poses_bt_file = "navigate_through_poses.xml";
    if (options.has_navigate_through_poses() &&
        !options.navigate_through_poses().behavior_tree_file().empty()) {
        through_poses_bt_file =
            options.navigate_through_poses().behavior_tree_file();
    }

    if (!navigate_to_pose_->Configure(engine_, ctx_, pose_bt_file) ||
        !navigate_through_poses_->Configure(engine_, ctx_,
                                            through_poses_bt_file)) {
        AERROR << "BehaviorTreeNavigationEngine::Configure: navigator setup failed.";
        configured_ = false;
        return false;
    }

    auto completion_hook = [this](RunStatus status) {
        last_run_status_.store(status);
        std::lock_guard<std::mutex> lock(mutex_);
        active_navigator_ = nullptr;
        active_mode_ = NavigationMode::NONE;
    };
    navigate_to_pose_->SetCompletionHook(completion_hook);
    navigate_through_poses_->SetCompletionHook(completion_hook);

    configured_ = true;
    return true;
}

bool BehaviorTreeNavigationEngine::StartNavigator(Navigator& navigator,
                                        const std::string& bt_xml_file,
                                        NavigationMode mode) {
    if (!configured_) {
        AERROR << "BehaviorTreeNavigationEngine not configured.";
        return false;
    }
    if (IsActive()) {
        AWARN << "BehaviorTreeNavigationEngine: cancel active navigation before starting "
                 "new.";
        return false;
    }
    active_navigator_ = &navigator;
    active_mode_ = mode;
    last_run_status_.store(RunStatus::FAILED);
    if (!navigator.Start(bt_xml_file)) {
        active_navigator_ = nullptr;
        active_mode_ = NavigationMode::NONE;
        return false;
    }
    return true;
}

bool BehaviorTreeNavigationEngine::StartNavigateToPose(
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& behavior_tree_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    navigate_to_pose_->SetGoal(goal);
    return StartNavigator(*navigate_to_pose_, behavior_tree_file,
                          NavigationMode::NAVIGATE_TO_POSE);
}

bool BehaviorTreeNavigationEngine::StartNavigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    const std::string& behavior_tree_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goals.empty()) {
        return false;
    }
    navigate_through_poses_->SetGoals(goals);
    return StartNavigator(*navigate_through_poses_, behavior_tree_file,
                          NavigationMode::NAVIGATE_THROUGH_POSES);
}

bool BehaviorTreeNavigationEngine::Cancel() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ctx_) {
        ctx_->cancel_requested = true;
    }
    bool ok = true;
    if (active_navigator_) {
        ok = active_navigator_->Cancel();
    }
    active_navigator_ = nullptr;
    active_mode_ = NavigationMode::NONE;
    return ok;
}

bool BehaviorTreeNavigationEngine::Pause() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_navigator_) {
        return false;
    }
    return active_navigator_->Pause();
}

bool BehaviorTreeNavigationEngine::Resume() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_navigator_) {
        return false;
    }
    return active_navigator_->Resume();
}

NavigationMode BehaviorTreeNavigationEngine::GetActiveMode() const {
    if (!IsActive()) {
        return NavigationMode::NONE;
    }
    return active_mode_.load();
}

bool BehaviorTreeNavigationEngine::IsActive() const {
    return active_navigator_ != nullptr && active_navigator_->IsRunning();
}

bool BehaviorTreeNavigationEngine::AttachAutolinkNode(
    std::shared_ptr<autolink::Node> node) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
        AERROR << "BehaviorTreeNavigationEngine::AttachAutolinkNode: not configured.";
        return false;
    }
    if (!node) {
        AERROR << "BehaviorTreeNavigationEngine::AttachAutolinkNode: null node.";
        return false;
    }
    if (autolink_attached_) {
        return true;
    }
    navigate_to_pose_server_ = std::make_unique<NavigateToPoseServer>(this);
    navigate_through_poses_server_ =
        std::make_unique<NavigateThroughPosesServer>(this);
    if (!navigate_to_pose_server_->Activate(node) ||
        !navigate_through_poses_server_->Activate(node)) {
        navigate_to_pose_server_.reset();
        navigate_through_poses_server_.reset();
        AERROR << "BehaviorTreeNavigationEngine::AttachAutolinkNode: failed to activate "
                  "servers.";
        return false;
    }
    autolink_attached_ = true;
    return true;
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
