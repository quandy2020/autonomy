/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/system/autonomy.hpp"

#include <thread>

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/smoother/smoother_server.hpp"
#include "autonomy/sensor/internal/sensor_collator.hpp"
#include "autonomy/tasks/constants.hpp"
#include "autonomy/tasks/options.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/transform_server.hpp"

namespace autonomy {
namespace system {
namespace {

commsgs::geometry_msgs::TwistStamped ZeroTwist() {
    commsgs::geometry_msgs::TwistStamped cmd;
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.linear.z = 0.0;
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
}

std::string PickId(const std::string& runtime_id,
                   const std::string& default_id) {
    return runtime_id.empty() ? default_id : runtime_id;
}

}  // namespace

Autonomy::Autonomy(proto::AutonomyOptions options)
    : options_(std::move(options)) {}

Autonomy::~Autonomy() { Shutdown(); }

bool Autonomy::EnsureStarted() const {
    if (!started_.load()) {
        AERROR << "Autonomy: call Start() before using the core.";
        return false;
    }
    return true;
}

Autonomy::PluginIds Autonomy::ResolvePluginIds() const {
    return {
        PickId(runtime_.planner_id, task_options_.default_planner_id()),
        PickId(runtime_.controller_id, task_options_.default_controller_id()),
        PickId(runtime_.goal_checker_id,
               task_options_.default_goal_checker_id()),
    };
}

void Autonomy::SyncNavigationFrames(const std::string& global_frame,
                                    const std::string& robot_base_frame) {
    if (!planner_) {
        return;
    }
    auto costmap = planner_->GetCostmapWrapper();
    if (!costmap) {
        return;
    }
    if (!global_frame.empty()) {
        costmap->setGlobalFrameID(global_frame);
    }
    if (!robot_base_frame.empty()) {
        costmap->setRobotBaseFrameID(robot_base_frame);
    }
}

void Autonomy::ApplyRuntimeToTaskOptions(
    const tasks::RuntimeOptions& runtime) {
    if (!runtime.global_frame.empty()) {
        task_options_.set_global_frame(runtime.global_frame);
    }
    if (!runtime.planner_id.empty()) {
        task_options_.set_default_planner_id(runtime.planner_id);
    }
    if (!runtime.controller_id.empty()) {
        task_options_.set_default_controller_id(runtime.controller_id);
    }
    if (!runtime.goal_checker_id.empty()) {
        task_options_.set_default_goal_checker_id(runtime.goal_checker_id);
    }
    if (!runtime.robot_base_frame.empty()) {
        task_options_.set_robot_base_frame(runtime.robot_base_frame);
    }
    if (runtime.goal_tolerance > 0.0) {
        task_options_.set_goal_reached_tolerance(runtime.goal_tolerance);
    }
}

void Autonomy::Start() {
    if (started_.exchange(true)) {
        return;
    }

    if (options_.has_map_options()) {
        map_server_ = std::make_shared<map::MapServer>(options_.map_options());
        map_server_->Start();
    }

    if (options_.has_planner_options()) {
        planner_ =
            std::make_shared<planning::PlannerServer>(options_.planner_options());
    }

    if (planner_) {
        smoother_ = std::make_shared<planning::SmootherServer>(
            options_.planner_options(), planner_->GetCostmapWrapper());
    }

    if (options_.has_controller_options()) {
        controller_ = std::make_shared<control::ControllerServer>(
            options_.controller_options());
        controller_->Start();
        if (planner_) {
            controller_->SetSharedCostmap(planner_->GetCostmapWrapper());
        }
    }

    tf_buffer_ = std::shared_ptr<transform::Buffer>(transform::Buffer::Instance(),
                                                    [](transform::Buffer*) {});
    if (tf_buffer_->Init() != 0) {
        AWARN << "Autonomy: transform::Buffer::Init returned non-zero.";
    }

    if (options_.has_transform_options()) {
        transform_server_ = std::make_unique<transform::TransformServer>(
            options_.transform_options());
        if (transform_server_->Initialize()) {
            transform_server_->Start();
            const auto& static_transforms =
                transform_server_->GetTransformStampedsData();
            for (const auto& trans : static_transforms.transforms) {
                try {
                    geometry_msgs::TransformStamped geo_msg;
                    geo_msg.header.stamp =
                        static_cast<uint64_t>(trans.header.stamp.sec) *
                            1000000000ULL +
                        static_cast<uint64_t>(trans.header.stamp.nanosec);
                    geo_msg.header.frame_id = trans.header.frame_id;
                    geo_msg.child_frame_id = trans.child_frame_id;
                    geo_msg.transform.translation.x =
                        trans.transform.translation.x;
                    geo_msg.transform.translation.y =
                        trans.transform.translation.y;
                    geo_msg.transform.translation.z =
                        trans.transform.translation.z;
                    geo_msg.transform.rotation.x = trans.transform.rotation.x;
                    geo_msg.transform.rotation.y = trans.transform.rotation.y;
                    geo_msg.transform.rotation.z = trans.transform.rotation.z;
                    geo_msg.transform.rotation.w = trans.transform.rotation.w;
                    tf_buffer_->setTransform(geo_msg, "autonomy_system", true);
                } catch (const std::exception& ex) {
                    AWARN << "Failed to load static transform: " << ex.what();
                }
            }
        }
    }

    if (map_server_ && planner_) {
        map_server_->SetMapPublishCallback(
            [this](const commsgs::map_msgs::OccupancyGrid::SharedPtr& map) {
                ApplyMapToCostmap(map);
                std::vector<MapPublishListener> listeners;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    listeners = map_listeners_;
                }
                for (const auto& listener : listeners) {
                    if (listener) {
                        listener(map);
                    }
                }
            });
        if (auto map = map_server_->GetStaticMapShared()) {
            ApplyMapToCostmap(map);
        }
    }

    sensor::SensorConsumer consumer;
    consumer.controller = controller_.get();
    consumer.on_laser_scan = [this](const sensor::LaserScanData& scan) {
        if (auto costmap = GetGlobalCostmap()) {
            costmap->feedLaserScan(scan);
        }
    };
    sensor_collator_ = std::make_unique<sensor::SensorCollator>(consumer);

    AINFO << "Autonomy started (map/planner/controller skeleton).";
}

void Autonomy::Configure(const tasks::RuntimeOptions& runtime) {
    if (!EnsureStarted()) {
        return;
    }
    runtime_ = runtime;
    use_bt_navigation_ = runtime.use_bt_navigation;

    if (options_.has_task_options()) {
        task_options_ = options_.task_options();
    } else if (!runtime.config_directory.empty()) {
        task_options_ = tasks::CreateOptions(runtime.config_directory,
                                             "tasks/tasks.lua");
    } else {
        AWARN << "Autonomy::Configure: no task_options in AutonomyOptions "
                 "and config_directory is empty.";
    }

    ApplyRuntimeToTaskOptions(runtime);
    SyncNavigationFrames(runtime.global_frame, runtime.robot_base_frame);

    if (!planner_ || !smoother_ || !controller_ || !tf_buffer_) {
        AERROR << "Autonomy::Configure: missing server dependencies.";
        configured_ = false;
        return;
    }

    if (!runtime.enable_bt_tasks) {
        AWARN << "Autonomy: enable_bt_tasks=false; Task will not be "
                 "configured.";
        configured_ = false;
        return;
    }

    const bool enable_autolink =
        !task_options_.has_enable_autolink_action_servers() ||
        task_options_.enable_autolink_action_servers();

    if (enable_autolink && !autolink_node_) {
        autolink_node_ = autolink::CreateNode(tasks::kTaskNodeName);
        if (!autolink_node_) {
            AWARN << "Autonomy::Configure: autolink::CreateNode failed "
                     "(call autolink::Init first). Using in-process BT only.";
        } else {
            AINFO << "Autonomy: autolink node '" << tasks::kTaskNodeName
                  << "' created.";
        }
    }

    std::shared_ptr<autolink::Node> task_node;
    if (enable_autolink && autolink_node_) {
        task_node = autolink_node_;
    }
    task_ = std::make_unique<tasks::Task>(
        task_options_, planner_, smoother_, controller_, tf_buffer_, task_node);
    if (!task_->IsConfigured()) {
        AERROR << "Autonomy::Configure: Task initialization failed.";
        task_.reset();
        configured_ = false;
        return;
    }
    const auto publish_path =
        [this](const commsgs::planning_msgs::Path& path) { NotifyPath(path); };
    if (planner_) {
        planner_->SetPathUpdateCallback(publish_path);
    }
    if (smoother_) {
        smoother_->SetPathUpdateCallback(publish_path);
    }
    task_->SetPathCallback(publish_path);

    configured_ = true;
    AINFO << "Autonomy: Task configured (BT navigation="
          << (use_bt_navigation_ ? "on" : "off")
          << ", autolink=" << (autolink_node_ ? "on" : "off") << ").";
}

void Autonomy::Shutdown() {
    if (!started_.exchange(false)) {
        return;
    }
    RequestCancelNavigation();
    if (task_) {
        task_->Shutdown();
    }
    if (controller_) {
        controller_->Shutdown();
    }
    if (map_server_) {
        map_server_->Shutdown();
    }
    if (transform_server_) {
        transform_server_->Stop();
    }
    configured_ = false;
    map_server_.reset();
    planner_.reset();
    smoother_.reset();
    controller_.reset();
    task_.reset();
    autolink_node_.reset();
    sensor_collator_.reset();
    transform_server_.reset();
    AINFO << "Autonomy shut down.";
}

bool Autonomy::IsReady() const {
    return configured_.load() && task_ &&
           task_->GetState() != tasks::TaskInterface::TaskState::kShutdown;
}

map::MapServer* Autonomy::GetMapServer() { return map_server_.get(); }

map::costmap_2d::Costmap2DWrapper::SharedPtr Autonomy::GetGlobalCostmap() {
    if (!planner_) {
        return nullptr;
    }
    return planner_->GetCostmapWrapper();
}

sensor::SensorCollator& Autonomy::GetSensorCollator() {
    return *sensor_collator_;
}

control::ControllerServer* Autonomy::GetController() {
    return controller_.get();
}

void Autonomy::AddMapPublishListener(MapPublishListener listener) {
    if (!listener) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    map_listeners_.push_back(std::move(listener));
}

void Autonomy::AddPathListener(PathListener listener) {
    if (!listener) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    path_listeners_.push_back(std::move(listener));
}

void Autonomy::ApplyMapToCostmap(
    const commsgs::map_msgs::OccupancyGrid::SharedPtr& map) {
    if (!map || !planner_) {
        return;
    }
    if (auto wrapper = planner_->GetCostmapWrapper()) {
        wrapper->applyOccupancyGrid(*map);
    }
}

void Autonomy::NotifyPath(const commsgs::planning_msgs::Path& path) {
    std::vector<PathListener> listeners;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_path_ = path;
        listeners = path_listeners_;
    }
    for (const auto& listener : listeners) {
        if (listener) {
            listener(path);
        }
    }
}

bool Autonomy::GetRobotPose(
    commsgs::geometry_msgs::PoseStamped& pose) const {
    if (!controller_) {
        return false;
    }
    commsgs::planning_msgs::Odometry odom;
    if (!controller_->GetLatestOdometry(odom)) {
        return false;
    }
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    return true;
}

bool Autonomy::TransformPoseToGlobalFrame(
    commsgs::geometry_msgs::PoseStamped& pose) {
    if (!tf_buffer_) {
        return false;
    }
    const std::string& target = task_options_.global_frame().empty()
                                    ? "map"
                                    : task_options_.global_frame();
    if (pose.header.frame_id.empty() || pose.header.frame_id == target) {
        pose.header.frame_id = target;
        return true;
    }
    try {
        pose = tf_buffer_->transform(pose, target, 0.2f);
        return true;
    } catch (const std::exception& ex) {
        AERROR << "TransformPoseToGlobalFrame: " << ex.what();
        return false;
    }
}

bool Autonomy::WaitForNavigation(
    std::function<bool()> cancel_checker, std::function<bool()> keep_alive,
    const double timeout_sec) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_sec);
    while (task_->IsNavigatorActive()) {
        if (cancel_checker && cancel_checker()) {
            task_->Cancel();
            return false;
        }
        if (keep_alive && !keep_alive()) {
            task_->Cancel();
            return false;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            AWARN << "Autonomy: navigation timed out.";
            task_->Cancel();
            return false;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(tasks::kBtWaitPollMs));
    }
    const bool success = task_->LastNavigationSucceeded();
    task_->FinalizeNavigation(success);
    return success;
}

bool Autonomy::NavigateDirectToPose(
    const commsgs::geometry_msgs::PoseStamped& goal,
    std::function<bool()> cancel_checker, std::function<bool()> keep_alive,
    const double timeout_sec) {
    if (!planner_ || !controller_) {
        return false;
    }

    commsgs::geometry_msgs::PoseStamped start;
    if (!GetRobotPose(start)) {
        AERROR << "NavigateDirectToPose: no robot pose.";
        return false;
    }
    commsgs::geometry_msgs::PoseStamped goal_tf = goal;
    if (!TransformPoseToGlobalFrame(goal_tf)) {
        return false;
    }
    if (!TransformPoseToGlobalFrame(start)) {
        return false;
    }

    const auto ids = ResolvePluginIds();
    commsgs::planning_msgs::Path path;
    try {
        path = planner_->GetPlan(start, goal_tf, ids.planner_id,
                                 cancel_checker);
    } catch (const std::exception& ex) {
        AERROR << "GetPlan failed: " << ex.what();
        return false;
    }
    if (path.poses.size() < tasks::kMinPathPoses) {
        AERROR << "NavigateDirectToPose: planned path too short.";
        return false;
    }
    NotifyPath(path);

    if (!controller_->BeginFollowPath(path, ids.controller_id,
                                      ids.goal_checker_id,
                                      "progress_checker")) {
        AERROR << "NavigateDirectToPose: BeginFollowPath failed.";
        return false;
    }
    direct_follow_active_ = true;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_sec);
    bool success = false;
    while (direct_follow_active_.load()) {
        if (cancel_checker && cancel_checker()) {
            controller_->EndFollowPath();
            direct_follow_active_ = false;
            return false;
        }
        if (keep_alive && !keep_alive()) {
            controller_->EndFollowPath();
            direct_follow_active_ = false;
            return false;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            AWARN << "NavigateDirectToPose: timed out.";
            controller_->EndFollowPath();
            direct_follow_active_ = false;
            return false;
        }
        const auto tick =
            controller_->TickFollowPath(cancel_checker ? cancel_checker
                                                       : []() { return false; });
        if (tick == control::ControllerServer::FollowPathTickResult::Succeeded) {
            success = true;
            break;
        }
        if (tick == control::ControllerServer::FollowPathTickResult::Failed ||
            tick == control::ControllerServer::FollowPathTickResult::Cancelled) {
            success = false;
            break;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1000 / tasks::kSpinRateHz));
    }
    controller_->EndFollowPath();
    direct_follow_active_ = false;
    return success;
}

bool Autonomy::NavigateToPose(
    const commsgs::geometry_msgs::PoseStamped& goal,
    std::function<bool()> cancel_checker, std::function<bool()> keep_alive,
    const double timeout_sec) {
    if (!configured_.load() || !task_) {
        AERROR << "Autonomy::NavigateToPose: Task not configured.";
        return false;
    }

    commsgs::geometry_msgs::PoseStamped goal_tf = goal;
    if (!TransformPoseToGlobalFrame(goal_tf)) {
        return false;
    }

    if (use_bt_navigation_) {
        if (!task_->StartNavigateToPose(goal_tf)) {
            return false;
        }
        return WaitForNavigation(std::move(cancel_checker),
                                 std::move(keep_alive), timeout_sec);
    }
    return NavigateDirectToPose(goal_tf, std::move(cancel_checker),
                                std::move(keep_alive), timeout_sec);
}

bool Autonomy::NavigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    std::function<bool()> cancel_checker, std::function<bool()> keep_alive,
    const double timeout_sec) {
    if (!configured_.load() || !task_) {
        AERROR << "Autonomy::NavigateThroughPoses: Task not configured.";
        return false;
    }
    if (goals.empty()) {
        AERROR << "Autonomy::NavigateThroughPoses: empty goals.";
        return false;
    }

    std::vector<commsgs::geometry_msgs::PoseStamped> goals_tf;
    goals_tf.reserve(goals.size());
    for (auto goal : goals) {
        if (!TransformPoseToGlobalFrame(goal)) {
            return false;
        }
        goals_tf.push_back(goal);
    }

    if (use_bt_navigation_) {
        if (!task_->StartNavigateThroughPoses(goals_tf)) {
            return false;
        }
        return WaitForNavigation(std::move(cancel_checker),
                                 std::move(keep_alive), timeout_sec);
    }

    for (const auto& goal : goals_tf) {
        if (!NavigateDirectToPose(goal, cancel_checker, keep_alive,
                                  timeout_sec)) {
            return false;
        }
    }
    return true;
}

void Autonomy::ReplanToGoal(
    const commsgs::geometry_msgs::PoseStamped& goal) {
    if (!configured_.load()) {
        return;
    }
    if (use_bt_navigation_) {
        RequestCancelNavigation();
        NavigateToPose(goal, []() { return false; }, []() { return true; });
        return;
    }
    if (!direct_follow_active_.load() || !planner_ || !controller_) {
        return;
    }
    commsgs::geometry_msgs::PoseStamped start;
    if (!GetRobotPose(start) || !TransformPoseToGlobalFrame(start)) {
        return;
    }
    commsgs::geometry_msgs::PoseStamped goal_tf = goal;
    if (!TransformPoseToGlobalFrame(goal_tf)) {
        return;
    }
    const auto ids = ResolvePluginIds();
    try {
        auto path = planner_->GetPlan(start, goal_tf, ids.planner_id);
        NotifyPath(path);
        controller_->BeginFollowPath(path, ids.controller_id, ids.goal_checker_id,
                                     "progress_checker");
    } catch (const std::exception& ex) {
        AWARN << "ReplanToGoal failed: " << ex.what();
    }
}

std::optional<commsgs::planning_msgs::Path> Autonomy::GetLastPath() {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_path_;
}

void Autonomy::RequestCancelNavigation() {
    if (task_ && task_->IsNavigating()) {
        task_->Cancel();
        task_->FinalizeNavigation(false);
    }
    if (direct_follow_active_.load() && controller_) {
        controller_->EndFollowPath();
        direct_follow_active_ = false;
    }
}

void Autonomy::SetControllerEnabled(const bool enabled) {
    controller_enabled_ = enabled;
    if (!enabled && controller_) {
        controller_->EndFollowPath();
        direct_follow_active_ = false;
    }
}

commsgs::geometry_msgs::TwistStamped Autonomy::TickControl() {
    if (!controller_enabled_.load() || !controller_) {
        return ZeroTwist();
    }
    if (direct_follow_active_.load()) {
        controller_->TickFollowPath([]() { return false; });
    }
    return controller_->GetLastCmdVel();
}

Autonomy::UniquePtr CreateAutonomy(
    const proto::AutonomyOptions& options) {
    return std::make_unique<Autonomy>(options);
}

}  // namespace system
}  // namespace autonomy
