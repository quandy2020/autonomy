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
#include "autonomy/sensor/internal/sensor_collator.hpp"
#include "autonomy/navigator/constants.hpp"
#include "autonomy/navigator/options.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/transform_server.hpp"

namespace autonomy {
namespace system {
namespace {

automsgs::msgs::geometry_msgs::TwistStamped ZeroTwist() {
    automsgs::msgs::geometry_msgs::TwistStamped cmd;
    cmd.mutable_twist()->mutable_linear()->set_x(0.0);
    cmd.mutable_twist()->mutable_linear()->set_y(0.0);
    cmd.mutable_twist()->mutable_linear()->set_z(0.0);
    cmd.mutable_twist()->mutable_angular()->set_x(0.0);
    cmd.mutable_twist()->mutable_angular()->set_y(0.0);
    cmd.mutable_twist()->mutable_angular()->set_z(0.0);
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
        PickId(runtime_.planner_id, navigator_options_.default_planner_id()),
        PickId(runtime_.controller_id, navigator_options_.default_controller_id()),
        PickId(runtime_.goal_checker_id,
               navigator_options_.default_goal_checker_id()),
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

void Autonomy::ApplyRuntimeToNavigatorOptions(const RuntimeOptions& runtime) {
    if (!runtime.global_frame.empty()) {
        navigator_options_.set_global_frame(runtime.global_frame);
    }
    if (!runtime.planner_id.empty()) {
        navigator_options_.set_default_planner_id(runtime.planner_id);
    }
    if (!runtime.controller_id.empty()) {
        navigator_options_.set_default_controller_id(runtime.controller_id);
    }
    if (!runtime.goal_checker_id.empty()) {
        navigator_options_.set_default_goal_checker_id(runtime.goal_checker_id);
    }
    if (!runtime.robot_base_frame.empty()) {
        navigator_options_.set_robot_base_frame(runtime.robot_base_frame);
    }
    if (runtime.goal_tolerance > 0.0) {
        navigator_options_.set_goal_reached_tolerance(runtime.goal_tolerance);
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
        const auto& static_transforms =
            transform_server_->GetTransformStampedsData();
        for (const auto& trans : static_transforms.transforms()) {
            try {
                geometry_msgs::TransformStamped geo_msg;
                geo_msg.header.stamp = static_cast<uint64_t>(trans.header().stamp().sec()) *
                        1000000000ULL +
                    static_cast<uint64_t>(trans.header().stamp().nanosec());
                geo_msg.header.frame_id = trans.header().frame_id();
                geo_msg.child_frame_id = trans.child_frame_id();
                geo_msg.transform.translation.x = trans.transform().translation().x();
                geo_msg.transform.translation.y = trans.transform().translation().y();
                geo_msg.transform.translation.z = trans.transform().translation().z();
                geo_msg.transform.rotation.x = trans.transform().rotation().x();
                geo_msg.transform.rotation.y = trans.transform().rotation().y();
                geo_msg.transform.rotation.z = trans.transform().rotation().z();
                geo_msg.transform.rotation.w = trans.transform().rotation().w();
                tf_buffer_->setTransform(geo_msg, "autonomy_system", true);
            } catch (const std::exception& ex) {
                AWARN << "Failed to load static transform: " << ex.what();
            }
        }
    }

    if (map_server_ && planner_) {
        map_server_->SetMapPublishCallback(
            [this](const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& map) {
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

void Autonomy::Configure(const RuntimeOptions& runtime) {
    if (!EnsureStarted()) {
        return;
    }
    runtime_ = runtime;
    use_bt_navigation_ = false;

    if (options_.has_navigator_options()) {
        navigator_options_ = options_.navigator_options();
    } else if (!runtime.config_directory.empty()) {
        navigator_options_ = navigator::CreateOptions(
            runtime.config_directory, "navigator/navigator.lua");
    }

    ApplyRuntimeToNavigatorOptions(runtime);
    SyncNavigationFrames(runtime.global_frame, runtime.robot_base_frame);

    if (!planner_ || !controller_ || !tf_buffer_) {
        AERROR << "Autonomy::Configure: missing server dependencies.";
        configured_ = false;
        return;
    }

    configured_ = true;
    AINFO << "Autonomy configured (direct planner wiring).";
}

void Autonomy::Shutdown() {
    if (!started_.exchange(false)) {
        return;
    }
    if (controller_) {
        controller_->Shutdown();
    }
    if (map_server_) {
        map_server_->Shutdown();
    }
    configured_ = false;
    map_server_.reset();
    planner_.reset();
    controller_.reset();
    sensor_collator_.reset();
    transform_server_.reset();
    AINFO << "Autonomy shut down.";
}

bool Autonomy::IsReady() const {
    return configured_.load() && started_.load();
}

map::MapServer* Autonomy::GetMapServer() { return map_server_.get(); }

map::costmap_2d::Costmap2DWrapper::SharedPtr Autonomy::GetGlobalCostmap() {
    if (!planner_) {
        return nullptr;
    }
    return planner_->GetCostmapWrapper();
}

sensor::CollatorInterface& Autonomy::GetSensorCollator() {
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
    const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& map) {
    if (!map || !planner_) {
        return;
    }
    if (auto wrapper = planner_->GetCostmapWrapper()) {
        wrapper->applyOccupancyGrid(*map);
    }
}

void Autonomy::NotifyPath(const automsgs::msgs::nav_msgs::Path& path) {
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
    automsgs::msgs::geometry_msgs::PoseStamped& pose) const {
    if (!controller_) {
        return false;
    }
    automsgs::msgs::nav_msgs::Odometry odom;
    if (!controller_->GetLatestOdometry(odom)) {
        return false;
    }
    *pose.mutable_header() = odom.header();
    *pose.mutable_pose() = odom.pose().pose().pose();
    return true;
}

bool Autonomy::TransformPoseToGlobalFrame(
    automsgs::msgs::geometry_msgs::PoseStamped& pose) {
    if (!tf_buffer_) {
        return false;
    }
    const std::string& target = navigator_options_.global_frame().empty()
                                    ? "map"
                                    : navigator_options_.global_frame();
    if (pose.header().frame_id().empty() || pose.header().frame_id() == target ) {
        pose.mutable_header()->set_frame_id(target);
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

bool Autonomy::NavigateDirectToPose(
    const automsgs::msgs::geometry_msgs::PoseStamped& goal,
    std::function<bool()> cancel_checker, std::function<bool()> keep_alive,
    const double timeout_sec) {
    (void)timeout_sec;
    (void)keep_alive;
    if (!planner_ || !controller_) {
        return false;
    }

    automsgs::msgs::geometry_msgs::PoseStamped start;
    if (!GetRobotPose(start)) {
        AERROR << "NavigateDirectToPose: no robot pose.";
        return false;
    }
    automsgs::msgs::geometry_msgs::PoseStamped goal_tf = goal;
    if (!TransformPoseToGlobalFrame(goal_tf)) {
        return false;
    }
    if (!TransformPoseToGlobalFrame(start)) {
        return false;
    }

    const auto ids = ResolvePluginIds();
    automsgs::msgs::nav_msgs::Path path;
    try {
        path = planner_->GetPlan(start, goal_tf, ids.planner_id,
                                 cancel_checker);
    } catch (const std::exception& ex) {
        AERROR << "GetPlan failed: " << ex.what();
        return false;
    }
    if (path.poses_size() < navigator::kMinPathPoses) {
        AERROR << "NavigateDirectToPose: planned path too short.";
        return false;
    }
    NotifyPath(path);
    return true;
}

bool Autonomy::NavigateToPose(
    const automsgs::msgs::geometry_msgs::PoseStamped& goal,
    std::function<bool()> cancel_checker, std::function<bool()> keep_alive,
    const double timeout_sec) {
    if (!configured_.load()) {
        AERROR << "Autonomy::NavigateToPose: not configured.";
        return false;
    }

    automsgs::msgs::geometry_msgs::PoseStamped goal_tf = goal;
    if (!TransformPoseToGlobalFrame(goal_tf)) {
        return false;
    }
    return NavigateDirectToPose(goal_tf, std::move(cancel_checker),
                                std::move(keep_alive), timeout_sec);
}

bool Autonomy::NavigateThroughPoses(
    const std::vector<automsgs::msgs::geometry_msgs::PoseStamped>& goals,
    std::function<bool()> cancel_checker, std::function<bool()> keep_alive,
    const double timeout_sec) {
    if (!configured_.load()) {
        AERROR << "Autonomy::NavigateThroughPoses: not configured.";
        return false;
    }
    if (goals.empty()) {
        AERROR << "Autonomy::NavigateThroughPoses: empty goals.";
        return false;
    }

    for (auto goal : goals) {
        if (!TransformPoseToGlobalFrame(goal)) {
            return false;
        }
        if (!NavigateDirectToPose(goal, cancel_checker, keep_alive,
                                  timeout_sec)) {
            return false;
        }
    }
    return true;
}

void Autonomy::ReplanToGoal(
    const automsgs::msgs::geometry_msgs::PoseStamped& goal) {
    if (!configured_.load() || !planner_) {
        return;
    }
    automsgs::msgs::geometry_msgs::PoseStamped start;
    if (!GetRobotPose(start) || !TransformPoseToGlobalFrame(start)) {
        return;
    }
    automsgs::msgs::geometry_msgs::PoseStamped goal_tf = goal;
    if (!TransformPoseToGlobalFrame(goal_tf)) {
        return;
    }
    const auto ids = ResolvePluginIds();
    try {
        auto path = planner_->GetPlan(start, goal_tf, ids.planner_id,
                                      []() { return false; });
        NotifyPath(path);
    } catch (const std::exception& ex) {
        AWARN << "ReplanToGoal failed: " << ex.what();
    }
}

std::optional<automsgs::msgs::nav_msgs::Path> Autonomy::GetLastPath() {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_path_;
}

void Autonomy::RequestCancelNavigation() {}

void Autonomy::SetControllerEnabled(const bool enabled) {
    controller_enabled_ = enabled;
}

automsgs::msgs::geometry_msgs::TwistStamped Autonomy::TickControl() {
    if (!controller_enabled_.load() || !controller_) {
        return ZeroTwist();
    }
    return last_control_cmd_;
}

automsgs::msgs::geometry_msgs::TwistStamped Autonomy::GetLastControlCommand() const {
    return last_control_cmd_;
}

Autonomy::UniquePtr CreateAutonomy(
    const proto::AutonomyOptions& options) {
    return std::make_unique<Autonomy>(options);
}

}  // namespace system
}  // namespace autonomy
