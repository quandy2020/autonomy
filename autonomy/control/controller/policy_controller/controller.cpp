/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include "autolink/common/log.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/proto/controller_options.pb.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

double YawFromQuaternion(
    const automsgs::msgs::geometry_msgs::Quaternion& quaternion) {
    return std::atan2(
        2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
        1.0 - 2.0 * (quaternion.y() * quaternion.y() +
                     quaternion.z() * quaternion.z()));
}

double MaxLinearLimit(const proto::TwistLimits& limits) {
    double max_lin = 0.0;
    if (limits.max_vx() > 0.0) {
        max_lin = std::max(max_lin, limits.max_vx());
    }
    if (limits.max_vy() > 0.0) {
        max_lin = std::max(max_lin, limits.max_vy());
    }
    if (limits.max_vz() > 0.0) {
        max_lin = std::max(max_lin, limits.max_vz());
    }
    return max_lin > 0.0 ? max_lin : 0.5;
}

bool TensorFrameFromFloatArray(
    const automsgs::msgs::std_msgs::Float32MultiArray& array,
    const std::vector<int64_t>& shape, const std::string& default_name,
    bool require_tensor_stamp, TensorFrame* frame) {
    if (frame == nullptr || array.data().empty()) {
        return false;
    }
    frame->name = default_name;
    frame->data.assign(array.data().begin(), array.data().end());
    frame->shape = shape;
    // Float32MultiArray has no header; assign receive wall-clock so age/skew
    // checks can apply once stamp_sec > 0.
    frame->stamp_sec = std::chrono::duration<double>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
    if (require_tensor_stamp && frame->stamp_sec <= 0.0) {
        return false;
    }
    if (shape.empty()) {
        frame->shape.push_back(static_cast<int64_t>(frame->data.size()));
        return true;
    }
    std::size_t expected = 1;
    for (int64_t dim : shape) {
        if (dim > 0) {
            expected *= static_cast<std::size_t>(dim);
        }
    }
    return expected == frame->data.size();
}

bool ReloadPathAllowed(const proto::PolicyControllerOptions& opts,
                       const std::string& config_path, std::string* error) {
    if (opts.reload_config_allow_prefix_size() == 0) {
        return true;
    }
    for (int i = 0; i < opts.reload_config_allow_prefix_size(); ++i) {
        const std::string& prefix = opts.reload_config_allow_prefix(i);
        if (!prefix.empty() && config_path.compare(0, prefix.size(), prefix) == 0) {
            return true;
        }
    }
    if (error != nullptr) {
        *error = "reload_config path not in allowlist: " + config_path;
    }
    return false;
}

std::vector<int64_t> BevShape(const proto::PolicyControllerOptions& opts) {
    std::vector<int64_t> shape;
    for (int i = 0; i < opts.lidar_bev_shape_size(); ++i) {
        if (opts.lidar_bev_shape(i) > 0) {
            shape.push_back(opts.lidar_bev_shape(i));
        }
    }
    return shape;
}

std::vector<int64_t> RawShape(const proto::PolicyControllerOptions& opts) {
    std::vector<int64_t> shape;
    for (int i = 0; i < opts.raw_tensor_shape_size(); ++i) {
        if (opts.raw_tensor_shape(i) > 0) {
            shape.push_back(opts.raw_tensor_shape(i));
        }
    }
    return shape;
}

}  // namespace

void PolicyController::Configure(
    const proto::ControllerOptions& options, std::string name,
    std::shared_ptr<transform::Buffer> tf_buffer,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    options_ = options;
    name_ = name;
    plugin_name_ = std::move(name);
    tf_buffer_ = std::move(tf_buffer);
    costmap_wrapper_ = std::move(costmap_wrapper);

    pipeline_.Configure(options.policy_controller_options());
    pipeline_.SetTfBuffer(tf_buffer_);
    if (options.controller_frequency() > 0.0) {
        control_period_seconds_ = 1.0 / options.controller_frequency();
    }

    std::string error;
    if (!pipeline_.LoadModel(&error)) {
        AWARN << "PolicyController configured without active policy: " << error;
        model_load_error_ = error;
    } else {
        AINFO << "PolicyController loaded config for plugin " << plugin_name_;
        model_load_error_.clear();
    }
    AINFO << "Configured PolicyController: " << plugin_name_;
}

void PolicyController::Cleanup() {
    pipeline_.ClearModels();
    pipeline_.Reset();
    last_goal_checker_ = nullptr;
    AINFO << "Cleaning up PolicyController: " << plugin_name_;
}

void PolicyController::Activate() {
    if (pipeline_.options().require_model() &&
        !pipeline_.policy_manager().HasActive()) {
        std::string error;
        if (!pipeline_.LoadModel(&error)) {
            AERROR << "PolicyController Activate require_model failed: "
                   << (error.empty() ? model_load_error_ : error);
            throw common::ControllerException(
                "PolicyController require_model: no active policy");
        }
    }
    AINFO << "Activating PolicyController: " << plugin_name_;
}

void PolicyController::Deactivate() {
    last_goal_checker_ = nullptr;
    AINFO << "Deactivating PolicyController: " << plugin_name_;
}

bool PolicyController::LoadConfig(const std::string& config_path,
                                  std::string* error) {
    return pipeline_.LoadConfig(config_path, error);
}

bool PolicyController::LoadConfig(const proto::PolicyConfig& config,
                                  std::string* error) {
    return pipeline_.LoadConfig(config, error);
}

bool PolicyController::LoadConfigAsync(
    const std::string& config_path,
    std::function<void(bool, const std::string&)> done) {
    std::string error;
    if (!ReloadPathAllowed(pipeline_.options(), config_path, &error)) {
        AWARN << "PolicyController: " << error;
        if (done) {
            done(false, error);
        }
        return false;
    }
    return pipeline_.LoadConfigAsync(config_path, std::move(done));
}

bool PolicyController::LoadConfigAsync(
    const proto::PolicyConfig& config,
    std::function<void(bool, const std::string&)> done) {
    PolicyConfig parsed;
    std::string error;
    if (!PolicyConfig::LoadFromProto(config, &parsed, &error)) {
        if (done) {
            done(false, error);
        }
        return false;
    }
    return pipeline_.LoadConfigAsync(std::move(parsed), std::move(done));
}

void PolicyController::SetLaserScan(
    const automsgs::msgs::sensor_msgs::LaserScan& laser_scan) {
    pipeline_.SetLaserScan(laser_scan);
}

void PolicyController::SetImage(
    const automsgs::msgs::sensor_msgs::Image& image) {
    pipeline_.SetImage(image);
}

void PolicyController::SetPointCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& point_cloud) {
    pipeline_.SetPointCloud(point_cloud);
}

void PolicyController::SetLidarBev(const TensorFrame& lidar_bev) {
    pipeline_.SetLidarBev(lidar_bev);
}

void PolicyController::SetRawTensor(const TensorFrame& raw) {
    pipeline_.SetRawTensor(raw);
}

void PolicyController::SetLidarBevArray(
    const automsgs::msgs::std_msgs::Float32MultiArray& array) {
    TensorFrame frame;
    const auto& opts = pipeline_.options();
    const std::string name = opts.lidar_bev_name().empty()
                                 ? "observation_lidar_bev"
                                 : opts.lidar_bev_name();
    if (!TensorFrameFromFloatArray(array, BevShape(opts), name,
                                   opts.require_tensor_stamp(), &frame)) {
        AWARN << "PolicyController: invalid lidar_bev Float32MultiArray";
        return;
    }
    pipeline_.SetLidarBev(frame);
}

void PolicyController::SetRawTensorArray(
    const automsgs::msgs::std_msgs::Float32MultiArray& array) {
    TensorFrame frame;
    const auto& opts = pipeline_.options();
    const std::string name =
        opts.raw_tensor_name().empty() ? "observation_raw" : opts.raw_tensor_name();
    if (!TensorFrameFromFloatArray(array, RawShape(opts), name,
                                   opts.require_tensor_stamp(), &frame)) {
        AWARN << "PolicyController: invalid raw Float32MultiArray";
        return;
    }
    pipeline_.SetRawTensor(frame);
}

void PolicyController::SetPlan(const automsgs::msgs::nav_msgs::Path& plan) {
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (plan.poses().empty()) {
            goal_pose_.reset();
        } else {
            goal_pose_ = plan.poses(plan.poses_size() - 1).pose();
        }
    }
    pipeline_.SetPath(plan);
}

void PolicyController::Reset() {
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        goal_pose_.reset();
        last_velocity_.reset();
    }
    last_goal_checker_ = nullptr;
    pipeline_.Reset();
}

void PolicyController::SetSpeedLimit(const double& speed_limit,
                                     const bool& percentage) {
    if (percentage) {
        speed_limit_scale_ = std::max(0.0, speed_limit / 100.0);
    } else {
        double max_lin = 0.5;
        auto model = pipeline_.policy_manager().Active();
        if (model) {
            max_lin = MaxLinearLimit(model->config().proto().action().limits());
        }
        speed_limit_scale_ =
            max_lin > 0.0 ? std::max(0.0, speed_limit / max_lin) : 1.0;
    }
    pipeline_.SetSpeedScale(speed_limit_scale_ * near_goal_scale_);
}

bool PolicyController::IsGoalReached(double distance_tolerance,
                                     double angle_tolerance) {
    automsgs::msgs::geometry_msgs::Pose goal;
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (!goal_pose_.has_value()) {
            return false;
        }
        goal = *goal_pose_;
    }
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    if (!pipeline_.observation_buffer().GetPose(&pose)) {
        return false;
    }

    if (last_goal_checker_ != nullptr) {
        automsgs::msgs::geometry_msgs::Twist velocity;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (last_velocity_.has_value()) {
                velocity = *last_velocity_;
            }
        }
        return last_goal_checker_->IsGoalReached(pose.pose(), goal, velocity);
    }

    const auto& position = pose.pose().position();
    const double delta_x = goal.position().x() - position.x();
    const double delta_y = goal.position().y() - position.y();
    const double distance = std::hypot(delta_x, delta_y);
    const double yaw_error = std::remainder(
        YawFromQuaternion(goal.orientation()) -
            YawFromQuaternion(pose.pose().orientation()),
        2.0 * M_PI);
    const double distance_tol =
        distance_tolerance > 0.0
            ? distance_tolerance
            : pipeline_.options().goal_xy_tolerance();
    const double yaw_tol = angle_tolerance > 0.0
                               ? angle_tolerance
                               : pipeline_.options().goal_yaw_tolerance();
    return distance <= distance_tol && std::fabs(yaw_error) <= yaw_tol;
}

uint32 PolicyController::ComputeVelocityCommands(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose,
    const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
    automsgs::msgs::geometry_msgs::TwistStamped& command_velocity,
    common::GoalChecker* goal_checker, std::string& message) {
    last_goal_checker_ = goal_checker;
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        last_velocity_ = velocity.twist();
    }

    near_goal_scale_ = 1.0;
    const double near_goal_distance =
        pipeline_.options().near_goal_distance();
    if (near_goal_distance > 0.0) {
        automsgs::msgs::geometry_msgs::Pose goal;
        bool has_goal = false;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (goal_pose_.has_value()) {
                goal = *goal_pose_;
                has_goal = true;
            }
        }
        if (has_goal) {
            const double dx = goal.position().x() - pose.pose().position().x();
            const double dy = goal.position().y() - pose.pose().position().y();
            if (std::hypot(dx, dy) <= near_goal_distance) {
                near_goal_scale_ =
                    pipeline_.options().near_goal_speed_scale() > 0.0
                        ? pipeline_.options().near_goal_speed_scale()
                        : 0.3;
            }
        }
    }
    pipeline_.SetNearGoalScaled(near_goal_scale_ < 1.0);
    pipeline_.SetSpeedScale(speed_limit_scale_ * near_goal_scale_);
    return pipeline_.ComputeCommand(pose, velocity, costmap_wrapper_.get(),
                                    control_period_seconds_, &command_velocity,
                                    &message);
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
