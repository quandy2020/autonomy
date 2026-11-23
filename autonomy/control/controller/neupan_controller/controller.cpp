/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/control/controller/neupan_controller/controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/control/controller/neupan_controller/distance_encoder.hpp"
#include "autonomy/control/proto/controller_options.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {
namespace {

double ExtractYawFromPose(const automsgs::msgs::geometry_msgs::Pose& pose) {
    const auto& q = pose.orientation();
    const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

Kinematics ParseKinematics(const std::string& value) {
    if (value == "omni" || value == "omnidirectional") {
        return Kinematics::kOmni;
    }
    return Kinematics::kDiff;
}

Mat2X TransformBaseToWorld(const automsgs::msgs::geometry_msgs::Pose2D& pose,
                           const Eigen::Matrix2Xd& points_in_base) {
    Mat2X world(2, points_in_base.cols());
    const double c = std::cos(pose.theta());
    const double s = std::sin(pose.theta());
    for (int i = 0; i < points_in_base.cols(); ++i) {
        world(0, i) =
            pose.x() + c * points_in_base(0, i) - s * points_in_base(1, i);
        world(1, i) =
            pose.y() + s * points_in_base(0, i) + c * points_in_base(1, i);
    }
    return world;
}

NeuPanPlanner::Config ConfigFromOptions(const proto::NeupanControllerOptions& o,
                                        const Mat2X& vertices) {
    NeuPanPlanner::Config config;
    config.receding = o.planning_horizon() > 0 ? o.planning_horizon() : 10;
    config.step_time = o.control_period() > 0.0 ? o.control_period() : 0.1;
    config.ref_speed =
        o.reference_linear_speed() > 0.0 ? o.reference_linear_speed() : 0.5;
    config.collision_threshold =
        o.collision_stop_distance() > 0.0 ? o.collision_stop_distance() : 0.1;
    config.kinematics = ParseKinematics(o.robot_kinematics());
    config.max_speed =
        Vec2(o.max_linear_speed() > 0.0 ? o.max_linear_speed() : 1.0,
             o.max_angular_speed() > 0.0 ? o.max_angular_speed() : 1.0);
    config.max_acce =
        Vec2(o.max_linear_acceleration() > 0.0 ? o.max_linear_acceleration()
                                               : 1.0,
             o.max_angular_acceleration() > 0.0 ? o.max_angular_acceleration()
                                                : 1.0);
    config.length = o.footprint_length() > 0.0 ? o.footprint_length() : 0.8;
    config.width = o.footprint_width() > 0.0 ? o.footprint_width() : 0.6;
    config.wheelbase = o.wheelbase();
    config.vertices = vertices;
    config.ipath.projected_reference = true;
    config.ipath.arrive_threshold =
        o.goal_position_tolerance() > 0.0 ? o.goal_position_tolerance() : 0.25;
    config.pan.iter_num =
        o.planner_iteration_count() > 0 ? o.planner_iteration_count() : 2;
    config.pan.dune_max_num = o.max_obstacle_points_for_encoder() > 0
                                  ? o.max_obstacle_points_for_encoder()
                                  : 100;
    config.adjust.max_num = o.max_obstacle_points_for_constraints() > 0
                                ? o.max_obstacle_points_for_constraints()
                                : 10;
    if (o.state_tracking_weight() > 0.0) {
        config.adjust.q_s = Vec3::Constant(o.state_tracking_weight());
    }
    if (o.control_effort_weight() > 0.0) {
        config.adjust.p_u = o.control_effort_weight();
    }
    if (o.distance_slack_gain() > 0.0) {
        config.adjust.eta = o.distance_slack_gain();
    }
    if (o.safety_distance_max() > 0.0) {
        config.adjust.d_max = o.safety_distance_max();
    }
    if (o.safety_distance_min() > 0.0) {
        config.adjust.d_min = o.safety_distance_min();
    }
    return config;
}

}  // namespace

void NeupanController::Configure(
    const proto::ControllerOptions& options, std::string name,
    std::shared_ptr<transform::Buffer> tf_buffer,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    options_ = options;
    name_ = std::move(name);
    tf_buffer_ = std::move(tf_buffer);
    costmap_wrapper_ = std::move(costmap_wrapper);
    options_neupan_ = options.neupan_controller_options();
    omni_ = ParseKinematics(options_neupan_.robot_kinematics()) ==
            Kinematics::kOmni;
    goal_position_tolerance_ = options_neupan_.goal_position_tolerance() > 0.0
                                   ? options_neupan_.goal_position_tolerance()
                                   : 0.25;
    max_encoder_point_count_ =
        options_neupan_.max_obstacle_points_for_encoder() > 0
            ? options_neupan_.max_obstacle_points_for_encoder()
            : 100;
    CacheLaserExtrinsic();

    std::string error;
    if (!EnsureInitialized(&error)) {
        AERROR << "[" << name_ << "] NeupanController load failed: " << error;
    } else {
        AINFO << "[" << name_ << "] NeuPAN planner ready"
              << (options_neupan_.neural_encoder_model_path().empty()
                      ? " (path tracking, no DUNE encoder)"
                      : " encoder=" +
                            options_neupan_.neural_encoder_model_path());
    }
}

void NeupanController::CacheLaserExtrinsic() {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (options_neupan_.laser_scan_pose_in_base_size() >= 1) {
        x = options_neupan_.laser_scan_pose_in_base(0);
    }
    if (options_neupan_.laser_scan_pose_in_base_size() >= 2) {
        y = options_neupan_.laser_scan_pose_in_base(1);
    }
    if (options_neupan_.laser_scan_pose_in_base_size() >= 3) {
        yaw = options_neupan_.laser_scan_pose_in_base(2);
    }
    laser_extrinsic_.x = x;
    laser_extrinsic_.y = y;
    laser_extrinsic_.cos_yaw = std::cos(yaw);
    laser_extrinsic_.sin_yaw = std::sin(yaw);
    laser_extrinsic_.range_min = options_neupan_.laser_range_min();
    laser_extrinsic_.range_max = options_neupan_.laser_range_max();
}

Mat2X NeupanController::CreateVertices(std::string* error) const {
    if (options_neupan_.footprint_vertices_size() >= 6 &&
        options_neupan_.footprint_vertices_size() % 2 == 0) {
        const int n = options_neupan_.footprint_vertices_size() / 2;
        Mat2X vertices(2, n);
        for (int i = 0; i < n; ++i) {
            vertices(0, i) = options_neupan_.footprint_vertices(2 * i);
            vertices(1, i) = options_neupan_.footprint_vertices(2 * i + 1);
        }
        return vertices;
    }
    if (options_neupan_.footprint_vertices_size() > 0) {
        if (error) {
            *error = "invalid footprint_vertices";
        }
        return Mat2X(2, 0);
    }
    const double length = options_neupan_.footprint_length() > 0.0
                              ? options_neupan_.footprint_length()
                              : 0.8;
    const double width = options_neupan_.footprint_width() > 0.0
                             ? options_neupan_.footprint_width()
                             : 0.6;
    return RobotModel::RectangleVertices(
        ParseKinematics(options_neupan_.robot_kinematics()), length, width,
        options_neupan_.wheelbase());
}

bool NeupanController::EnsureInitialized(std::string* error) {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    if (planner_) {
        return true;
    }
    const Mat2X vertices = CreateVertices(error);
    if (vertices.cols() < 3) {
        if (error && error->empty()) {
            *error = "failed to create robot footprint vertices";
        }
        return false;
    }
    std::shared_ptr<const DistanceEncoder> encoder;
    if (!options_neupan_.neural_encoder_model_path().empty()) {
        std::string model_path;
        if (!::autonomy::common::ResolveModuleAsset(
                "control", options_neupan_.neural_encoder_model_path(),
                &model_path)) {
            if (error) {
                *error = "neural encoder not found: " +
                         options_neupan_.neural_encoder_model_path() +
                         " (place ONNX under autonomy/control/conf/models/)";
            }
            return false;
        }
        const RobotModel robot(ParseKinematics(options_neupan_.robot_kinematics()),
                               options_neupan_.planning_horizon() > 0
                                   ? options_neupan_.planning_horizon()
                                   : 10,
                               options_neupan_.control_period() > 0.0
                                   ? options_neupan_.control_period()
                                   : 0.1,
                               Vec2(1.0, 1.0), Vec2(1.0, 1.0), vertices);
        encoder = DistanceEncoder::Create(
            model_path, robot.G, robot.h, options_neupan_.inference_backend(),
            options_neupan_.inference_execution_provider(), error);
        if (!encoder) {
            return false;
        }
    }
    planner_ = std::make_unique<NeuPanPlanner>(
        ConfigFromOptions(options_neupan_, vertices), std::move(encoder));
    if (path_points_.size() >= 2) {
        planner_->SetInitialPath(path_points_);
    }
    return true;
}

void NeupanController::Cleanup() {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    planner_.reset();
}

void NeupanController::Activate() {}

void NeupanController::Deactivate() {}

void NeupanController::Reset() {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    path_points_.clear();
    has_pose_ = false;
    if (planner_) {
        planner_->Reset();
    }
}

void NeupanController::SetPlan(const automsgs::msgs::nav_msgs::Path& plan) {
    std::vector<InitialPath::PathPoint> path;
    path.reserve(static_cast<size_t>(plan.poses_size()));
    for (const auto& stamped : plan.poses()) {
        path.emplace_back(stamped.pose().position().x(),
                          stamped.pose().position().y(),
                          ExtractYawFromPose(stamped.pose()), 1.0);
    }
    std::lock_guard<std::mutex> lock(planner_mutex_);
    path_points_ = std::move(path);
    if (planner_ && path_points_.size() >= 2) {
        planner_->SetInitialPath(path_points_);
    }
}

void NeupanController::SetSpeedLimit(const double& speed_limit,
                                     const bool& percentage) {
    if (percentage) {
        speed_limit_scale_ = std::max(0.0, std::min(1.0, speed_limit));
    } else if (options_neupan_.max_linear_speed() > 0.0 && speed_limit > 0.0) {
        speed_limit_scale_ = std::max(
            0.0,
            std::min(1.0, speed_limit / options_neupan_.max_linear_speed()));
    }
}

void NeupanController::SetLaserScan(
    const automsgs::msgs::sensor_msgs::LaserScan& laser_scan) {
    auto scan = std::make_shared<const automsgs::msgs::sensor_msgs::LaserScan>(
        laser_scan);
    std::lock_guard<std::mutex> lock(laser_scan_mutex_);
    laser_scan_ = std::move(scan);
}

bool NeupanController::IsGoalReached(double distance_tolerance,
                                     double angle_tolerance) {
    (void)angle_tolerance;
    std::lock_guard<std::mutex> lock(planner_mutex_);
    if (!has_pose_ || path_points_.empty()) {
        return false;
    }
    const double tolerance = distance_tolerance > 0.0
                                 ? distance_tolerance
                                 : goal_position_tolerance_;
    const double dx = last_pose_.x() - path_points_.back()(0);
    const double dy = last_pose_.y() - path_points_.back()(1);
    return std::hypot(dx, dy) <= tolerance;
}

uint32 NeupanController::ComputeVelocityCommands(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose,
    const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
    automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel,
    common::GoalChecker* goal_checker, std::string& message) {
    (void)goal_checker;
    cmd_vel.Clear();
    cmd_vel.mutable_header()->CopyFrom(pose.header());

    std::string error;
    if (!EnsureInitialized(&error)) {
        message = error;
        return proto::CONTROLLER_RESULT_NOT_INITIALIZED;
    }

    last_pose_.set_x(pose.pose().position().x());
    last_pose_.set_y(pose.pose().position().y());
    last_pose_.set_theta(ExtractYawFromPose(pose.pose()));
    has_pose_ = true;

    std::shared_ptr<const automsgs::msgs::sensor_msgs::LaserScan> laser_scan;
    {
        std::lock_guard<std::mutex> lock(laser_scan_mutex_);
        laser_scan = laser_scan_;
    }
    if (laser_scan) {
        ExtractObstaclePointsFromLaserScan(*laser_scan, laser_extrinsic_,
                                           max_encoder_point_count_,
                                           &obstacle_points_buffer_);
    } else {
        obstacle_points_buffer_.resize(2, 0);
    }

    const Mat2X world_points =
        TransformBaseToWorld(last_pose_, obstacle_points_buffer_);
    const Vec3 state(last_pose_.x(), last_pose_.y(), last_pose_.theta());

    NeuPanPlanner::Command prev;
    if (omni_) {
        prev.action = Vec2(velocity.twist().linear().x(),
                           velocity.twist().linear().y());
    } else {
        prev.action = Vec2(velocity.twist().linear().x(),
                           velocity.twist().angular().z());
    }
    prev.elapsed = options_neupan_.control_period() > 0.0
                       ? options_neupan_.control_period()
                       : 0.1;

    NeuPanPlanner::Info info;
    Vec2 u = Vec2::Zero();
    {
        std::lock_guard<std::mutex> lock(planner_mutex_);
        if (!planner_ || path_points_.size() < 2) {
            message = "empty path";
            return proto::CONTROLLER_RESULT_INVALID_PATH;
        }
        u = planner_->Forward(state, world_points, &info, prev);
    }

    if (info.arrive) {
        message = "goal reached";
        cmd_vel.mutable_twist()->mutable_linear()->set_x(0.0);
        cmd_vel.mutable_twist()->mutable_angular()->set_z(0.0);
        return proto::CONTROLLER_RESULT_SUCCESS;
    }
    if (info.stop) {
        message = "collision stop distance";
        cmd_vel.mutable_twist()->mutable_linear()->set_x(0.0);
        cmd_vel.mutable_twist()->mutable_angular()->set_z(0.0);
        return proto::CONTROLLER_RESULT_COLLISION;
    }

    cmd_vel.mutable_twist()->mutable_linear()->set_x(u(0) * speed_limit_scale_);
    if (omni_) {
        cmd_vel.mutable_twist()->mutable_linear()->set_y(u(1) *
                                                         speed_limit_scale_);
    } else {
        cmd_vel.mutable_twist()->mutable_angular()->set_z(u(1));
    }
    message = info.solved ? "ok" : "nominal";
    return proto::CONTROLLER_RESULT_SUCCESS;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

CLASS_LOADER_REGISTER_CLASS(
    autonomy::control::controller::neupan_controller::NeupanController,
    autonomy::control::common::ControllerInterface)
