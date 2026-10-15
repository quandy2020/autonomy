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

#include "autonomy/control/controller/neupan_controller/motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {
namespace {

double ClampValue(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

double NormalizeAngle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

void ResetTwistToZero(automsgs::msgs::geometry_msgs::Twist* command) {
    if (command == nullptr) {
        return;
    }
    command->Clear();
    command->mutable_linear()->set_x(0.0);
    command->mutable_angular()->set_z(0.0);
}

}  // namespace

MotionPlanner::MotionPlanner(proto::NeupanControllerOptions options,
                             std::unique_ptr<DistanceEncoder> encoder)
    : encoder_(std::move(encoder)) {
    control_period_ =
        options.control_period() > 0.0 ? options.control_period() : 0.1;
    reference_linear_speed_ = options.reference_linear_speed() > 0.0
                                  ? options.reference_linear_speed()
                                  : 0.5;
    const int planning_horizon =
        options.planning_horizon() > 0 ? options.planning_horizon() : 10;
    lookahead_distance_ = std::max(
        0.3, reference_linear_speed_ * control_period_ * planning_horizon);
    goal_position_tolerance_ = options.goal_position_tolerance() > 0.0
                                   ? options.goal_position_tolerance()
                                   : 0.25;
    collision_stop_distance_ = options.collision_stop_distance() > 0.0
                                   ? options.collision_stop_distance()
                                   : 0.1;
    max_linear_speed_ =
        options.max_linear_speed() > 0.0 ? options.max_linear_speed() : 1.0;
    max_angular_speed_ =
        options.max_angular_speed() > 0.0 ? options.max_angular_speed() : 1.0;
    max_linear_acceleration_ = options.max_linear_acceleration();
    max_angular_acceleration_ = options.max_angular_acceleration();
    safety_distance_min_ = options.safety_distance_min() > 0.0
                               ? options.safety_distance_min()
                               : 0.1;
    safety_distance_max_ = options.safety_distance_max() > 0.0
                               ? options.safety_distance_max()
                               : 1.0;
    distance_slack_gain_ = options.distance_slack_gain() > 0.0
                               ? options.distance_slack_gain()
                               : 15.0;
    state_tracking_weight_ = options.state_tracking_weight() > 0.0
                                 ? options.state_tracking_weight()
                                 : 1.0;
    max_encoder_point_count_ = options.max_obstacle_points_for_encoder() > 0
                                   ? options.max_obstacle_points_for_encoder()
                                   : 100;
}

MotionPlanner::Result MotionPlanner::ComputeVelocityCommand(
    const automsgs::msgs::geometry_msgs::Pose2D& pose,
    const automsgs::msgs::geometry_msgs::Twist& velocity,
    const PathCache& path_cache,
    const Eigen::Matrix2Xd& obstacle_points_in_base,
    automsgs::msgs::geometry_msgs::Twist* command,
    double* min_obstacle_distance, std::string* message) const {
    ResetTwistToZero(command);
    if (min_obstacle_distance != nullptr) {
        *min_obstacle_distance = std::numeric_limits<double>::infinity();
    }

    auto set_message = [&](const char* text) {
        if (message != nullptr) {
            *message = text;
        }
    };

    if (!encoder_) {
        set_message("obstacle distance encoder not loaded");
        return Result::kFailure;
    }
    if (path_cache.Empty()) {
        set_message("empty path");
        return Result::kFailure;
    }
    if (command == nullptr) {
        set_message("null command");
        return Result::kFailure;
    }

    automsgs::msgs::geometry_msgs::Pose2D lookahead_pose;
    double goal_distance = 0.0;
    if (!path_cache.ComputeLookaheadPose(pose, lookahead_distance_,
                                         &lookahead_pose, &goal_distance)) {
        set_message("lookahead failed");
        return Result::kFailure;
    }

    if (goal_distance < goal_position_tolerance_) {
        set_message("goal reached");
        return Result::kGoalReached;
    }

    const double cos_theta = std::cos(pose.theta());
    const double sin_theta = std::sin(pose.theta());
    Eigen::Matrix2d world_from_robot;
    world_from_robot << cos_theta, -sin_theta, sin_theta, cos_theta;

    DistanceFeatures features;
    std::string error;
    if (!encoder_->ComputeDistanceFeatures(
            obstacle_points_in_base, world_from_robot, &features, &error)) {
        if (message != nullptr) {
            *message = std::move(error);
        }
        return Result::kFailure;
    }
    if (min_obstacle_distance != nullptr) {
        *min_obstacle_distance = features.min_distance;
    }

    if (std::isfinite(features.min_distance) &&
        features.min_distance < collision_stop_distance_) {
        set_message("collision stop distance");
        return Result::kCollision;
    }

    const double delta_x = lookahead_pose.x() - pose.x();
    const double delta_y = lookahead_pose.y() - pose.y();
    const double local_x = cos_theta * delta_x + sin_theta * delta_y;
    const double local_y = -sin_theta * delta_x + cos_theta * delta_y;
    const double pursuit_distance =
        std::max(0.2, std::sqrt(local_x * local_x + local_y * local_y));
    double curvature = 2.0 * local_y / (pursuit_distance * pursuit_distance);

    double linear_speed = std::min(reference_linear_speed_, max_linear_speed_);
    if (std::isfinite(features.min_distance)) {
        linear_speed *= ClampValue(
            (features.min_distance - safety_distance_min_) /
                std::max(1e-3, safety_distance_max_ - safety_distance_min_),
            0.0, 1.0);
    }

    if (std::isfinite(features.min_distance) &&
        features.min_distance < safety_distance_max_) {
        const double lam_wx = features.nearest_dual_direction.x();
        const double lam_wy = features.nearest_dual_direction.y();
        const double lam_robot_y = -sin_theta * lam_wx + cos_theta * lam_wy;
        const double repulsive_gain =
            distance_slack_gain_ *
            ClampValue(1.0 - features.min_distance / safety_distance_max_, 0.0,
                       1.0) *
            0.05;
        curvature += repulsive_gain * ClampValue(lam_robot_y, -1.0, 1.0);
    }

    double angular_speed = ClampValue(linear_speed * curvature,
                                      -max_angular_speed_, max_angular_speed_);

    if (max_linear_acceleration_ > 0.0) {
        const double dv = max_linear_acceleration_ * control_period_;
        linear_speed = ClampValue(linear_speed, velocity.linear().x() - dv,
                                  velocity.linear().x() + dv);
    }
    if (max_angular_acceleration_ > 0.0) {
        const double dw = max_angular_acceleration_ * control_period_;
        angular_speed = ClampValue(angular_speed, velocity.angular().z() - dw,
                                   velocity.angular().z() + dw);
    }

    angular_speed = ClampValue(
        angular_speed +
            0.2 * state_tracking_weight_ *
                NormalizeAngle(lookahead_pose.theta() - pose.theta()),
        -max_angular_speed_, max_angular_speed_);

    command->mutable_linear()->set_x(linear_speed);
    command->mutable_angular()->set_z(angular_speed);
    set_message("ok");
    return Result::kSuccess;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
