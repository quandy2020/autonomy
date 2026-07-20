/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/control/controller/graceful_controller/controller.hpp"

#include <cmath>
#include <memory>
#include <mutex>

#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/common/math/math_utils.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/utils/controller_utils.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/filters/filter_values.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/convert.h"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {

namespace {

double PositiveOr(double value, double fallback) {
    return value > 0.0 ? value : fallback;
}

geometry_msgs::TransformStamped ToTf2Transform(
    const commsgs::geometry_msgs::TransformStamped& costmap_transform) {
    geometry_msgs::TransformStamped tf2_transform;
    tf2_transform.header.stamp =
        static_cast<uint64_t>(costmap_transform.header.stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(costmap_transform.header.stamp.nanosec);
    tf2_transform.header.frame_id = costmap_transform.header.frame_id;
    tf2_transform.child_frame_id = costmap_transform.child_frame_id;
    tf2_transform.transform.translation.x =
        costmap_transform.transform.translation.x;
    tf2_transform.transform.translation.y =
        costmap_transform.transform.translation.y;
    tf2_transform.transform.translation.z =
        costmap_transform.transform.translation.z;
    tf2_transform.transform.rotation.x =
        costmap_transform.transform.rotation.x;
    tf2_transform.transform.rotation.y =
        costmap_transform.transform.rotation.y;
    tf2_transform.transform.rotation.z =
        costmap_transform.transform.rotation.z;
    tf2_transform.transform.rotation.w =
        costmap_transform.transform.rotation.w;
    return tf2_transform;
}

GracefulRuntimeParams BuildRuntimeParams(
    const proto::GracefulControllerOptions& options,
    map::costmap_2d::Costmap2DWrapper& costmap_wrapper) {
    GracefulRuntimeParams params;
    auto* costmap = costmap_wrapper.getCostmap();
    const double costmap_extent =
        costmap != nullptr
            ? std::max(costmap->getSizeInMetersX(),
                       costmap->getSizeInMetersY()) /
                  2.0
            : 3.0;

    params.transform_tolerance =
        PositiveOr(options.transform_tolerance(), 0.1);
    params.max_lookahead = PositiveOr(options.max_lookahead(), 1.0);
    params.min_lookahead = PositiveOr(options.min_lookahead(), 0.25);
    params.max_robot_pose_search_dist =
        PositiveOr(options.max_robot_pose_search_dist(), costmap_extent);
    params.k_phi = PositiveOr(options.k_phi(), 2.0);
    params.k_delta = PositiveOr(options.k_delta(), 1.0);
    params.beta = PositiveOr(options.beta(), 0.4);
    params.lambda = PositiveOr(options.lambda(), 2.0);
    params.v_linear_min = PositiveOr(options.v_linear_min(), 0.1);
    params.v_linear_max = PositiveOr(options.v_linear_max(), 0.5);
    params.v_angular_max = PositiveOr(options.v_angular_max(), 1.0);
    params.v_angular_min_in_place =
        PositiveOr(options.v_angular_min_in_place(), 0.25);
    params.slowdown_radius = PositiveOr(options.slowdown_radius(), 1.5);
    params.deceleration_max = PositiveOr(options.deceleration_max(), 2.5);
    params.initial_rotation_tolerance =
        PositiveOr(options.initial_rotation_tolerance(), 0.75);
    params.rotation_scaling_factor =
        PositiveOr(options.rotation_scaling_factor(), 0.5);
    params.in_place_collision_resolution =
        PositiveOr(options.in_place_collision_resolution(), 0.1);
    params.footprint_scaling_linear_vel =
        PositiveOr(options.footprint_scaling_linear_vel(), 0.5);
    params.footprint_scaling_factor =
        PositiveOr(options.footprint_scaling_factor(), 0.25);
    params.footprint_scaling_step =
        PositiveOr(options.footprint_scaling_step(), 0.1);
    params.final_rotation_search_step =
        PositiveOr(options.final_rotation_search_step(), 0.1);
    params.obstacle_cost_margin =
        options.obstacle_cost_margin() > 0 ? options.obstacle_cost_margin() : 1;
    params.initial_rotation = options.initial_rotation();
    params.prefer_final_rotation = options.prefer_final_rotation();
    params.allow_backward = options.allow_backward();
    params.use_collision_detection = options.use_collision_detection();

    if (params.initial_rotation && params.allow_backward) {
        AWARN << "initial_rotation and allow_backward are both true; "
                 "disabling allow_backward.";
        params.allow_backward = false;
    }
    return params;
}

}  // namespace

void GracefulController::Configure(
    const proto::ControllerOptions& options, std::string name,
    std::shared_ptr<transform::Buffer> tf,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    costmap_wrapper_ = costmap_wrapper;
    tf_buffer_ = tf;
    plugin_name_ = name;
    options_ = options.graceful_controller_options();
    params_ = BuildRuntimeParams(options_, *costmap_wrapper_);
    initial_v_linear_min_ = params_.v_linear_min;
    initial_v_linear_max_ = params_.v_linear_max;
    initial_v_angular_max_ = params_.v_angular_max;

    path_handler_ = std::make_unique<PathHandler>(
        params_.transform_tolerance, tf_buffer_, costmap_wrapper_);

    control_law_ = std::make_unique<SmoothControlLaw>(
        params_.k_phi, params_.k_delta, params_.beta, params_.lambda,
        params_.slowdown_radius, params_.deceleration_max, params_.v_linear_min,
        params_.v_linear_max, params_.v_angular_max);

    if (params_.use_collision_detection && costmap_wrapper_) {
        collision_checker_ = std::make_unique<
            map::costmap_2d::FootprintCollisionChecker<
                map::costmap_2d::Costmap2D*>>(
            costmap_wrapper_->getCostmap());
    }

    const bool consider_footprint = !costmap_wrapper_->getUseRadius();
    const double max_valid_cost = consider_footprint
        ? static_cast<double>(map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        : static_cast<double>(map::costmap_2d::MAX_NON_OBSTACLE);
    if (max_valid_cost -
            static_cast<double>(params_.obstacle_cost_margin) <
        0.0) {
        AWARN << "obstacle_cost_margin (" << params_.obstacle_cost_margin
              << ") is higher than max cost (" << max_valid_cost << ")";
    }

    AINFO << "Configured Graceful Motion Controller: " << plugin_name_
          << " (lookahead=" << params_.max_lookahead
          << " collision=" << (params_.use_collision_detection ? "on" : "off")
          << ")";
}

void GracefulController::Cleanup() {
    transformed_plan_pub_.reset();
    local_plan_pub_.reset();
    motion_target_pub_.reset();
    slowdown_pub_.reset();
    collision_checker_.reset();
    path_handler_.reset();
    control_law_.reset();
}

void GracefulController::Activate() {}

void GracefulController::Deactivate() {
    AINFO << "Deactivating controller: " << plugin_name_;
}

uint32 GracefulController::ComputeVelocityCommands(
    const commsgs::geometry_msgs::PoseStamped& pose,
    const commsgs::geometry_msgs::TwistStamped& velocity,
    commsgs::geometry_msgs::TwistStamped& cmd_vel,
    common::GoalChecker* goal_checker, std::string& message) {
    control_law_->SetCurvatureConstants(
        params_.k_phi, params_.k_delta, params_.beta, params_.lambda);
    control_law_->SetSlowdownRadius(params_.slowdown_radius);
    control_law_->SetMaxDeceleration(params_.deceleration_max);
    control_law_->SetSpeedLimit(
        params_.v_linear_min, params_.v_linear_max, params_.v_angular_max);

    map::costmap_2d::Costmap2D* costmap = costmap_wrapper_->getCostmap();
    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> costmap_lock(
        *(costmap->getMutex()));

    auto transformed_plan = path_handler_->TransformGlobalPlan(
        pose, params_.max_robot_pose_search_dist);
    const auto transformed_global_plan = path_handler_->GetPlan();

    ValidateOrientations(transformed_plan.poses);

    commsgs::geometry_msgs::TransformStamped costmap_transform;
    try {
        commsgs::builtin_interfaces::Time zero_time;
        zero_time.sec = 0;
        zero_time.nanosec = 0;
        costmap_transform = tf_buffer_->lookupTransform(
            costmap_wrapper_->getGlobalFrameID(),
            costmap_wrapper_->getBaseFrameID(), zero_time, 0.1f);
    } catch (const std::exception& ex) {
        AERROR << "Could not transform " << costmap_wrapper_->getBaseFrameID()
               << " to " << costmap_wrapper_->getGlobalFrameID() << ": "
               << ex.what();
        message = "Transform error: " + std::string(ex.what());
        return 1;
    }

    double dist_to_goal = 0.0;
    for (size_t i = 1; i < transformed_plan.poses.size(); ++i) {
        dist_to_goal += map::costmap_2d::utils::euclidean_distance(
            transformed_plan.poses[i - 1], transformed_plan.poses[i]);
    }

    if (goal_checker && !transformed_global_plan.poses.empty()) {
        const auto& global_goal = transformed_global_plan.poses.back().pose;
        if (goal_checker->IsGoalXYReached(
                pose.pose, global_goal, velocity.twist,
                transformed_global_plan)) {
            double angle_to_goal = transform::tf2::getYaw(
                transformed_plan.poses.back().pose.orientation);
            size_t num_steps = static_cast<size_t>(
                fabs(angle_to_goal) / params_.in_place_collision_resolution);
            num_steps = std::max(static_cast<size_t>(1), num_steps);
            bool collision_free = true;
            const auto tf2_transform = ToTf2Transform(costmap_transform);
            for (size_t i = 1; i <= num_steps; ++i) {
                const double step =
                    static_cast<double>(i) / static_cast<double>(num_steps);
                const double yaw = step * angle_to_goal;
                commsgs::geometry_msgs::PoseStamped next_pose;
                next_pose.header.frame_id = costmap_wrapper_->getBaseFrameID();
                next_pose.pose.orientation =
                    map::costmap_2d::utils::OrientationAroundZAxis(yaw);
                commsgs::geometry_msgs::PoseStamped costmap_pose;
                transform::tf2::doTransform(next_pose, costmap_pose,
                                              tf2_transform);
                if (params_.use_collision_detection &&
                    InCollision(costmap_pose.pose.position.x,
                                costmap_pose.pose.position.y,
                                transform::tf2::getYaw(
                                    costmap_pose.pose.orientation))) {
                    collision_free = false;
                    break;
                }
            }
            if (collision_free) {
                cmd_vel.twist = RotateToTarget(angle_to_goal);
                message = "";
                return 0;
            }
        }
    }

    commsgs::planning_msgs::Path local_plan;
    commsgs::geometry_msgs::PoseStamped target_pose;

    double dist_to_target;
    std::vector<double> target_distances;
    ComputeDistanceAlongPath(transformed_plan.poses, target_distances);

    bool is_first_iteration = true;
    for (int i = static_cast<int>(transformed_plan.poses.size()) - 1; i >= 0;
         --i) {
        if (is_first_iteration) {
            dist_to_target = params_.max_lookahead;
            target_pose = control::utils::GetLookAheadPoint(
                dist_to_target, transformed_plan, false);
            is_first_iteration = false;
        } else {
            dist_to_target = target_distances[static_cast<size_t>(i)];
            target_pose = transformed_plan.poses[static_cast<size_t>(i)];
        }

        if (ValidateTargetPoseOnApproach(
                target_pose, dist_to_target, dist_to_goal, local_plan,
                costmap_transform, cmd_vel) ||
            ValidateTargetPose(target_pose, dist_to_target, local_plan,
                               costmap_transform, cmd_vel)) {
            if (motion_target_pub_) {
                motion_target_pub_->Write(target_pose);
            }
            auto slowdown_marker =
                CreateSlowdownMarker(target_pose, params_.slowdown_radius);
            if (slowdown_pub_) {
                slowdown_pub_->Write(slowdown_marker);
            }
            local_plan.header = transformed_plan.header;
            if (local_plan_pub_) {
                local_plan_pub_->Write(local_plan);
            }
            message = "";
            return 0;
        }
    }

    message = "Collision detected in trajectory";
    return 1;
}

bool GracefulController::IsGoalReached(double dist_tolerance,
                                       double angle_tolerance) {
    (void)dist_tolerance;
    (void)angle_tolerance;
    return false;
}

void GracefulController::SetPlan(const commsgs::planning_msgs::Path& path) {
    path_handler_->SetPlan(path);
    do_initial_rotation_ = true;
    safe_approach_angle_.reset();
}

void GracefulController::Reset() {
    do_initial_rotation_ = true;
    safe_approach_angle_.reset();
}

void GracefulController::SetSpeedLimit(const double& speed_limit,
                                       const bool& percentage) {
    if (speed_limit == map::costmap_2d::NO_SPEED_LIMIT) {
        params_.v_linear_max = initial_v_linear_max_;
        params_.v_angular_max = initial_v_angular_max_;
    } else if (percentage) {
        params_.v_linear_max = std::max(
            initial_v_linear_max_ * speed_limit / 100.0, params_.v_linear_min);
        params_.v_angular_max =
            initial_v_angular_max_ * speed_limit / 100.0;
    } else {
        params_.v_linear_max =
            std::max(speed_limit, params_.v_linear_min);
        params_.v_angular_max =
            initial_v_angular_max_ *
            (initial_v_linear_max_ > 0.0 ? speed_limit / initial_v_linear_max_
                                         : 1.0);
    }
    if (control_law_) {
        control_law_->SetSpeedLimit(
            params_.v_linear_min, params_.v_linear_max, params_.v_angular_max);
    }
}

bool GracefulController::ValidateTargetPose(
    commsgs::geometry_msgs::PoseStamped& target_pose, double dist_to_target,
    commsgs::planning_msgs::Path& trajectory,
    commsgs::geometry_msgs::TransformStamped& costmap_transform,
    commsgs::geometry_msgs::TwistStamped& cmd_vel) {
    if (dist_to_target > params_.max_lookahead) {
        return false;
    }

    bool reversing = false;
    if (params_.allow_backward && target_pose.pose.position.x < 0.0) {
        reversing = true;
        target_pose.pose.orientation =
            map::costmap_2d::utils::OrientationAroundZAxis(
                transform::tf2::getYaw(target_pose.pose.orientation) + M_PI);
    }

    double sim_linear_velocity = params_.v_linear_max;
    do {
        control_law_->SetSpeedLimit(params_.v_linear_min, sim_linear_velocity,
                                    params_.v_angular_max);
        if (SimulateTrajectory(target_pose, costmap_transform, trajectory,
                               cmd_vel, reversing)) {
            return true;
        }
        sim_linear_velocity -= params_.footprint_scaling_step;
    } while (sim_linear_velocity >= params_.footprint_scaling_linear_vel);

    return false;
}

bool GracefulController::ValidateTargetPoseOnApproach(
    commsgs::geometry_msgs::PoseStamped& target_pose, double dist_to_target,
    double dist_to_goal, commsgs::planning_msgs::Path& trajectory,
    commsgs::geometry_msgs::TransformStamped& costmap_transform,
    commsgs::geometry_msgs::TwistStamped& cmd_vel) {
    if (dist_to_goal >= params_.max_lookahead || !params_.prefer_final_rotation) {
        return false;
    }

    const double yaw =
        std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);
    target_pose.pose.orientation =
        map::costmap_2d::utils::OrientationAroundZAxis(yaw);

    if (!ValidateTargetPose(target_pose, dist_to_target, trajectory,
                            costmap_transform, cmd_vel)) {
        return false;
    }

    const bool consider_footprint = !costmap_wrapper_->getUseRadius();
    const double max_valid_cost = consider_footprint
        ? static_cast<double>(map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        : static_cast<double>(map::costmap_2d::MAX_NON_OBSTACLE);
    const double safety_threshold =
        max_valid_cost - static_cast<double>(params_.obstacle_cost_margin);
    if (GetMaxCost(trajectory, costmap_transform) >= safety_threshold) {
        FindBestApproachTrajectory(target_pose, dist_to_target,
                                   costmap_transform, safety_threshold,
                                   trajectory, cmd_vel);
    }
    return true;
}

bool GracefulController::SimulateTrajectory(
    const commsgs::geometry_msgs::PoseStamped& motion_target,
    const commsgs::geometry_msgs::TransformStamped& costmap_transform,
    commsgs::planning_msgs::Path& trajectory,
    commsgs::geometry_msgs::TwistStamped& cmd_vel, bool backward) {
    trajectory.poses.clear();

    commsgs::geometry_msgs::PoseStamped next_pose;
    next_pose.header.frame_id = costmap_wrapper_->getBaseFrameID();
    next_pose.pose.orientation.w = 1.0;

    bool sim_initial_rotation = do_initial_rotation_ && params_.initial_rotation;
    const double angle_to_target = std::atan2(motion_target.pose.position.y,
                                              motion_target.pose.position.x);
    if (fabs(angle_to_target) < params_.initial_rotation_tolerance) {
        sim_initial_rotation = false;
        do_initial_rotation_ = false;
    }

    double distance = std::numeric_limits<double>::max();
    const double resolution = costmap_wrapper_->getCostmap()->getResolution();
    const double dt = (params_.v_linear_max > 0.0)
                          ? resolution / params_.v_linear_max
                          : 0.0;
    const unsigned int max_iter = 3 * static_cast<unsigned int>(
                                        std::hypot(motion_target.pose.position.x,
                                                   motion_target.pose.position.y) /
                                        resolution);
    const auto tf2_transform = ToTf2Transform(costmap_transform);

    do {
        if (sim_initial_rotation) {
            double next_pose_yaw =
                transform::tf2::getYaw(next_pose.pose.orientation);
            const auto cmd = RotateToTarget(angle_to_target - next_pose_yaw);
            if (trajectory.poses.empty()) {
                cmd_vel.twist = cmd;
            }
            if (fabs(angle_to_target - next_pose_yaw) <
                params_.initial_rotation_tolerance) {
                sim_initial_rotation = false;
            }
            next_pose_yaw += cmd_vel.twist.angular.z * dt;
            next_pose.pose.orientation =
                map::costmap_2d::utils::OrientationAroundZAxis(next_pose_yaw);
        } else {
            if (trajectory.poses.empty() && control_law_) {
                cmd_vel.twist = control_law_->CalculateRegularVelocity(
                    motion_target.pose, next_pose.pose, backward);
            }
            if (control_law_) {
                next_pose.pose = control_law_->CalculateNextPose(
                    dt, motion_target.pose, next_pose.pose, backward);
            }
        }

        trajectory.poses.push_back(next_pose);

        double footprint_scaling = 1.0;
        if (cmd_vel.twist.linear.x > params_.footprint_scaling_linear_vel) {
            double ratio =
                params_.v_linear_max - params_.footprint_scaling_linear_vel;
            if (ratio > 0.0) {
                ratio = (cmd_vel.twist.linear.x -
                         params_.footprint_scaling_linear_vel) /
                        ratio;
                footprint_scaling +=
                    ratio * params_.footprint_scaling_factor;
            }
        }

        commsgs::geometry_msgs::PoseStamped global_pose;
        transform::tf2::doTransform(next_pose, global_pose, tf2_transform);
        if (params_.use_collision_detection &&
            InCollision(global_pose.pose.position.x,
                       global_pose.pose.position.y,
                       transform::tf2::getYaw(global_pose.pose.orientation),
                       footprint_scaling)) {
            return false;
        }

        distance = map::costmap_2d::utils::euclidean_distance(
            motion_target.pose, next_pose.pose);
    } while (distance > resolution && trajectory.poses.size() < max_iter);

    return true;
}

commsgs::geometry_msgs::Twist GracefulController::RotateToTarget(
    double angle_to_target) {
    commsgs::geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z =
        params_.rotation_scaling_factor * angle_to_target * params_.v_angular_max;
    vel.angular.z = std::copysign(1.0, vel.angular.z) *
                    std::max(static_cast<double>(std::abs(vel.angular.z)),
                             params_.v_angular_min_in_place);
    return vel;
}

double GracefulController::GetMaxCost(
    const commsgs::planning_msgs::Path& path,
    commsgs::geometry_msgs::TransformStamped& costmap_transform) {
    double max_cost = 0.0;
    const auto tf2_transform = ToTf2Transform(costmap_transform);
    auto* costmap = costmap_wrapper_->getCostmap();

    for (const auto& pose : path.poses) {
        commsgs::geometry_msgs::PoseStamped costmap_pose;
        transform::tf2::doTransform(pose, costmap_pose, tf2_transform);
        unsigned int mx, my;
        if (costmap->worldToMap(costmap_pose.pose.position.x,
                                costmap_pose.pose.position.y, mx, my)) {
            max_cost = std::max(max_cost,
                                static_cast<double>(
                                    collision_checker_->pointCost(mx, my)));
        }
    }
    return max_cost;
}

bool GracefulController::InCollision(const double& x, const double& y,
                                   const double& theta,
                                   double inflation_scale) {
    if (!collision_checker_) {
        return false;
    }

    unsigned int mx, my;
    if (!costmap_wrapper_->getCostmap()->worldToMap(x, y, mx, my)) {
        AWARN << "The path is not in the costmap. Cannot check for collisions.";
        return false;
    }

    if (inflation_scale < 1.0) {
        throw common::NoValidControl("Inflation ratio less than 1.0");
    }

    const bool is_tracking_unknown =
        costmap_wrapper_->getLayeredCostmap()->isTrackingUnknown();
    const bool consider_footprint = !costmap_wrapper_->getUseRadius();

    double footprint_cost = 0.0;
    if (consider_footprint) {
        auto spec = costmap_wrapper_->getRobotFootprint();
        if (spec.size() > 3) {
            for (auto& point : spec) {
                point.x *= inflation_scale;
                point.y *= inflation_scale;
            }
        }
        footprint_cost =
            collision_checker_->footprintCostAtPose(x, y, theta, spec);
    } else {
        footprint_cost = collision_checker_->pointCost(mx, my);
    }

    switch (static_cast<unsigned char>(footprint_cost)) {
        case map::costmap_2d::LETHAL_OBSTACLE:
            return true;
        case map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
            return !consider_footprint;
        case map::costmap_2d::NO_INFORMATION:
            return !is_tracking_unknown;
        default:
            return false;
    }
}

bool GracefulController::FindBestApproachTrajectory(
    commsgs::geometry_msgs::PoseStamped& target_pose, double dist_to_target,
    commsgs::geometry_msgs::TransformStamped& costmap_transform,
    double safety_cost, commsgs::planning_msgs::Path& best_trajectory,
    commsgs::geometry_msgs::TwistStamped& best_cmd_vel) {
    bool found_valid = false;
    double best_eta = std::numeric_limits<double>::max();
    const int num_steps = static_cast<int>(
        2.0 * M_PI / params_.final_rotation_search_step);

    for (int i = 0; i < num_steps; ++i) {
        double angle =
            static_cast<double>(i) * params_.final_rotation_search_step;
        if (safe_approach_angle_.has_value()) {
            angle += safe_approach_angle_.value();
        }

        auto candidate_pose = target_pose;
        candidate_pose.pose.orientation =
            map::costmap_2d::utils::OrientationAroundZAxis(angle);

        commsgs::planning_msgs::Path candidate_path = best_trajectory;
        commsgs::geometry_msgs::TwistStamped candidate_cmd_vel = best_cmd_vel;

        if (!ValidateTargetPose(candidate_pose, dist_to_target, candidate_path,
                                costmap_transform, candidate_cmd_vel)) {
            continue;
        }

        const double candidate_cost =
            GetMaxCost(candidate_path, costmap_transform);

        bool reversing = false;
        if (params_.allow_backward && target_pose.pose.position.x < 0.0) {
            reversing = true;
        }

        double eta = 0.0;
        for (size_t j = 1; j < candidate_path.poses.size(); ++j) {
            const auto& current_pose = candidate_path.poses[j - 1];
            const auto& next_pose = candidate_path.poses[j];
            const auto cmd = control_law_->CalculateRegularVelocity(
                candidate_pose.pose, current_pose.pose, reversing);
            double speed = std::abs(cmd.linear.x);
            speed = std::max(speed, 1e-3);
            const double step_dist =
                map::costmap_2d::utils::euclidean_distance(current_pose.pose,
                                                           next_pose.pose);
            eta += step_dist / speed;
        }

        if (eta < best_eta) {
            best_eta = eta;
            if (candidate_cost < safety_cost) {
                best_trajectory = candidate_path;
                best_cmd_vel = candidate_cmd_vel;
                target_pose = candidate_pose;
                found_valid = true;
                if (safe_approach_angle_.value_or(1e3) == angle) {
                    break;
                }
                safe_approach_angle_ = angle;
            }
        }
    }

    return found_valid;
}

void GracefulController::ComputeDistanceAlongPath(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& poses,
    std::vector<double>& distances) {
    distances.resize(poses.size());
    double d = std::hypot(poses[0].pose.position.x, poses[0].pose.position.y);
    distances[0] = d;
    for (size_t i = 1; i < poses.size(); ++i) {
        d += map::costmap_2d::utils::euclidean_distance(poses[i - 1], poses[i]);
        distances[i] = d;
    }
}

void GracefulController::ValidateOrientations(
    std::vector<commsgs::geometry_msgs::PoseStamped>& path) {
    if (path.size() < 3) {
        return;
    }

    const double initial_yaw = transform::tf2::getYaw(path[1].pose.orientation);
    for (size_t i = 2; i < path.size() - 1; ++i) {
        const double this_yaw = transform::tf2::getYaw(path[i].pose.orientation);
        if (std::abs(autonomy::common::math::AngleDiff(this_yaw, initial_yaw)) >
            1e-6) {
            return;
        }
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        const double dx =
            path[i + 1].pose.position.x - path[i].pose.position.x;
        const double dy =
            path[i + 1].pose.position.y - path[i].pose.position.y;
        const double yaw = std::atan2(dy, dx);
        path[i].pose.orientation =
            map::costmap_2d::utils::OrientationAroundZAxis(yaw);
    }
}

}  // namespace controller
}  // namespace control
}  // namespace autonomy

CLASS_LOADER_REGISTER_CLASS(autonomy::control::controller::GracefulController,
                            autonomy::control::common::ControllerInterface)
