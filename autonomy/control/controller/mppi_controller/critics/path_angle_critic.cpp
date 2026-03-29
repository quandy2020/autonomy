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

#include "autonomy/control/controller/mppi_controller/critics/path_angle_critic.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace critics {

void PathAngleCritic::initialize() {
    if (!options_) {
        AWARN << "Options not set, using defaults";
        power_ = 1;
        weight_ = 2.0f;
        offset_from_furthest_ = 4;
        threshold_to_consider_ = 0.5f;
        max_angle_to_furthest_ = 1.0f;
        mode_ = PathAngleMode::FORWARD_PREFERENCE;
        reversing_allowed_ = true;
        return;
    }

    // Check if reversing is allowed based on vx_min
    float vx_min = options_->vx_min();
    if (fabs(vx_min) < 1e-6f) {  // zero
        reversing_allowed_ = false;
    } else if (vx_min < 0.0f) {  // reversing possible
        reversing_allowed_ = true;
    } else {
        reversing_allowed_ = false;
    }

    // Load from proto options
    if (options_->has_path_angle_critic()) {
        const auto& critic = options_->path_angle_critic();
        enabled_ = critic.enabled();
        power_ = critic.cost_power();
        weight_ = static_cast<float>(critic.cost_weight());
        offset_from_furthest_ =
            static_cast<size_t>(critic.offset_from_furthest());
        threshold_to_consider_ =
            static_cast<float>(critic.threshold_to_consider());
        max_angle_to_furthest_ =
            static_cast<float>(critic.max_angle_to_furthest());

        // Map forward_preference to mode
        if (critic.forward_preference()) {
            mode_ = PathAngleMode::FORWARD_PREFERENCE;
        } else {
            mode_ = PathAngleMode::NO_DIRECTIONAL_PREFERENCE;
        }
    } else {
        enabled_ = true;
        power_ = 1;
        weight_ = 2.0f;                             // Default
        offset_from_furthest_ = 4;                  // Default
        threshold_to_consider_ = 0.5f;              // Default
        max_angle_to_furthest_ = 1.0f;              // Default
        mode_ = PathAngleMode::FORWARD_PREFERENCE;  // Default
    }

    if (!reversing_allowed_ &&
        mode_ == PathAngleMode::NO_DIRECTIONAL_PREFERENCE) {
        mode_ = PathAngleMode::FORWARD_PREFERENCE;
        AWARN << "Path angle mode set to no directional preference, but "
              << "controller's settings don't allow for reversing! "
              << "Setting mode to forward preference.";
    }

    AINFO << "PathAngleCritic instantiated with " << power_ << " power and "
          << weight_ << " weight. Mode set to: " << modeToStr(mode_);
}

void PathAngleCritic::score(CriticData& data) {
    if (!enabled_) {
        return;
    }

    commsgs::geometry_msgs::Pose goal =
        tools::getCriticGoal(data, enforce_path_inversion_);

    if (tools::withinPositionGoalTolerance(threshold_to_consider_,
                                           data.state.pose.pose, goal)) {
        return;
    }

    tools::setPathFurthestPointIfNotSet(data);
    auto offsetted_idx =
        std::min(*data.furthest_reached_path_point + offset_from_furthest_,
                 static_cast<size_t>(data.path.x.size()) - 1);

    const float goal_x = data.path.x(offsetted_idx);
    const float goal_y = data.path.y(offsetted_idx);
    const float goal_yaw = data.path.yaws(offsetted_idx);
    const commsgs::geometry_msgs::Pose& pose = data.state.pose.pose;

    switch (mode_) {
        case PathAngleMode::FORWARD_PREFERENCE:
            if (tools::posePointAngle(pose, goal_x, goal_y, true) <
                max_angle_to_furthest_) {
                return;
            }
            break;
        case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
            if (tools::posePointAngle(pose, goal_x, goal_y, false) <
                max_angle_to_furthest_) {
                return;
            }
            break;
        case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
            if (tools::posePointAngle(pose, goal_x, goal_y, goal_yaw) <
                max_angle_to_furthest_) {
                return;
            }
            break;
        default:
            throw common::ControllerException("Invalid path angle mode!");
    }

    int last_idx = data.trajectories.y.cols() - 1;
    auto diff_y = goal_y - data.trajectories.y.col(last_idx);
    auto diff_x = goal_x - data.trajectories.x.col(last_idx);
    auto yaws_between_points =
        diff_y
            .binaryExpr(diff_x, [&](const float& y,
                                    const float& x) { return atan2f(y, x); })
            .eval();

    switch (mode_) {
        case PathAngleMode::FORWARD_PREFERENCE: {
            auto last_yaws = data.trajectories.yaws.col(last_idx);
            auto yaws =
                tools::shortest_angular_distance(last_yaws, yaws_between_points)
                    .abs();
            if (power_ > 1u) {
                data.costs += (yaws * weight_).pow(power_);
            } else {
                data.costs += yaws * weight_;
            }
            return;
        }
        case PathAngleMode::NO_DIRECTIONAL_PREFERENCE: {
            auto last_yaws = data.trajectories.yaws.col(last_idx);
            auto yaws_between_points_corrected =
                tools::normalize_yaws_between_points(last_yaws,
                                                     yaws_between_points);
            auto corrected_yaws = tools::shortest_angular_distance(
                                      last_yaws, yaws_between_points_corrected)
                                      .abs();
            if (power_ > 1u) {
                data.costs += (corrected_yaws * weight_).pow(power_);
            } else {
                data.costs += corrected_yaws * weight_;
            }
            return;
        }
        case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS: {
            auto last_yaws = data.trajectories.yaws.col(last_idx);
            auto yaws_between_points_corrected =
                tools::normalize_yaws_between_points(goal_yaw,
                                                     yaws_between_points);
            auto corrected_yaws = tools::shortest_angular_distance(
                                      last_yaws, yaws_between_points_corrected)
                                      .abs();
            if (power_ > 1u) {
                data.costs += (corrected_yaws * weight_).pow(power_);
            } else {
                data.costs += corrected_yaws * weight_;
            }
            return;
        }
    }
}

}  // namespace critics
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(
    autonomy::control::controller::mppi_controller::critics::PathAngleCritic,
    autonomy::control::controller::mppi_controller::CriticFunction)