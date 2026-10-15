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

/**
 * @file motion_planner.hpp
 * @brief One-step Neupan local planner over PathCache + distance encoder.
 */

#pragma once

#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <Eigen/Core>

#include "autonomy/control/controller/neupan_controller/distance_encoder.hpp"
#include "autonomy/control/controller/neupan_controller/path_cache.hpp"
#include "autonomy/control/proto/neupan_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

/**
 * @class MotionPlanner
 * @brief Differential-drive Twist from path tracking and nearest DUNE features.
 */
class MotionPlanner
{
public:
    enum class Result {
        kSuccess = 0,
        kGoalReached,
        kCollision,
        kFailure,
    };

    MotionPlanner(proto::NeupanControllerOptions options,
                  std::unique_ptr<DistanceEncoder> encoder);

    /**
     * @brief Compute one control tick.
     *
     * @param[in] pose Robot pose.
     * @param[in] velocity Current Twist.
     * @param[in] path_cache Flat path geometry (from SetPlan).
     * @param[in] obstacle_points_in_base Base-frame points, shape [2, N].
     * @param[out] command Commanded Twist.
     * @param[out] min_obstacle_distance Optional diagnostics.
     * @param[out] message Optional status text.
     */
    Result ComputeVelocityCommand(
        const automsgs::msgs::geometry_msgs::Pose2D& pose,
        const automsgs::msgs::geometry_msgs::Twist& velocity,
        const PathCache& path_cache,
        const Eigen::Matrix2Xd& obstacle_points_in_base,
        automsgs::msgs::geometry_msgs::Twist* command,
        double* min_obstacle_distance, std::string* message) const;

    int GetMaxEncoderPointCount() const {
        return max_encoder_point_count_;
    }

private:
    std::unique_ptr<DistanceEncoder> encoder_;

    double control_period_{0.1};
    double reference_linear_speed_{0.5};
    double lookahead_distance_{0.5};
    double goal_position_tolerance_{0.25};
    double collision_stop_distance_{0.1};
    double max_linear_speed_{1.0};
    double max_angular_speed_{1.0};
    double max_linear_acceleration_{0.0};
    double max_angular_acceleration_{0.0};
    double safety_distance_min_{0.1};
    double safety_distance_max_{1.0};
    double distance_slack_gain_{15.0};
    double state_tracking_weight_{1.0};
    int max_encoder_point_count_{100};
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
