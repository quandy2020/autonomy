/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file critic.hpp
 * @brief ESDF generate-and-select critic (SanD-Planner IV-D) via map/esdf.
 */

#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "autonomy/control/controller/policy_controller/types.hpp"
#include "autonomy/control/proto/policy_controller.pb.h"
#include "autonomy/map/esdf/esdf_map.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @brief Unpack flattened control points for common/math ClampedCubicBSpline, Q0 = 0.
 *
 * @param[in] raw Flattened xyz sequence.
 * @param[in] control_point_count N.
 * @return N control points in the robot frame.
 */
std::vector<Eigen::Vector3d> ControlPointsFromFlat(
    const std::vector<float>& raw, int control_point_count);

/**
 * @brief Official SanD evaluator: clearance + length + goal, lower is better.
 *
 * Clearance uses @ref map::esdf::EsdfMap::GetDistance on subsampled waypoints:
 * squared under-margin and a light penalty when distance exceeds
 * preferred_clearance, discounted along the path. Goal cost is squared
 * endpoint error. @p esdf may be null (then clearance is 0).
 *
 * @param[in] waypoints Arc-length samples in the robot frame.
 * @param[in] goal Relative 3D goal in the robot frame.
 * @param[in] options Margins, discounts, lambdas, clearance_samples.
 * @param[in] esdf Local map from @ref BuildLocalEsdfFromDepth; not owned.
 */
double EvaluateSandPlannerCost(const std::vector<Eigen::Vector3d>& waypoints,
                               const Eigen::Vector3d& goal,
                               const proto::SandPlanner& options,
                               const map::esdf::EsdfMap* esdf);

/**
 * @brief Build one local ESDF, score every candidate, emit receding Twist.
 *
 * Heading is Q1 (Q0 = 0). Linear velocity is execute_speed along that heading.
 *
 * @param[in] candidates Flattened control-point vectors.
 * @param[in] snapshot Depth + path. Depth is fused once through map/esdf.
 * @param[in] options Effective SanD-Planner options.
 * @param[out] twist Command in the robot frame.
 * @param[out] selected_heading Q1 direction (unit), for the next v_prev token.
 * @param[out] error Optional failure reason.
 * @return false when every candidate is empty.
 */
bool SelectSandPlannerTwist(
    const std::vector<std::vector<float>>& candidates,
    const ObservationSnapshot& snapshot, const proto::SandPlanner& options,
    automsgs::msgs::geometry_msgs::Twist* twist,
    Eigen::Vector3d* selected_heading, std::string* error);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
