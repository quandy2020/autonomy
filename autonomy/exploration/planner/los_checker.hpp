/*
 * Copyright 2026 The Openbot Authors
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

#pragma once

#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include "autonomy/exploration/planning_env.hpp"

namespace autonomy {
namespace exploration {

/**
 * @file los_checker.hpp
 * @brief 2D Bresenham line-of-sight against Costmap2D.
 */

/**
 * @brief Check line-of-sight between two world points on the planning costmap.
 * @param env Planning environment
 * @param x0 Start x [m]
 * @param y0 Start y [m]
 * @param x1 End x [m]
 * @param y1 End y [m]
 * @param stop_at_unknown If true, NO_INFORMATION cells block LOS
 * @return true if the ray does not hit lethal / inscribed obstacles
 */
bool HasLineOfSight(const PlanningEnv& env, double x0, double y0, double x1,
                    double y1, bool stop_at_unknown = false);

/**
 * @brief Polyline length of geometry points (xy).
 * @param points Path points
 * @return Length [m]
 */
double PathLengthXy(
    const std::vector<automsgs::msgs::geometry_msgs::Point>& points);

}  // namespace exploration
}  // namespace autonomy
