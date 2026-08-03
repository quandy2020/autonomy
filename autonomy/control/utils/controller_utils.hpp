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

#pragma once

#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>

namespace autonomy {
namespace control {
namespace utils {

/**
 * @brief Find the intersection a circle and a line segment.
 * This assumes the circle is centered at the origin.
 * If no intersection is found, a floating point error will occur.
 * @param p1 first endpoint of line segment
 * @param p2 second endpoint of line segment
 * @param r radius of circle
 * @return point of intersection
 */
automsgs::msgs::geometry_msgs::Point CircleSegmentIntersection(
    const automsgs::msgs::geometry_msgs::Point& p1,
    const automsgs::msgs::geometry_msgs::Point& p2, double r);

/**
 * @brief Find the linear interpolation between two points
 * at a given distance starting from first endpoint.
 * @param p1 first endpoint of line segment
 * @param p2 second endpoint of line segment
 * @param target_dist interpolation distance from first endpoint of line segment
 * @return point of intersection
 */
automsgs::msgs::geometry_msgs::Point LinearInterpolation(
    const automsgs::msgs::geometry_msgs::Point& p1,
    const automsgs::msgs::geometry_msgs::Point& p2, const double target_dist);

/**
 * @brief Get lookahead point
 * @param lookahead_dist Optimal lookahead distance
 * @param path Current global path
 * @param interpolate_after_goal If true, interpolate the lookahead point after
 * the goal based on the orientation given by the position of the last two pose
 * of the path
 * @return Lookahead point
 */
automsgs::msgs::geometry_msgs::PoseStamped GetLookAheadPoint(
    double&, const automsgs::msgs::planning_msgs::Path&,
    const bool interpolate_after_goal = false);

}  // namespace utils
}  // namespace control
}  // namespace autonomy