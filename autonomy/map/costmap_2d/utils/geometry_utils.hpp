/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include "autonomy/transform/tf2/convert.h"
#include "autonomy/transform/tf2/LinearMath/Quaternion.h"
#include "autonomy/transform/tf2/LinearMath/Transform.h"
#include "autonomy/transform/tf2/LinearMath/Vector3.h"

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {
namespace utils {

 /**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline commsgs::geometry_msgs::Quaternion OrientationAroundZAxis(double angle)
{
    transform::tf2::Quaternion q;
    q.setRPY(0, 0, angle);  // void returning function
    return {q.x(), q.y(), q.z(), q.w()};
}

/**
 * @brief Get the euclidean distance between 2 geometry_msgs::Points
 * @param pos1 First point
 * @param pos1 Second point
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const commsgs::geometry_msgs::Point& pos1,
  const commsgs::geometry_msgs::Point& pos2,
  const bool is_3d = false)
{
    double dx = pos1.x - pos2.x;
    double dy = pos1.y - pos2.y;

    if (is_3d) {
        double dz = pos1.z - pos2.z;
        return std::hypot(dx, dy, dz);
    }

    return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double euclidean distance
 */
inline double euclidean_distance(
  const commsgs::geometry_msgs::Pose& pos1,
  const commsgs::geometry_msgs::Pose& pos2,
  const bool is_3d = false)
{
    double dx = pos1.position.x - pos2.position.x;
    double dy = pos1.position.y - pos2.position.y;

    if (is_3d) {
        double dz = pos1.position.z - pos2.position.z;
        return std::hypot(dx, dy, dz);
    }

    return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const commsgs::geometry_msgs::PoseStamped& pos1,
  const commsgs::geometry_msgs::PoseStamped& pos2,
  const bool is_3d = false)
{
    return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Pose2D
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(
  const commsgs::geometry_msgs::Pose2D& pos1,
  const commsgs::geometry_msgs::Pose2D& pos2)
{
    double dx = pos1.x - pos2.x;
    double dy = pos1.y - pos2.y;
    return std::hypot(dx, dy);
}

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
    if (begin == end) {
        return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it) {
        auto comp = getCompareVal(*it);
        if (comp <= lowest) {
            lowest = comp;
            lowest_it = it;
        }
    }
    return lowest_it;
}

/**
 * Find first element in iterator that is greater integrated distance than comparevalue
 */
template<typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end, Getter getCompareVal)
{
    if (begin == end) {
        return end;
    }
    Getter dist = 0.0;
    for (Iter it = begin; it != end - 1; it++) {
        dist += euclidean_distance(*it, *(it + 1));
            if (dist > getCompareVal) {
            return it + 1;
        }
    }
    return end;
}

/**
 * @brief Calculate the length of the provided path, starting at the provided index
 * @param path Path containing the poses that are planned
 * @param start_index Optional argument specifying the starting index for
 * the calculation of path length. Provide this if you want to calculate length of a
 * subset of the path.
 * @return double Path length
 */
inline double calculate_path_length(const commsgs::planning_msgs::Path& path, size_t start_index = 0)
{
    if (start_index + 1 >= path.poses.size()) {
        return 0.0;
    }
    double path_length = 0.0;
    for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
        path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
    }
    return path_length;
}

}  // namespace utils
}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy

