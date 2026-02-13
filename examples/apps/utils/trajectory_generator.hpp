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

#include <string>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace examples {
namespace utils {

/**
 * @brief Supported trajectory shapes for path following (e.g. by controller).
 */
enum class TrajectoryShape {
  /** Circle: center (cx, cy), radius, optional start angle */
  kCircle,
  /** Square: center (cx, cy), half side length, optional start corner */
  kSquare,
  /** Figure-8 (lemniscate): center (cx, cy), scale (a), orientation */
  kFigure8,
  /** Lemniscate of Bernoulli: same as figure-8 with different parameterization */
  kLemniscate,
};

/**
 * @brief Parameters common to all trajectory types.
 */
struct TrajectoryOptions {
  /** Frame id for Path.header and each PoseStamped (e.g. "map", "odom"). */
  std::string frame_id = "map";
  /** Number of poses to sample along the trajectory. */
  int num_points = 100;
  /** Whether to close the path (first point == last point for closed curves). */
  bool closed = true;
};

/**
 * @brief Parameters for circle trajectory.
 */
struct CircleOptions : TrajectoryOptions {
  double center_x = 0.0;
  double center_y = 0.0;
  double radius = 2.0;
  /** Start angle in radians (0 = +x, counter-clockwise). */
  double start_angle_rad = 0.0;
};

/**
 * @brief Parameters for square trajectory.
 */
struct SquareOptions : TrajectoryOptions {
  double center_x = 0.0;
  double center_y = 0.0;
  /** Half side length (distance from center to edge). */
  double half_side = 2.0;
  /** Start from corner 0..3 (0 = right-top, counter-clockwise). */
  int start_corner = 0;
};

/**
 * @brief Parameters for figure-8 (lemniscate) trajectory.
 * Parametric: x = cx + a*cos(t), y = cy + a*sin(t)*cos(t) or similar.
 */
struct Figure8Options : TrajectoryOptions {
  double center_x = 0.0;
  double center_y = 0.0;
  /** Scale (size) of the figure-8 in meters. */
  double scale = 2.0;
  /** Orientation of the figure-8 in radians (rotation around z). */
  double orientation_rad = 0.0;
};

/**
 * @brief Generate a Path for controller to follow.
 *
 * @param shape Trajectory shape (circle, square, figure-8).
 * @param path Output path (header + poses); previous content is cleared.
 */
void GenerateTrajectory(TrajectoryShape shape,
                       autonomy::commsgs::planning_msgs::Path& path);

/**
 * @brief Generate a circle trajectory.
 */
void GenerateCircle(const CircleOptions& opts,
                    autonomy::commsgs::planning_msgs::Path& path);

/**
 * @brief Generate a square trajectory.
 */
void GenerateSquare(const SquareOptions& opts,
                    autonomy::commsgs::planning_msgs::Path& path);

/**
 * @brief Generate a figure-8 (lemniscate) trajectory.
 */
void GenerateFigure8(const Figure8Options& opts,
                     autonomy::commsgs::planning_msgs::Path& path);

/**
 * @brief Create a quaternion from yaw (rotation around z, radians).
 */
autonomy::commsgs::geometry_msgs::Quaternion QuaternionFromYaw(double yaw_rad);

/**
 * @brief Create a PoseStamped at (x, y) with heading yaw_rad.
 */
autonomy::commsgs::geometry_msgs::PoseStamped MakePoseStamped(
    const std::string& frame_id, double x, double y, double yaw_rad);

}  // namespace utils
}  // namespace examples
}  // namespace autonomy
