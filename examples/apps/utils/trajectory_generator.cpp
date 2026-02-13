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

#include "autonomy/examples/apps/utils/trajectory_generator.hpp"

#include <cmath>
#include <string>

namespace autonomy {
namespace examples {
namespace utils {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

}  // namespace

autonomy::commsgs::geometry_msgs::Quaternion QuaternionFromYaw(double yaw_rad) {
  autonomy::commsgs::geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw_rad * 0.5);
  q.w = std::cos(yaw_rad * 0.5);
  return q;
}

autonomy::commsgs::geometry_msgs::PoseStamped MakePoseStamped(
    const std::string& frame_id, double x, double y, double yaw_rad) {
  autonomy::commsgs::geometry_msgs::PoseStamped ps;
  ps.header.frame_id = frame_id;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0.0;
  ps.pose.orientation = QuaternionFromYaw(yaw_rad);
  return ps;
}

void GenerateCircle(const CircleOptions& opts,
                    autonomy::commsgs::planning_msgs::Path& path) {
  path.poses.clear();
  path.header.frame_id = opts.frame_id;

  const int n = opts.closed ? opts.num_points : (opts.num_points + 1);
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / opts.num_points;
    const double angle = opts.start_angle_rad + t * kTwoPi;
    const double x = opts.center_x + opts.radius * std::cos(angle);
    const double y = opts.center_y + opts.radius * std::sin(angle);
    // Tangent direction: (-sin(angle), cos(angle)), yaw = angle + pi/2 in standard
    // convention (x forward); here we use yaw = angle + pi/2 so robot faces along tangent.
    const double yaw = angle + kPi * 0.5;
    path.poses.push_back(MakePoseStamped(opts.frame_id, x, y, yaw));
  }
}

void GenerateSquare(const SquareOptions& opts,
                   autonomy::commsgs::planning_msgs::Path& path) {
  path.poses.clear();
  path.header.frame_id = opts.frame_id;

  const double h = opts.half_side;
  const int corner = ((opts.start_corner % 4) + 4) % 4;
  // Corners in order (counter-clockwise): 0=(+h,+h), 1=(-h,+h), 2=(-h,-h), 3=(+h,-h)
  double cx = opts.center_x;
  double cy = opts.center_y;

  auto add_edge = [&path, &opts](double x0, double y0, double x1, double y1,
                                  int num_pts) {
    for (int i = 0; i <= num_pts; ++i) {
      if (i == 0 && path.poses.size() > 0) continue;  // avoid duplicate at joint
      const double s = static_cast<double>(i) / num_pts;
      const double x = x0 + s * (x1 - x0);
      const double y = y0 + s * (y1 - y0);
      const double dx = x1 - x0;
      const double dy = y1 - y0;
      const double yaw = std::atan2(dy, dx) - kPi * 0.5;  // tangent along edge
      path.poses.push_back(MakePoseStamped(opts.frame_id, x, y, yaw));
    }
  };

  const int pts_per_edge = (opts.num_points / 4) + 1;
  if (opts.closed) {
    for (int e = 0; e < 4; ++e) {
      int c0 = (corner + e) % 4;
      int c1 = (corner + e + 1) % 4;
      double x0 = cx + (c0 == 0 || c0 == 3 ? h : -h);
      double y0 = cy + (c0 == 0 || c0 == 1 ? h : -h);
      double x1 = cx + (c1 == 0 || c1 == 3 ? h : -h);
      double y1 = cy + (c1 == 0 || c1 == 1 ? h : -h);
      add_edge(x0, y0, x1, y1, pts_per_edge);
    }
  } else {
    add_edge(cx + h, cy + h, cx - h, cy + h, pts_per_edge);
    add_edge(cx - h, cy + h, cx - h, cy - h, pts_per_edge);
    add_edge(cx - h, cy - h, cx + h, cy - h, pts_per_edge);
    add_edge(cx + h, cy - h, cx + h, cy + h, pts_per_edge);
  }
}

void GenerateFigure8(const Figure8Options& opts,
                    autonomy::commsgs::planning_msgs::Path& path) {
  path.poses.clear();
  path.header.frame_id = opts.frame_id;

  // Lemniscate of Bernoulli: x = a*cos(t)/(1+sin^2(t)), y = a*sin(t)*cos(t)/(1+sin^2(t))
  // Or simpler figure-8: x = a*cos(t), y = a*sin(t)*cos(t) in [-pi, pi]
  const double a = opts.scale;
  const double co = std::cos(opts.orientation_rad);
  const double so = std::sin(opts.orientation_rad);

  const int n = opts.closed ? opts.num_points : (opts.num_points + 1);
  for (int i = 0; i < n; ++i) {
    const double t = -kPi + (static_cast<double>(i) / opts.num_points) * kTwoPi;
    const double cx_local = a * std::cos(t);
    const double cy_local = a * std::sin(t) * std::cos(t);
    const double x = opts.center_x + (cx_local * co - cy_local * so);
    const double y = opts.center_y + (cx_local * so + cy_local * co);
    // Tangent: dx/dt = -a*sin(t), dy/dt = a*cos(2t) -> yaw
    const double dx = -a * std::sin(t);
    const double dy = a * std::cos(2.0 * t);
    const double dx_w = dx * co - dy * so;
    const double dy_w = dx * so + dy * co;
    const double yaw = std::atan2(dy_w, dx_w) - kPi * 0.5;
    path.poses.push_back(MakePoseStamped(opts.frame_id, x, y, yaw));
  }
}

void GenerateTrajectory(TrajectoryShape shape,
                        autonomy::commsgs::planning_msgs::Path& path) {
  switch (shape) {
    case TrajectoryShape::kCircle: {
      CircleOptions opts;
      GenerateCircle(opts, path);
      break;
    }
    case TrajectoryShape::kSquare: {
      SquareOptions opts;
      GenerateSquare(opts, path);
      break;
    }
    case TrajectoryShape::kFigure8:
    case TrajectoryShape::kLemniscate: {
      Figure8Options opts;
      GenerateFigure8(opts, path);
      break;
    }
  }
}

}  // namespace utils
}  // namespace examples
}  // namespace autonomy
