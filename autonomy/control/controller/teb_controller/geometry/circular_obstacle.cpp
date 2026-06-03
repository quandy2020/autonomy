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

#include "autonomy/control/controller/teb_controller/geometry/circular_obstacle.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

CircularObstacle::CircularObstacle() : Obstacle(), pos_{} {}

CircularObstacle::CircularObstacle(const Point& position, double radius)
    : Obstacle(), pos_(position), radius_(radius) {}

CircularObstacle::CircularObstacle(double x, double y, double radius)
    : Obstacle(), pos_(MakePoint(x, y)), radius_(radius) {}

bool CircularObstacle::CheckCollision(const Point& point, double min_dist) const {
    return GetMinimumDistance(point) < min_dist;
}

bool CircularObstacle::CheckIntersection(const Point& line_start, const Point& line_end,
                                         double min_dist) const {
    const Point segment = line_end - line_start;
    const Point to_start = pos_ - line_start;

    double t = Dot(segment, to_start) / Dot(segment, segment);
    if (t < 0) {
        t = 0;
    } else if (t > 1) {
        t = 1;
    }
    return CheckCollision(line_start + segment * t, min_dist);
}

double CircularObstacle::GetMinimumDistance(const Point& position) const {
    return Norm(position - pos_) - radius_;
}

double CircularObstacle::GetMinimumDistance(const Point& line_start,
                                          const Point& line_end) const {
    return Distance(pos_, line_start, line_end) - radius_;
}

double CircularObstacle::GetMinimumDistance(
    const Point2dContainer& polygon) const {
    return Distance(pos_, polygon) - radius_;
}

Point CircularObstacle::GetClosestPoint(const Point& position) const {
    return pos_ + radius_ * Normalized(position - pos_);
}

double CircularObstacle::GetMinimumSpatioTemporalDistance(const Point& position,
                                                          double t) const {
    return Norm(pos_ + VelocityDisplacement(centroid_velocity_, t) - position) -
           radius_;
}

double CircularObstacle::GetMinimumSpatioTemporalDistance(const Point& line_start,
                                                          const Point& line_end,
                                                          double t) const {
    return Distance(pos_ + VelocityDisplacement(centroid_velocity_, t), line_start,
                    line_end) -
           radius_;
}

double CircularObstacle::GetMinimumSpatioTemporalDistance(
    const Point2dContainer& polygon, double t) const {
    return Distance(pos_ + VelocityDisplacement(centroid_velocity_, t), polygon) -
           radius_;
}

const Point& CircularObstacle::GetCentroid() const {
    return pos_;
}

std::complex<double> CircularObstacle::GetCentroidCplx() const {
    return std::complex<double>(pos_.x, pos_.y);
}

const Point& CircularObstacle::position() const {
    return pos_;
}

Point& CircularObstacle::position() {
    return pos_;
}

double& CircularObstacle::X() {
    return pos_.x;
}

const double& CircularObstacle::X() const {
    return pos_.x;
}

double& CircularObstacle::Y() {
    return pos_.y;
}

const double& CircularObstacle::Y() const {
    return pos_.y;
}

double& CircularObstacle::radius() {
    return radius_;
}

const double& CircularObstacle::radius() const {
    return radius_;
}

void CircularObstacle::ToPolygonMsg(
    autonomy::commsgs::geometry_msgs::Polygon& polygon) {
    polygon.points.resize(1);
    polygon.points.front().x = pos_.x;
    polygon.points.front().y = pos_.y;
    polygon.points.front().z = 0;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
