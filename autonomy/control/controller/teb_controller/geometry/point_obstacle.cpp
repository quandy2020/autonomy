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

#include "autonomy/control/controller/teb_controller/geometry/point_obstacle.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

PointObstacle::PointObstacle() : Obstacle(), pos_{} {}

PointObstacle::PointObstacle(const Point& position) : Obstacle(), pos_(position) {}

PointObstacle::PointObstacle(double x, double y) : Obstacle(), pos_(MakePoint(x, y)) {}

bool PointObstacle::CheckCollision(const Point& point, double min_dist) const {
    return GetMinimumDistance(point) < min_dist;
}

bool PointObstacle::CheckIntersection(const Point& line_start, const Point& line_end,
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

double PointObstacle::GetMinimumDistance(const Point& position) const {
    return Norm(position - pos_);
}

double PointObstacle::GetMinimumDistance(const Point& line_start,
                                         const Point& line_end) const {
    return Distance(pos_, line_start, line_end);
}

double PointObstacle::GetMinimumDistance(const Point2dContainer& polygon) const {
    return Distance(pos_, polygon);
}

Point PointObstacle::GetClosestPoint(const Point& /*position*/) const {
    return pos_;
}

double PointObstacle::GetMinimumSpatioTemporalDistance(const Point& position,
                                                       double t) const {
    return Norm(pos_ + VelocityDisplacement(centroid_velocity_, t) - position);
}

double PointObstacle::GetMinimumSpatioTemporalDistance(const Point& line_start,
                                                       const Point& line_end,
                                                       double t) const {
    return Distance(pos_ + VelocityDisplacement(centroid_velocity_, t), line_start,
                    line_end);
}

double PointObstacle::GetMinimumSpatioTemporalDistance(
    const Point2dContainer& polygon, double t) const {
    return Distance(pos_ + VelocityDisplacement(centroid_velocity_, t), polygon);
}

const Point& PointObstacle::GetCentroid() const {
    return pos_;
}

std::complex<double> PointObstacle::GetCentroidCplx() const {
    return std::complex<double>(pos_.x, pos_.y);
}

const Point& PointObstacle::position() const {
    return pos_;
}

Point& PointObstacle::position() {
    return pos_;
}

double& PointObstacle::X() {
    return pos_.x;
}

const double& PointObstacle::X() const {
    return pos_.x;
}

double& PointObstacle::Y() {
    return pos_.y;
}

const double& PointObstacle::Y() const {
    return pos_.y;
}

void PointObstacle::ToPolygonMsg(
    autonomy::commsgs::geometry_msgs::Polygon& polygon) {
    polygon.points.resize(1);
    polygon.points.front().x = pos_.x;
    polygon.points.front().y = pos_.y;
    polygon.points.front().z = 0;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
