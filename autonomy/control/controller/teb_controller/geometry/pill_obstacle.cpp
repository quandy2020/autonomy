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

#include "autonomy/control/controller/teb_controller/geometry/pill_obstacle.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

PillObstacle::PillObstacle() : Obstacle() {
    SetZero(start_);
    SetZero(end_);
    SetZero(centroid_);
}

PillObstacle::PillObstacle(const Point& line_start, const Point& line_end,
                           double radius)
    : Obstacle(), start_(line_start), end_(line_end), radius_(radius) {
    CalcCentroid();
}

PillObstacle::PillObstacle(double x1, double y1, double x2, double y2,
                           double radius)
    : Obstacle(), start_(MakePoint(x1, y1)), end_(MakePoint(x2, y2)),
      radius_(radius) {
    CalcCentroid();
}

bool PillObstacle::CheckCollision(const Point& point, double min_dist) const {
    return GetMinimumDistance(point) <= min_dist;
}

bool PillObstacle::CheckIntersection(const Point& line_start, const Point& line_end,
                                   double /*min_dist*/) const {
    return teb_controller::CheckIntersection(line_start, line_end, start_, end_);
}

double PillObstacle::GetMinimumDistance(const Point& position) const {
    return Distance(position, start_, end_) - radius_;
}

double PillObstacle::GetMinimumDistance(const Point& line_start,
                                        const Point& line_end) const {
    return Distance(start_, end_, line_start, line_end) - radius_;
}

double PillObstacle::GetMinimumDistance(const Point2dContainer& polygon) const {
    return Distance(start_, end_, polygon) - radius_;
}

Point PillObstacle::GetClosestPoint(const Point& position) const {
    const Point closest_on_axis = ClosestPoint(position, start_, end_);
    return closest_on_axis +
           radius_ * Normalized(position - closest_on_axis);
}

double PillObstacle::GetMinimumSpatioTemporalDistance(const Point& position,
                                                      double t) const {
    const Point offset = VelocityDisplacement(centroid_velocity_, t);
    return Distance(position, start_ + offset, end_ + offset) - radius_;
}

double PillObstacle::GetMinimumSpatioTemporalDistance(const Point& line_start,
                                                      const Point& line_end,
                                                      double t) const {
    const Point offset = VelocityDisplacement(centroid_velocity_, t);
    return Distance(start_ + offset, end_ + offset, line_start, line_end) -
           radius_;
}

double PillObstacle::GetMinimumSpatioTemporalDistance(
    const Point2dContainer& polygon, double t) const {
    const Point offset = VelocityDisplacement(centroid_velocity_, t);
    return Distance(start_ + offset, end_ + offset, polygon) - radius_;
}

const Point& PillObstacle::GetCentroid() const {
    return centroid_;
}

std::complex<double> PillObstacle::GetCentroidCplx() const {
    return std::complex<double>(centroid_.x, centroid_.y);
}

const Point& PillObstacle::start() const {
    return start_;
}

void PillObstacle::SetStart(const Point& start) {
    start_ = start;
    CalcCentroid();
}

const Point& PillObstacle::end() const {
    return end_;
}

void PillObstacle::SetEnd(const Point& end) {
    end_ = end;
    CalcCentroid();
}

void PillObstacle::ToPolygonMsg(
    autonomy::commsgs::geometry_msgs::Polygon& polygon) {
    polygon.points.resize(2);
    polygon.points.front().x = start_.x;
    polygon.points.front().y = start_.y;
    polygon.points.back().x = end_.x;
    polygon.points.back().y = end_.y;
    polygon.points.back().z = polygon.points.front().z = 0;
}

void PillObstacle::CalcCentroid() {
    centroid_ = (start_ + end_) * 0.5;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
