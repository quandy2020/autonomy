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

#include "autonomy/control/controller/teb_controller/geometry/line_obstacle.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

LineObstacle::LineObstacle() : Obstacle() {
    SetZero(start_);
    SetZero(end_);
    SetZero(centroid_);
}

LineObstacle::LineObstacle(const Point& line_start, const Point& line_end)
    : Obstacle(), start_(line_start), end_(line_end) {
    CalcCentroid();
}

LineObstacle::LineObstacle(double x1, double y1, double x2, double y2)
    : Obstacle(), start_(MakePoint(x1, y1)), end_(MakePoint(x2, y2)) {
    CalcCentroid();
}

bool LineObstacle::CheckCollision(const Point& point, double min_dist) const {
    return GetMinimumDistance(point) <= min_dist;
}

bool LineObstacle::CheckIntersection(const Point& line_start, const Point& line_end,
                                     double /*min_dist*/) const {
    return teb_controller::CheckIntersection(line_start, line_end, start_, end_);
}

double LineObstacle::GetMinimumDistance(const Point& position) const {
    return Distance(position, start_, end_);
}

double LineObstacle::GetMinimumDistance(const Point& line_start,
                                        const Point& line_end) const {
    return Distance(start_, end_, line_start, line_end);
}

double LineObstacle::GetMinimumDistance(const Point2dContainer& polygon) const {
    return Distance(start_, end_, polygon);
}

Point LineObstacle::GetClosestPoint(const Point& position) const {
    return ClosestPoint(position, start_, end_);
}

double LineObstacle::GetMinimumSpatioTemporalDistance(const Point& position,
                                                      double t) const {
    const Point offset = VelocityDisplacement(centroid_velocity_, t);
    return Distance(position, start_ + offset, end_ + offset);
}

double LineObstacle::GetMinimumSpatioTemporalDistance(const Point& line_start,
                                                      const Point& line_end,
                                                      double t) const {
    const Point offset = VelocityDisplacement(centroid_velocity_, t);
    return Distance(start_ + offset, end_ + offset, line_start, line_end);
}

double LineObstacle::GetMinimumSpatioTemporalDistance(
    const Point2dContainer& polygon, double t) const {
    const Point offset = VelocityDisplacement(centroid_velocity_, t);
    return Distance(start_ + offset, end_ + offset, polygon);
}

const Point& LineObstacle::GetCentroid() const {
    return centroid_;
}

std::complex<double> LineObstacle::GetCentroidCplx() const {
    return std::complex<double>(centroid_.x, centroid_.y);
}

const Point& LineObstacle::start() const {
    return start_;
}

void LineObstacle::SetStart(const Point& start) {
    start_ = start;
    CalcCentroid();
}

const Point& LineObstacle::end() const {
    return end_;
}

void LineObstacle::SetEnd(const Point& end) {
    end_ = end;
    CalcCentroid();
}

void LineObstacle::ToPolygonMsg(
    autonomy::commsgs::geometry_msgs::Polygon& polygon) {
    polygon.points.resize(2);
    polygon.points.front().x = start_.x;
    polygon.points.front().y = start_.y;
    polygon.points.back().x = end_.x;
    polygon.points.back().y = end_.y;
    polygon.points.back().z = polygon.points.front().z = 0;
}

void LineObstacle::CalcCentroid() {
    centroid_ = (start_ + end_) * 0.5;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
