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

#include <cassert>
#include <cmath>
#include <limits>

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/teb_controller/geometry/polygon_obstacle.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

namespace {

void SetCentroidNaN(Point& centroid) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    centroid.x = nan;
    centroid.y = nan;
    centroid.z = nan;
}

}  // namespace

PolygonObstacle::PolygonObstacle() : Obstacle(), finalized_(false) {
    SetCentroidNaN(centroid_);
}

PolygonObstacle::PolygonObstacle(const Point2dContainer& vertices)
    : Obstacle(), vertices_(vertices) {
    finalizePolygon();
}

void PolygonObstacle::fixPolygonClosure() {
    if (vertices_.size() < 2) {
        return;
    }

    if (PointsApproxEqual(vertices_.front(), vertices_.back())) {
        vertices_.pop_back();
    }
}

void PolygonObstacle::CalcCentroid() {
    if (vertices_.empty()) {
        SetCentroidNaN(centroid_);
        AERROR << "PolygonObstacle::CalcCentroid(): number of vertices is "
                  "empty. the resulting centroid is a vector of NANs.";
        return;
    }

    if (NoVertices() == 1) {
        centroid_ = vertices_.front();
        return;
    }

    if (NoVertices() == 2) {
        centroid_ = (vertices_.front() + vertices_.back()) * 0.5;
        return;
    }

    SetZero(centroid_);

    double A = 0;
    for (int i = 0; i < NoVertices() - 1; ++i) {
        A += vertices_.at(i).x * vertices_.at(i + 1).y -
             vertices_.at(i + 1).x * vertices_.at(i).y;
    }
    A += vertices_.at(NoVertices() - 1).x * vertices_.at(0).y -
         vertices_.at(0).x * vertices_.at(NoVertices() - 1).y;
    A *= 0.5;

    if (A != 0) {
        for (int i = 0; i < NoVertices() - 1; ++i) {
            const double aux =
                (vertices_.at(i).x * vertices_.at(i + 1).y -
                 vertices_.at(i + 1).x * vertices_.at(i).y);
            centroid_ += (vertices_.at(i) + vertices_.at(i + 1)) * aux;
        }
        const double aux = (vertices_.at(NoVertices() - 1).x *
                                vertices_.at(0).y -
                            vertices_.at(0).x *
                                vertices_.at(NoVertices() - 1).y);
        centroid_ += (vertices_.at(NoVertices() - 1) + vertices_.at(0)) * aux;
        centroid_ = centroid_ * (1.0 / (6 * A));
    } else {
        int i_cand = 0;
        int j_cand = 0;
        double max_dist = 0;
        for (int i = 0; i < NoVertices(); ++i) {
            for (int j = i + 1; j < NoVertices(); ++j) {
                const double dist = Norm(vertices_[j] - vertices_[i]);
                if (dist > max_dist) {
                    max_dist = dist;
                    i_cand = i;
                    j_cand = j;
                }
            }
        }
        centroid_ = (vertices_[i_cand] + vertices_[j_cand]) * 0.5;
    }
}

Point PolygonObstacle::GetClosestPoint(const Point& position) const {
    if (NoVertices() == 1) {
        return vertices_.front();
    }

    if (NoVertices() > 1) {
        Point new_pt = ClosestPoint(position, vertices_.at(0), vertices_.at(1));

        if (NoVertices() > 2) {
            double dist = Norm(new_pt - position);
            Point closest_pt = new_pt;

            for (int i = 1; i < NoVertices() - 1; ++i) {
                new_pt = ClosestPoint(position, vertices_.at(i),
                                      vertices_.at(i + 1));
                const double new_dist = Norm(new_pt - position);
                if (new_dist < dist) {
                    dist = new_dist;
                    closest_pt = new_pt;
                }
            }
            new_pt = ClosestPoint(position, vertices_.back(),
                                  vertices_.front());
            const double new_dist = Norm(new_pt - position);
            return new_dist < dist ? new_pt : closest_pt;
        }
        return new_pt;
    }

    AERROR << "PolygonObstacle::GetClosestPoint(): cannot find closest point; "
              "polygon may be ill-defined.";
    return MakePoint();
}

bool PolygonObstacle::CheckCollision(const Point& point, double min_dist) const {
    if (NoVertices() == 2) {
        return GetMinimumDistance(point) <= min_dist;
    }

    int i = 0;
    int j = 0;
    bool inside = false;
    for (i = 0, j = NoVertices() - 1; i < NoVertices(); j = i++) {
        if (((vertices_.at(i).y > point.y) != (vertices_.at(j).y > point.y)) &&
            (point.x <
             (vertices_.at(j).x - vertices_.at(i).x) *
                     (point.y - vertices_.at(i).y) /
                     (vertices_.at(j).y - vertices_.at(i).y) +
                 vertices_.at(i).x)) {
            inside = !inside;
        }
    }
    if (inside) {
        return true;
    }

    return min_dist == 0 ? false : GetMinimumDistance(point) < min_dist;
}

bool PolygonObstacle::CheckIntersection(const Point& line_start,
                                        const Point& line_end,
                                        double /*min_dist*/) const {
    for (int i = 0; i < NoVertices() - 1; ++i) {
        if (teb_controller::CheckIntersection(line_start, line_end,
                                              vertices_.at(i),
                                              vertices_.at(i + 1))) {
            return true;
        }
    }
    if (NoVertices() == 2) {
        return false;
    }

    return teb_controller::CheckIntersection(line_start, line_end, vertices_.back(),
                                           vertices_.front());
}

double PolygonObstacle::GetMinimumDistance(const Point& position) const {
    return Distance(position, vertices_);
}

double PolygonObstacle::GetMinimumDistance(const Point& line_start,
                                           const Point& line_end) const {
    return Distance(line_start, line_end, vertices_);
}

double PolygonObstacle::GetMinimumDistance(
    const Point2dContainer& polygon) const {
    return Distance(polygon, vertices_);
}

double PolygonObstacle::GetMinimumSpatioTemporalDistance(const Point& position,
                                                         double t) const {
    Point2dContainer pred_vertices;
    predictVertices(t, pred_vertices);
    return Distance(position, pred_vertices);
}

double PolygonObstacle::GetMinimumSpatioTemporalDistance(const Point& line_start,
                                                         const Point& line_end,
                                                         double t) const {
    Point2dContainer pred_vertices;
    predictVertices(t, pred_vertices);
    return Distance(line_start, line_end, pred_vertices);
}

double PolygonObstacle::GetMinimumSpatioTemporalDistance(
    const Point2dContainer& polygon, double t) const {
    Point2dContainer pred_vertices;
    predictVertices(t, pred_vertices);
    return Distance(polygon, pred_vertices);
}

void PolygonObstacle::predictVertices(double t,
                                    Point2dContainer& pred_vertices) const {
    pred_vertices.resize(vertices_.size());
    const Point offset = VelocityDisplacement(centroid_velocity_, t);
    for (std::size_t i = 0; i < vertices_.size(); ++i) {
        pred_vertices[i] = vertices_[i] + offset;
    }
}

const Point& PolygonObstacle::GetCentroid() const {
    assert(finalized_ &&
           "Finalize the polygon after all vertices are added.");
    return centroid_;
}

std::complex<double> PolygonObstacle::GetCentroidCplx() const {
    assert(finalized_ &&
           "Finalize the polygon after all vertices are added.");
    return std::complex<double>(centroid_.x, centroid_.y);
}

void PolygonObstacle::ToPolygonMsg(
    autonomy::commsgs::geometry_msgs::Polygon& polygon) {
    polygon.points.resize(vertices_.size());
    for (std::size_t i = 0; i < vertices_.size(); ++i) {
        polygon.points[i].x = vertices_[i].x;
        polygon.points[i].y = vertices_[i].y;
        polygon.points[i].z = 0;
    }
}

const Point2dContainer& PolygonObstacle::vertices() const {
    return vertices_;
}

Point2dContainer& PolygonObstacle::vertices() {
    return vertices_;
}

void PolygonObstacle::pushBackVertex(const Point& vertex) {
    vertices_.push_back(vertex);
    finalized_ = false;
}

void PolygonObstacle::pushBackVertex(double x, double y) {
    vertices_.push_back(MakePoint(x, y));
    finalized_ = false;
}

void PolygonObstacle::finalizePolygon() {
    fixPolygonClosure();
    CalcCentroid();
    finalized_ = true;
}

void PolygonObstacle::clearVertices() {
    vertices_.clear();
    finalized_ = false;
}

int PolygonObstacle::NoVertices() const {
    return static_cast<int>(vertices_.size());
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
