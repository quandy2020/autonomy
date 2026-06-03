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

#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Polygon obstacle (point, line, or closed polygon)
 */
class PolygonObstacle : public Obstacle
{
public:
    PolygonObstacle();
    explicit PolygonObstacle(const Point2dContainer& vertices);

    bool CheckCollision(const Point& point, double min_dist) const override;

    bool CheckIntersection(const Point& line_start, const Point& line_end,
                           double min_dist = 0) const override;

    double GetMinimumDistance(const Point& position) const override;

    double GetMinimumDistance(const Point& line_start,
                              const Point& line_end) const override;

    double GetMinimumDistance(const Point2dContainer& polygon) const override;

    Point GetClosestPoint(const Point& position) const override;

    double GetMinimumSpatioTemporalDistance(const Point& position,
                                            double t) const override;

    double GetMinimumSpatioTemporalDistance(const Point& line_start,
                                            const Point& line_end,
                                            double t) const override;

    double GetMinimumSpatioTemporalDistance(const Point2dContainer& polygon,
                                            double t) const override;

    void predictVertices(double t, Point2dContainer& pred_vertices) const;

    const Point& GetCentroid() const override;

    std::complex<double> GetCentroidCplx() const override;

    void ToPolygonMsg(
        autonomy::commsgs::geometry_msgs::Polygon& polygon) override;

    const Point2dContainer& vertices() const;
    Point2dContainer& vertices();
    void pushBackVertex(const Point& vertex);
    void pushBackVertex(double x, double y);
    void finalizePolygon();
    void clearVertices();
    int NoVertices() const;

protected:
    void fixPolygonClosure();
    void CalcCentroid();

    Point2dContainer vertices_;
    Point centroid_{};
    bool finalized_;
};

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
