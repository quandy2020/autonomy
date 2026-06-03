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

#include <complex>
#include <memory>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/controller/teb_controller/distance.hpp"
#include "autonomy/control/controller/teb_controller/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

class Obstacle
{
public:
    AUTONOMY_SHARED_PTR_DEFINITIONS(Obstacle);

    Obstacle();

    virtual const Point& GetCentroid() const = 0;

    virtual std::complex<double> GetCentroidCplx() const = 0;

    virtual bool CheckCollision(const Point& position, double min_dist) const = 0;

    virtual bool CheckIntersection(const Point& line_start, const Point& line_end,
                                   double min_dist = 0) const = 0;

    virtual double GetMinimumDistance(const Point& position) const = 0;

    virtual double GetMinimumDistance(const Point& line_start,
                                      const Point& line_end) const = 0;

    virtual double GetMinimumDistance(const PointContainer& polygon) const = 0;

    virtual Point GetClosestPoint(const Point& position) const = 0;

    virtual double GetMinimumSpatioTemporalDistance(const Point& position,
                                                    double t) const = 0;

    virtual double GetMinimumSpatioTemporalDistance(const Point& line_start,
                                                    const Point& line_end,
                                                    double t) const = 0;

    virtual double GetMinimumSpatioTemporalDistance(const PointContainer& polygon,
                                                    double t) const = 0;

    virtual void PredictCentroidConstantVelocity(double t, Point& position) const;

    bool IsDynamic() const;

    void SetCentroidVelocity(const Twist2D& vel);

    void SetCentroidVelocity(
        const autonomy::commsgs::geometry_msgs::TwistWithCovariance& velocity,
        const autonomy::commsgs::geometry_msgs::Quaternion& orientation);

    void SetCentroidVelocity(
        const autonomy::commsgs::geometry_msgs::TwistWithCovariance& velocity,
        const autonomy::commsgs::geometry_msgs::QuaternionStamped& orientation);

    const Twist2D& GetCentroidVelocity() const;

    virtual void ToPolygonMsg(
        autonomy::commsgs::geometry_msgs::Polygon& polygon) = 0;

    virtual void ToTwistWithCovarianceMsg(
        autonomy::commsgs::geometry_msgs::TwistWithCovariance& twistWithCovariance);

protected:
    bool dynamic_;
    Twist2D centroid_velocity_;
};

using ObstaclePtr = Obstacle::SharedPtr;
using ObstContainer = std::vector<ObstaclePtr>;

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
