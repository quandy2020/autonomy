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

#include "autonomy/control/controller/teb_controller/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

constexpr double kGeometryEpsilon = 1e-8;

using Point2dContainer = PointContainer;

Point ClosestPoint(const Point& point, const Point& segment_start,
                   const Point& segment_end);

double Distance(const Point& point, const Point& segment_start,
                const Point& segment_end);

bool CheckIntersection(const Point& first_segment_start,
                       const Point& first_segment_end,
                       const Point& second_segment_start,
                       const Point& second_segment_end,
                       Point* intersection = nullptr);

double Distance(const Point& first_segment_start,
                const Point& first_segment_end,
                const Point& second_segment_start,
                const Point& second_segment_end);

double Distance(const Point& point, const PointContainer& polygon_vertices);

double Distance(const Point& segment_start, const Point& segment_end,
                const PointContainer& polygon_vertices);

double Distance(const PointContainer& first_polygon_vertices,
                const PointContainer& second_polygon_vertices);

double Distance(const Vector3& first_line_point,
                const Vector3& first_line_direction,
                const Vector3& second_line_point,
                const Vector3& second_line_direction);

double Distance(const Vector3& first_segment_start,
                const Vector3& first_segment_end,
                const Vector3& second_segment_start,
                const Vector3& second_segment_end);

inline double ApproachTime(const Point& position1, const Twist2D& velocity1,
                           const Point& position2, const Twist2D& velocity2) {
    const Twist2D relative_velocity{
        velocity1.x - velocity2.x, velocity1.y - velocity2.y,
        velocity1.theta - velocity2.theta};
    const double velocity_squared =
        relative_velocity.x * relative_velocity.x +
        relative_velocity.y * relative_velocity.y;
    if (velocity_squared < kGeometryEpsilon) {
        return 0.0;
    }
    const Point relative_position = position1 - position2;
    return -(relative_position.x * relative_velocity.x +
             relative_position.y * relative_velocity.y) /
           velocity_squared;
}

inline double Distance(const Point& position1, const Twist2D& velocity1,
                       const Point& position2, const Twist2D& velocity2,
                       double time_upper_bound = 0.0) {
    double time = ApproachTime(position1, velocity1, position2, velocity2);
    if (time_upper_bound != 0.0 && time > time_upper_bound) {
        time = time_upper_bound;
    }
    const Point first = PredictPosition(position1, velocity1, time);
    const Point second = PredictPosition(position2, velocity2, time);
    return Norm(second - first);
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
