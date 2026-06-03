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

#include "autonomy/control/controller/teb_controller/distance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "autonomy/common/math/line_segment2d.hpp"
#include "autonomy/common/math/polygon2d.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

namespace {

common::math::Vec2d ToVec2d(const Point& point) {
    return common::math::Vec2d(point.x, point.y);
}

Point ToPoint(const common::math::Vec2d& point) {
    return MakePoint(point.x(), point.y());
}

common::math::Polygon2d ToPolygon2d(const PointContainer& polygon_vertices) {
    std::vector<common::math::Vec2d> points;
    points.reserve(polygon_vertices.size());
    for (const Point& vertex : polygon_vertices) {
        points.emplace_back(vertex.x, vertex.y);
    }
    return common::math::Polygon2d(std::move(points));
}

}  // namespace

Point ClosestPoint(const Point& point, const Point& segment_start,
                   const Point& segment_end) {
    common::math::LineSegment2d segment(ToVec2d(segment_start),
                                        ToVec2d(segment_end));
    common::math::Vec2d nearest_point;
    segment.DistanceTo(ToVec2d(point), &nearest_point);
    return ToPoint(nearest_point);
}

double Distance(const Point& point, const Point& segment_start,
                const Point& segment_end) {
    common::math::LineSegment2d segment(ToVec2d(segment_start),
                                        ToVec2d(segment_end));
    return segment.DistanceTo(ToVec2d(point));
}

bool CheckIntersection(const Point& first_segment_start,
                       const Point& first_segment_end,
                       const Point& second_segment_start,
                       const Point& second_segment_end, Point* intersection) {
    common::math::LineSegment2d first_segment(ToVec2d(first_segment_start),
                                              ToVec2d(first_segment_end));
    common::math::LineSegment2d second_segment(ToVec2d(second_segment_start),
                                               ToVec2d(second_segment_end));
    common::math::Vec2d intersection_point;
    const bool has_intersection =
        first_segment.GetIntersect(second_segment, &intersection_point);
    if (has_intersection && intersection != nullptr) {
        *intersection = ToPoint(intersection_point);
    }
    return has_intersection;
}

double Distance(const Point& first_segment_start,
                const Point& first_segment_end,
                const Point& second_segment_start,
                const Point& second_segment_end) {
    if (CheckIntersection(first_segment_start, first_segment_end,
                          second_segment_start, second_segment_end)) {
        return 0.0;
    }

    common::math::LineSegment2d first(ToVec2d(first_segment_start),
                                      ToVec2d(first_segment_end));
    common::math::LineSegment2d second(ToVec2d(second_segment_start),
                                       ToVec2d(second_segment_end));
    const std::array<double, 4> endpoint = {
        first.DistanceTo(second.start()),
        first.DistanceTo(second.end()),
        second.DistanceTo(first.start()),
        second.DistanceTo(first.end()),
    };
    return *std::min_element(endpoint.begin(), endpoint.end());
}

double Distance(const Point& point, const PointContainer& polygon_vertices) {
    if (polygon_vertices.empty()) {
        return HUGE_VAL;
    }
    if (polygon_vertices.size() == 1) {
        return Norm(point - polygon_vertices.front());
    }
    if (polygon_vertices.size() == 2) {
        return Distance(point, polygon_vertices[0], polygon_vertices[1]);
    }

    const common::math::Polygon2d polygon = ToPolygon2d(polygon_vertices);
    return polygon.DistanceTo(ToVec2d(point));
}

double Distance(const Point& segment_start, const Point& segment_end,
                const PointContainer& polygon_vertices) {
    if (polygon_vertices.empty()) {
        return HUGE_VAL;
    }
    if (polygon_vertices.size() == 1) {
        return Distance(polygon_vertices.front(), segment_start, segment_end);
    }
    if (polygon_vertices.size() == 2) {
        return Distance(segment_start, segment_end, polygon_vertices[0],
                        polygon_vertices[1]);
    }

    common::math::LineSegment2d segment(ToVec2d(segment_start),
                                       ToVec2d(segment_end));
    const common::math::Polygon2d polygon = ToPolygon2d(polygon_vertices);
    return polygon.DistanceTo(segment);
}

double Distance(const PointContainer& first_polygon_vertices,
                const PointContainer& second_polygon_vertices) {
    if (first_polygon_vertices.empty() || second_polygon_vertices.empty()) {
        return HUGE_VAL;
    }

    if (first_polygon_vertices.size() < 3 ||
        second_polygon_vertices.size() < 3) {
        double minimum_distance = HUGE_VAL;
        for (size_t index = 0; index < first_polygon_vertices.size(); ++index) {
            minimum_distance =
                std::min(minimum_distance,
                         Distance(first_polygon_vertices[index],
                                  second_polygon_vertices));
            if (index + 1 < first_polygon_vertices.size()) {
                minimum_distance = std::min(
                    minimum_distance,
                    Distance(first_polygon_vertices[index],
                             first_polygon_vertices[index + 1],
                             second_polygon_vertices));
            }
        }
        for (size_t index = 0; index < second_polygon_vertices.size(); ++index) {
            minimum_distance =
                std::min(minimum_distance,
                         Distance(second_polygon_vertices[index],
                                  first_polygon_vertices));
            if (index + 1 < second_polygon_vertices.size()) {
                minimum_distance = std::min(
                    minimum_distance,
                    Distance(second_polygon_vertices[index],
                             second_polygon_vertices[index + 1],
                             first_polygon_vertices));
            }
        }
        return minimum_distance;
    }

    auto first = ToPolygon2d(first_polygon_vertices);
    auto second = ToPolygon2d(second_polygon_vertices);
    return first.DistanceTo(second);
}

double Distance(const Vector3& first_line_point,
                const Vector3& first_line_direction,
                const Vector3& second_line_point,
                const Vector3& second_line_direction) {
    const Vector3 line_origin_offset = second_line_point - first_line_point;
    const float first_direction_squared_norm = SquaredNorm(first_line_direction);
    const float direction_dot_product =
        Dot(first_line_direction, second_line_direction);
    const float second_direction_squared_norm =
        SquaredNorm(second_line_direction);
    const float first_direction_projection =
        Dot(first_line_direction, line_origin_offset);
    const float second_direction_projection =
        Dot(second_line_direction, line_origin_offset);
    const float denominator = first_direction_squared_norm *
                              second_direction_squared_norm -
                              direction_dot_product * direction_dot_product;

    float first_line_parameter = 0.0f;
    float second_line_parameter = 0.0f;
    if (denominator < kGeometryEpsilon) {
        second_line_parameter =
            direction_dot_product > second_direction_squared_norm
                ? first_direction_projection / direction_dot_product
                : second_direction_projection / second_direction_squared_norm;
    } else {
        first_line_parameter =
            (direction_dot_product * second_direction_projection -
             second_direction_squared_norm * first_direction_projection) /
            denominator;
        second_line_parameter =
            (first_direction_squared_norm * second_direction_projection -
             direction_dot_product * first_direction_projection) /
            denominator;
    }

    const Vector3 closest_vector =
        line_origin_offset + first_line_parameter * first_line_direction -
        second_line_parameter * second_line_direction;
    return Norm(closest_vector);
}

double Distance(const Vector3& first_segment_start,
                const Vector3& first_segment_end,
                const Vector3& second_segment_start,
                const Vector3& second_segment_end) {
    const Vector3 first_segment_direction =
        first_segment_end - first_segment_start;
    const Vector3 second_segment_direction =
        second_segment_end - second_segment_start;
    const Vector3 segment_origin_offset =
        second_segment_start - first_segment_start;

    const float first_direction_squared_norm =
        SquaredNorm(first_segment_direction);
    const float direction_dot_product =
        Dot(first_segment_direction, second_segment_direction);
    const float second_direction_squared_norm =
        SquaredNorm(second_segment_direction);
    const float first_direction_projection =
        Dot(first_segment_direction, segment_origin_offset);
    const float second_direction_projection =
        Dot(second_segment_direction, segment_origin_offset);
    const float denominator = first_direction_squared_norm *
                                second_direction_squared_norm -
                                direction_dot_product * direction_dot_product;

    float first_segment_numerator = 0.0f;
    float first_segment_denominator = denominator;
    float second_segment_numerator = 0.0f;
    float second_segment_denominator = denominator;

    if (denominator < kGeometryEpsilon) {
        first_segment_numerator = 0.0f;
        first_segment_denominator = 1.0f;
        second_segment_numerator = second_direction_projection;
        second_segment_denominator = second_direction_squared_norm;
    } else {
        first_segment_numerator =
            direction_dot_product * second_direction_projection -
            second_direction_squared_norm * first_direction_projection;
        second_segment_numerator =
            first_direction_squared_norm * second_direction_projection -
            direction_dot_product * first_direction_projection;
        if (first_segment_numerator < 0.0f) {
            first_segment_numerator = 0.0f;
            second_segment_numerator = second_direction_projection;
            second_segment_denominator = second_direction_squared_norm;
        } else if (first_segment_numerator > first_segment_denominator) {
            first_segment_numerator = first_segment_denominator;
            second_segment_numerator =
                second_direction_projection + direction_dot_product;
            second_segment_denominator = second_direction_squared_norm;
        }
    }

    if (second_segment_numerator < 0.0f) {
        second_segment_numerator = 0.0f;
        if (-first_direction_projection < 0.0f) {
            first_segment_numerator = 0.0f;
        } else if (-first_direction_projection >
                   first_direction_squared_norm) {
            first_segment_numerator = first_segment_denominator;
        } else {
            first_segment_numerator = -first_direction_projection;
            first_segment_denominator = first_direction_squared_norm;
        }
    } else if (second_segment_numerator > second_segment_denominator) {
        second_segment_numerator = second_segment_denominator;
        if ((-first_direction_projection + direction_dot_product) < 0.0f) {
            first_segment_numerator = 0.0f;
        } else if ((-first_direction_projection + direction_dot_product) >
                   first_direction_squared_norm) {
            first_segment_numerator = first_segment_denominator;
        } else {
            first_segment_numerator =
                -first_direction_projection + direction_dot_product;
            first_segment_denominator = first_direction_squared_norm;
        }
    }

    const float first_segment_parameter =
        std::abs(first_segment_numerator) < kGeometryEpsilon
            ? 0.0f
            : first_segment_numerator / first_segment_denominator;
    const float second_segment_parameter =
        std::abs(second_segment_numerator) < kGeometryEpsilon
            ? 0.0f
            : second_segment_numerator / second_segment_denominator;
    const Vector3 closest_vector =
        segment_origin_offset +
        first_segment_parameter * first_segment_direction -
        second_segment_parameter * second_segment_direction;
    return Norm(closest_vector);
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
