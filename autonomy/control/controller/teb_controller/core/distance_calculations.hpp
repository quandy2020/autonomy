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

#include <algorithm>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "autonomy/common/math/line_segment2d.hpp"
#include "autonomy/common/math/polygon2d.hpp"
#include "autonomy/common/math/vec2d.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

using Point2dContainer =
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

namespace {

using ::autonomy::common::math::LineSegment2d;
using ::autonomy::common::math::Polygon2d;
using ::autonomy::common::math::Vec2d;

inline Vec2d ToVec2d(const Eigen::Ref<const Eigen::Vector2d>& p) {
    return Vec2d(p.x(), p.y());
}

inline Eigen::Vector2d ToEigen(const Vec2d& p) {
    return Eigen::Vector2d(p.x(), p.y());
}

inline std::vector<Vec2d> ToVec2dList(const Point2dContainer& vertices) {
    std::vector<Vec2d> out;
    out.reserve(vertices.size());
    for (const auto& v : vertices) {
        out.emplace_back(v.x(), v.y());
    }
    return out;
}

inline LineSegment2d ToSegment(const Eigen::Ref<const Eigen::Vector2d>& start,
                               const Eigen::Ref<const Eigen::Vector2d>& end) {
    return LineSegment2d(ToVec2d(start), ToVec2d(end));
}

}  // namespace

inline Eigen::Vector2d closest_point_on_line_segment_2d(
    const Eigen::Ref<const Eigen::Vector2d>& point,
    const Eigen::Ref<const Eigen::Vector2d>& line_start,
    const Eigen::Ref<const Eigen::Vector2d>& line_end) {
    Vec2d nearest;
    ToSegment(line_start, line_end).DistanceTo(ToVec2d(point), &nearest);
    return ToEigen(nearest);
}

inline double distance_point_to_segment_2d(
    const Eigen::Ref<const Eigen::Vector2d>& point,
    const Eigen::Ref<const Eigen::Vector2d>& line_start,
    const Eigen::Ref<const Eigen::Vector2d>& line_end) {
    return ToSegment(line_start, line_end).DistanceTo(ToVec2d(point));
}

inline bool check_line_segments_intersection_2d(
    const Eigen::Ref<const Eigen::Vector2d>& line1_start,
    const Eigen::Ref<const Eigen::Vector2d>& line1_end,
    const Eigen::Ref<const Eigen::Vector2d>& line2_start,
    const Eigen::Ref<const Eigen::Vector2d>& line2_end,
    Eigen::Vector2d* intersection = nullptr) {
    const auto a = ToSegment(line1_start, line1_end);
    const auto b = ToSegment(line2_start, line2_end);
    if (intersection == nullptr) {
        return a.HasIntersect(b);
    }
    Vec2d pt;
    if (!a.GetIntersect(b, &pt)) {
        return false;
    }
    *intersection = ToEigen(pt);
    return true;
}

inline double distance_segment_to_segment_2d(
    const Eigen::Ref<const Eigen::Vector2d>& line1_start,
    const Eigen::Ref<const Eigen::Vector2d>& line1_end,
    const Eigen::Ref<const Eigen::Vector2d>& line2_start,
    const Eigen::Ref<const Eigen::Vector2d>& line2_end) {
    const auto a = ToSegment(line1_start, line1_end);
    const auto b = ToSegment(line2_start, line2_end);
    if (a.HasIntersect(b)) {
        return 0.0;
    }
    return std::min({a.DistanceTo(b.start()), a.DistanceTo(b.end()),
                     b.DistanceTo(a.start()), b.DistanceTo(a.end())});
}

inline double distance_point_to_polygon_2d(const Eigen::Vector2d& point,
                                           const Point2dContainer& vertices) {
    if (vertices.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (vertices.size() == 1) {
        return (point - vertices.front()).norm();
    }
    if (vertices.size() == 2) {
        return distance_point_to_segment_2d(point, vertices[0], vertices[1]);
    }
    // Edge distance (TEB semantics); not zero when the point is inside.
    return Polygon2d(ToVec2dList(vertices)).DistanceToBoundary(ToVec2d(point));
}

inline double distance_segment_to_polygon_2d(const Eigen::Vector2d& line_start,
                                             const Eigen::Vector2d& line_end,
                                             const Point2dContainer& vertices) {
    if (vertices.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (vertices.size() == 1) {
        return distance_point_to_segment_2d(vertices.front(), line_start,
                                            line_end);
    }
    if (vertices.size() == 2) {
        return distance_segment_to_segment_2d(line_start, line_end, vertices[0],
                                              vertices[1]);
    }
    return Polygon2d(ToVec2dList(vertices))
        .DistanceTo(ToSegment(line_start, line_end));
}

inline double distance_polygon_to_polygon_2d(
    const Point2dContainer& vertices1, const Point2dContainer& vertices2) {
    if (vertices1.empty() || vertices2.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (vertices1.size() == 1) {
        return distance_point_to_polygon_2d(vertices1.front(), vertices2);
    }
    if (vertices2.size() == 1) {
        return distance_point_to_polygon_2d(vertices2.front(), vertices1);
    }
    if (vertices1.size() == 2) {
        return distance_segment_to_polygon_2d(vertices1[0], vertices1[1],
                                              vertices2);
    }
    if (vertices2.size() == 2) {
        return distance_segment_to_polygon_2d(vertices2[0], vertices2[1],
                                              vertices1);
    }
    return Polygon2d(ToVec2dList(vertices1))
        .DistanceTo(Polygon2d(ToVec2dList(vertices2)));
}

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
