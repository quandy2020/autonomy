/* Copyright 2026 The Openbot Authors. */
/** @file polygon_boundary.hpp
 * @brief Validated task-polygon point and line-segment domain predicates.
 */
#pragma once

#include <automsgs/msgs/geometry_msgs/polygon.pb.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "autonomy/perception/data/map_reader.hpp"

namespace autonomy::task::exploration {
namespace polygon_boundary_internal {
using Polygon = ::automsgs::msgs::geometry_msgs::Polygon;

inline int VertexCount(const Polygon& polygon) {
    int count = polygon.points_size();
    if (count > 1 && polygon.points(0).x() == polygon.points(count - 1).x() &&
        polygon.points(0).y() == polygon.points(count - 1).y()) {
        --count;
    }
    return count;
}

inline double Cross(double ax, double ay, double bx, double by, double cx,
                    double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

inline bool OnSegment(double ax, double ay, double bx, double by, double px,
                      double py) {
    const double scale =
        std::max({1.0, std::abs(ax), std::abs(ay), std::abs(bx), std::abs(by),
                  std::abs(px), std::abs(py)});
    const double tolerance =
        32 * std::numeric_limits<double>::epsilon() * scale * scale;
    return std::abs(Cross(ax, ay, bx, by, px, py)) <= tolerance &&
           px >= std::min(ax, bx) - tolerance &&
           px <= std::max(ax, bx) + tolerance &&
           py >= std::min(ay, by) - tolerance &&
           py <= std::max(ay, by) + tolerance;
}

inline int Orientation(double ax, double ay, double bx, double by, double cx,
                       double cy) {
    const double cross = Cross(ax, ay, bx, by, cx, cy);
    const double scale =
        std::max({1.0, std::abs(ax), std::abs(ay), std::abs(bx), std::abs(by),
                  std::abs(cx), std::abs(cy)});
    const double tolerance =
        32 * std::numeric_limits<double>::epsilon() * scale * scale;
    return cross > tolerance ? 1 : (cross < -tolerance ? -1 : 0);
}

inline bool Intersects(const Polygon& polygon, int first, int second,
                       int count) {
    const auto& a = polygon.points(first);
    const auto& b = polygon.points((first + 1) % count);
    const auto& c = polygon.points(second);
    const auto& d = polygon.points((second + 1) % count);
    const int abc = Orientation(a.x(), a.y(), b.x(), b.y(), c.x(), c.y());
    const int abd = Orientation(a.x(), a.y(), b.x(), b.y(), d.x(), d.y());
    const int cda = Orientation(c.x(), c.y(), d.x(), d.y(), a.x(), a.y());
    const int cdb = Orientation(c.x(), c.y(), d.x(), d.y(), b.x(), b.y());
    if (abc != abd && cda != cdb)
        return true;
    return (abc == 0 && OnSegment(a.x(), a.y(), b.x(), b.y(), c.x(), c.y())) ||
           (abd == 0 && OnSegment(a.x(), a.y(), b.x(), b.y(), d.x(), d.y())) ||
           (cda == 0 && OnSegment(c.x(), c.y(), d.x(), d.y(), a.x(), a.y())) ||
           (cdb == 0 && OnSegment(c.x(), c.y(), d.x(), d.y(), b.x(), b.y()));
}

inline void AppendSegmentIntersections(const Polygon& polygon, double ax,
                                       double ay, double bx, double by,
                                       std::vector<double>* parameters) {
    const long double rx = static_cast<long double>(bx) - ax;
    const long double ry = static_cast<long double>(by) - ay;
    const long double route_squared = rx * rx + ry * ry;
    if (route_squared == 0)
        return;
    const int count = VertexCount(polygon);
    const long double parameter_tolerance =
        128 * std::numeric_limits<double>::epsilon();
    for (int index = 0; index < count; ++index) {
        const auto& first = polygon.points(index);
        const auto& second = polygon.points((index + 1) % count);
        const long double qx = static_cast<long double>(first.x()) - ax;
        const long double qy = static_cast<long double>(first.y()) - ay;
        const long double sx = static_cast<long double>(second.x()) - first.x();
        const long double sy = static_cast<long double>(second.y()) - first.y();
        const long double denominator = rx * sy - ry * sx;
        if (denominator != 0) {
            const long double route_parameter =
                (qx * sy - qy * sx) / denominator;
            const long double edge_parameter =
                (qx * ry - qy * rx) / denominator;
            if (route_parameter >= -parameter_tolerance &&
                route_parameter <= 1 + parameter_tolerance &&
                edge_parameter >= -parameter_tolerance &&
                edge_parameter <= 1 + parameter_tolerance) {
                parameters->push_back(static_cast<double>(
                    std::clamp(route_parameter, 0.0L, 1.0L)));
            }
            continue;
        }
        if (qx * ry - qy * rx != 0)
            continue;
        long double first_parameter = (qx * rx + qy * ry) / route_squared;
        const long double second_qx = static_cast<long double>(second.x()) - ax;
        const long double second_qy = static_cast<long double>(second.y()) - ay;
        long double second_parameter =
            (second_qx * rx + second_qy * ry) / route_squared;
        if (first_parameter > second_parameter)
            std::swap(first_parameter, second_parameter);
        const long double overlap_start = std::max(0.0L, first_parameter);
        const long double overlap_end = std::min(1.0L, second_parameter);
        if (overlap_start <= overlap_end + parameter_tolerance) {
            parameters->push_back(
                static_cast<double>(std::clamp(overlap_start, 0.0L, 1.0L)));
            parameters->push_back(
                static_cast<double>(std::clamp(overlap_end, 0.0L, 1.0L)));
        }
    }
}
}  // namespace polygon_boundary_internal

/** @brief Validate a finite simple polygon and return its broad-phase bounds.
 *
 * An optional repeated closing vertex is accepted. Degenerate edges, zero-area
 * rings and self intersections are rejected.
 */
inline std::optional<perception::Bounds2d> ValidatePolygonBoundary(
    const ::automsgs::msgs::geometry_msgs::Polygon& polygon) {
    using polygon_boundary_internal::Intersects;
    using polygon_boundary_internal::VertexCount;
    const int count = VertexCount(polygon);
    if (count < 3)
        return std::nullopt;
    for (const auto& point : polygon.points()) {
        if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
            !std::isfinite(point.z())) {
            return std::nullopt;
        }
    }
    perception::Bounds2d bounds{std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity()};
    double twice_area = 0;
    for (int index = 0; index < count; ++index) {
        const auto& point = polygon.points(index);
        const auto& next = polygon.points((index + 1) % count);
        if (point.x() == next.x() && point.y() == next.y()) {
            return std::nullopt;
        }
        bounds.min_x = std::min<double>(bounds.min_x, point.x());
        bounds.min_y = std::min<double>(bounds.min_y, point.y());
        bounds.max_x = std::max<double>(bounds.max_x, point.x());
        bounds.max_y = std::max<double>(bounds.max_y, point.y());
        twice_area += point.x() * next.y() - next.x() * point.y();
    }
    const double scale =
        std::max({1.0, std::abs(bounds.min_x), std::abs(bounds.min_y),
                  std::abs(bounds.max_x), std::abs(bounds.max_y)});
    if (std::abs(twice_area) <=
        64 * std::numeric_limits<double>::epsilon() * scale * scale) {
        return std::nullopt;
    }
    for (int first = 0; first < count; ++first) {
        for (int second = first + 1; second < count; ++second) {
            if (second == first + 1 || (first == 0 && second == count - 1)) {
                continue;
            }
            if (Intersects(polygon, first, second, count))
                return std::nullopt;
        }
    }
    return bounds;
}

/** @brief Return whether broad-phase bounds enclose the validated polygon. */
inline bool PolygonBoundaryWithinBounds(
    const ::automsgs::msgs::geometry_msgs::Polygon& polygon,
    const perception::Bounds2d& bounds) {
    const auto polygon_bounds = ValidatePolygonBoundary(polygon);
    return polygon_bounds && std::isfinite(bounds.min_x) &&
           std::isfinite(bounds.min_y) && std::isfinite(bounds.max_x) &&
           std::isfinite(bounds.max_y) &&
           bounds.min_x <= polygon_bounds->min_x &&
           bounds.min_y <= polygon_bounds->min_y &&
           bounds.max_x >= polygon_bounds->max_x &&
           bounds.max_y >= polygon_bounds->max_y;
}

/** @brief Return whether a finite point lies in the polygon's closed set. */
inline bool PolygonBoundaryContains(
    const ::automsgs::msgs::geometry_msgs::Polygon& polygon, double x,
    double y) {
    using polygon_boundary_internal::OnSegment;
    using polygon_boundary_internal::VertexCount;
    if (!std::isfinite(x) || !std::isfinite(y))
        return false;
    const int count = VertexCount(polygon);
    if (count < 3)
        return false;
    bool inside = false;
    for (int index = 0, previous = count - 1; index < count;
         previous = index++) {
        const auto& a = polygon.points(previous);
        const auto& b = polygon.points(index);
        if (OnSegment(a.x(), a.y(), b.x(), b.y(), x, y))
            return true;
        if ((a.y() > y) != (b.y() > y)) {
            const double intersection =
                a.x() + (y - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
            if (intersection > x)
                inside = !inside;
        }
    }
    return inside;
}

namespace polygon_boundary_internal {
inline bool ValidatedPolygonContainsSegment(const Polygon& polygon, double ax,
                                            double ay, double bx, double by) {
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(bx) ||
        !std::isfinite(by) || !PolygonBoundaryContains(polygon, ax, ay) ||
        !PolygonBoundaryContains(polygon, bx, by)) {
        return false;
    }
    std::vector<double> parameters{0, 1};
    AppendSegmentIntersections(polygon, ax, ay, bx, by, &parameters);
    std::sort(parameters.begin(), parameters.end());
    parameters.erase(std::unique(parameters.begin(), parameters.end()),
                     parameters.end());
    for (std::size_t index = 1; index < parameters.size(); ++index) {
        const double midpoint =
            (parameters[index - 1] + parameters[index]) * .5;
        const double x = ax + (bx - ax) * midpoint;
        const double y = ay + (by - ay) * midpoint;
        if (!PolygonBoundaryContains(polygon, x, y))
            return false;
    }
    return true;
}

inline bool ValidatedPolygonDisjointFromSegment(const Polygon& polygon,
                                                double ax, double ay, double bx,
                                                double by) {
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(bx) ||
        !std::isfinite(by) || PolygonBoundaryContains(polygon, ax, ay) ||
        PolygonBoundaryContains(polygon, bx, by)) {
        return false;
    }
    std::vector<double> parameters;
    AppendSegmentIntersections(polygon, ax, ay, bx, by, &parameters);
    return parameters.empty();
}
}  // namespace polygon_boundary_internal

/** @brief Return whether a finite segment stays in a polygon's closed set.
 * @details Traversal along the polygon boundary is permitted. The predicate
 * partitions the segment at every boundary contact and validates each open
 * interval, so endpoint-only checks cannot admit a chord across a concavity.
 * A malformed polygon or nonfinite segment fails closed.
 */
inline bool PolygonBoundaryContainsSegment(
    const ::automsgs::msgs::geometry_msgs::Polygon& polygon, double ax,
    double ay, double bx, double by) {
    return ValidatePolygonBoundary(polygon) &&
           polygon_boundary_internal::ValidatedPolygonContainsSegment(
               polygon, ax, ay, bx, by);
}

/** @brief Return whether a finite segment avoids a polygon's closed set.
 * @details Endpoint containment and every boundary contact are forbidden. A
 * malformed polygon or nonfinite segment fails closed.
 */
inline bool PolygonBoundaryDisjointFromSegment(
    const ::automsgs::msgs::geometry_msgs::Polygon& polygon, double ax,
    double ay, double bx, double by) {
    return ValidatePolygonBoundary(polygon) &&
           polygon_boundary_internal::ValidatedPolygonDisjointFromSegment(
               polygon, ax, ay, bx, by);
}

/** @brief Return whether a point is outside every closed exclusion polygon. */
inline bool PolygonBoundaryOutsideAll(
    const std::vector<::automsgs::msgs::geometry_msgs::Polygon>& no_go_areas,
    double x, double y) {
    return std::none_of(no_go_areas.begin(), no_go_areas.end(),
                        [&](const auto& polygon) {
                            return PolygonBoundaryContains(polygon, x, y);
                        });
}

}  // namespace autonomy::task::exploration
