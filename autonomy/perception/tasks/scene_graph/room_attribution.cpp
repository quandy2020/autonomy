#include "autonomy/perception/tasks/scene_graph/room_attribution.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy::perception::scene_graph {
namespace {

enum class Membership { kOutside, kBoundary, kInside, kInvalid };

bool Finite(Point2D point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

Membership Contains(Point2D point, const std::vector<Point2D>& polygon) {
    if (polygon.size() < 3 || !std::all_of(polygon.begin(), polygon.end(), Finite))
        return Membership::kInvalid;
    constexpr double kBoundaryToleranceM = 1e-6;
    bool inside = false;
    auto a = polygon.back();
    for (const auto b : polygon) {
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double px = point.x - a.x;
        const double py = point.y - a.y;
        const double length = std::hypot(dx, dy);
        if (!std::isfinite(length))
            return Membership::kInvalid;
        if (length > 0) {
            const double along = px * (dx / length) + py * (dy / length);
            const double perpendicular = px * (dy / length) - py * (dx / length);
            if (std::abs(perpendicular) <= kBoundaryToleranceM &&
                along >= -kBoundaryToleranceM && along <= length + kBoundaryToleranceM)
                return Membership::kBoundary;
        }
        if ((a.y > point.y) != (b.y > point.y) &&
            px < (py / dy) * dx)
            inside = !inside;
        a = b;
    }
    return inside ? Membership::kInside : Membership::kOutside;
}

}  // namespace

std::optional<std::string> FindRoom(
    Point2D map_center, const std::vector<RoomAnnotation>& rooms) {
    if (!Finite(map_center))
        return std::nullopt;
    std::optional<std::string> id;
    for (const auto& room : rooms) {
        const auto membership = Contains(map_center, room.contour);
        if (room.id.empty() || membership == Membership::kInvalid ||
            membership == Membership::kBoundary)
            return std::nullopt;
        if (membership == Membership::kInside) {
            if (id)
                return std::nullopt;
            id = room.id;
        }
    }
    return id;
}

}  // namespace autonomy::perception::scene_graph
