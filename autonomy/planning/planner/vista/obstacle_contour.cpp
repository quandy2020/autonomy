/* Copyright 2026 The Openbot Authors. */
/** @file obstacle_contour.cpp
 * @brief Clip world regions before indexing contour neighborhoods.
 */
#include "autonomy/planning/planner/vista/obstacle_contour.hpp"
#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>

namespace autonomy::planning::planner::vista {
namespace {
/** @brief Clamp continuous cell coordinates before their checked integer conversion. */
int CellCoordinate(double world, double origin, double resolution, int count) {
    const long double cell = std::floor((static_cast<long double>(world) - origin) / resolution);
    if (cell <= 0) return 0;
    if (cell >= count - 1) return count - 1;
    return static_cast<int>(cell);
}
}  // namespace
std::vector<Pose2d> ObstacleContour::Extract(const perception::MapReadView& view,
                                            const std::vector<Bounds2d>& regions,
                                            const EdgeValidator& validator) const {
    const auto& map = *view.grid;
    const double max_x = map.origin_x()+map.width()*map.resolution();
    const double max_y = map.origin_y()+map.height()*map.resolution();
    std::set<std::pair<int,int>> cells;
    for (const auto& b : regions) {
        if (!std::isfinite(b.min_x) || !std::isfinite(b.min_y) ||
            !std::isfinite(b.max_x) || !std::isfinite(b.max_y) ||
            b.min_x > b.max_x || b.min_y > b.max_y)
            throw std::invalid_argument("invalid contour update bounds");
        if (b.max_x < map.origin_x() || b.max_y < map.origin_y() ||
            b.min_x >= max_x || b.min_y >= max_y) continue;
        int x0 = CellCoordinate(b.min_x,map.origin_x(),map.resolution(),map.width());
        int y0 = CellCoordinate(b.min_y,map.origin_y(),map.resolution(),map.height());
        int x1 = CellCoordinate(b.max_x,map.origin_x(),map.resolution(),map.width());
        int y1 = CellCoordinate(b.max_y,map.origin_y(),map.resolution(),map.height());
        for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) cells.insert({x,y});
    }
    std::vector<Pose2d> result;
    // A larger footprint offsets candidates into known free cells; collision checking
    // uses that footprint once, rather than inflating the obstacle twice.
    const double offset = validator.safety().options().robot_radius +
                          validator.safety().options().horizontal_margin;
    const long double requested_halo = std::ceil(static_cast<long double>(offset)/map.resolution())+1;
    if (!std::isfinite(requested_halo) || requested_halo >= std::numeric_limits<int>::max()/2)
        throw std::length_error("contour footprint exceeds neighborhood range");
    const int halo = std::max(1,static_cast<int>(requested_halo));
    for (const auto& cell : cells) {
        int x = cell.first, y = cell.second;
        if (map.At(x,y).state != opt::navigation::CellState::kFree) continue;
        bool quadrants[4]{false,false,false,false};
        for (std::int64_t dy = -halo; dy <= halo; ++dy)
            for (std::int64_t dx = -halo; dx <= halo; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const auto nx = x+dx, ny = y+dy;
            if (nx < 0 || ny < 0 || nx >= map.width() || ny >= map.height() ||
                map.At(static_cast<int>(nx),static_cast<int>(ny)).state != opt::navigation::CellState::kFree)
                quadrants[(dx >= 0 ? 1 : 0)+(dy >= 0 ? 2 : 0)] = true;
        }
        const int count = static_cast<int>(quadrants[0])+quadrants[1]+quadrants[2]+quadrants[3];
        if (count == 0 || count == 4) continue;
        // Retain corners, doorway connections and a coarse contour sampling fallback.
        if (count == 1 || count == 3 || (x%halo == 0 && y%halo == 0)) {
            auto p = map.CellCenter(x,y); if (validator.PoseSafe(p)) result.push_back(p);
        }
    }
    return result;
}
}  // namespace autonomy::planning::planner::vista
