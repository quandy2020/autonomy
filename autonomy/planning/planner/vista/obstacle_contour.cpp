/* Copyright 2026 The Openbot Authors. */
#include "autonomy/planning/planner/vista/obstacle_contour.hpp"
#include <set>

namespace autonomy::planning::planner::vista {
std::vector<Pose2d> ObstacleContour::Extract(const perception::MapReadView& view,
                                            const std::vector<Bounds2d>& regions,
                                            const EdgeValidator& validator) const {
    const auto& map = *view.grid;
    std::set<std::pair<int,int>> cells;
    for (const auto& b : regions) {
        int x0 = std::max(0,static_cast<int>(std::floor((b.min_x-map.origin_x())/map.resolution())));
        int y0 = std::max(0,static_cast<int>(std::floor((b.min_y-map.origin_y())/map.resolution())));
        int x1 = std::min(map.width()-1,static_cast<int>(std::floor((b.max_x-map.origin_x())/map.resolution())));
        int y1 = std::min(map.height()-1,static_cast<int>(std::floor((b.max_y-map.origin_y())/map.resolution())));
        for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) cells.insert({x,y});
    }
    std::vector<Pose2d> result;
    // A larger footprint offsets candidates into known free cells; collision checking
    // uses that footprint once, rather than inflating the obstacle twice.
    const double offset = validator.safety().options().robot_radius +
                          validator.safety().options().horizontal_margin;
    const int halo = std::max(1,static_cast<int>(std::ceil(offset/map.resolution()))+1);
    for (const auto& cell : cells) {
        int x = cell.first, y = cell.second;
        if (map.At(x,y).state != opt::navigation::CellState::kFree) continue;
        bool quadrants[4]{false,false,false,false};
        for (int dy = -halo; dy <= halo; ++dy) for (int dx = -halo; dx <= halo; ++dx) {
            if (dx == 0 && dy == 0) continue;
            if (!map.Contains(x+dx,y+dy) || map.At(x+dx,y+dy).state != opt::navigation::CellState::kFree)
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
