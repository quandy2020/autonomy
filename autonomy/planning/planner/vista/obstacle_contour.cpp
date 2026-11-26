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
/** @brief Classify a contour vertex from its blocked quadrants, like FAR's NodeFreeDirect. */
ContourDirect Classify(const bool quadrants[4]) {
    const int count = static_cast<int>(quadrants[0]) + quadrants[1] + quadrants[2] + quadrants[3];
    if (count == 1) return ContourDirect::kConvex;
    if (count == 3) return ContourDirect::kConcave;
    if (count == 2 && ((quadrants[0] && quadrants[3]) || (quadrants[1] && quadrants[2])))
        return ContourDirect::kPillar;
    return ContourDirect::kUnknown;
}
/** @brief Label 4-connected occupied components by their minimum linear cell index. */
std::vector<std::uint64_t> LabelObstacles(const autonomy::navigation::GridMap& map) {
    const int width = map.width(), height = map.height();
    std::vector<std::uint64_t> labels(static_cast<std::size_t>(width) * height, 0);
    std::vector<int> queue;
    queue.reserve(256);
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
        const int start = y * width + x;
        if (labels[start] || map.At(x, y).state != autonomy::navigation::CellState::kOccupied) continue;
        queue.clear(); queue.push_back(start); labels[start] = 1;
        std::uint64_t minimum = static_cast<std::uint64_t>(start);
        for (std::size_t head = 0; head < queue.size(); ++head) {
            const int cell = queue[head];
            const int cx = cell % width, cy = cell / width;
            const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
            for (int k = 0; k < 4; ++k) {
                const int nx = cx + dx[k], ny = cy + dy[k];
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
                const int next = ny * width + nx;
                if (labels[next] || map.At(nx, ny).state != autonomy::navigation::CellState::kOccupied) continue;
                labels[next] = 1; queue.push_back(next);
                minimum = std::min(minimum, static_cast<std::uint64_t>(next));
            }
        }
        for (int cell : queue) labels[cell] = minimum;
    }
    return labels;
}
}  // namespace

std::vector<ContourVertex> ObstacleContour::Extract(const perception::MapReadView& view,
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
    std::vector<ContourVertex> result;
    if (cells.empty()) return result;
    // A larger footprint offsets candidates into known free cells; collision checking
    // uses that footprint once, rather than inflating the obstacle twice.
    const double offset = validator.safety().options().robot_radius +
                          validator.safety().options().horizontal_margin;
    const long double requested_halo = std::ceil(static_cast<long double>(offset)/map.resolution())+1;
    if (!std::isfinite(requested_halo) || requested_halo >= std::numeric_limits<int>::max()/2)
        throw std::length_error("contour footprint exceeds neighborhood range");
    const int halo = std::max(1,static_cast<int>(requested_halo));
    const auto labels = LabelObstacles(map);
    for (const auto& cell : cells) {
        int x = cell.first, y = cell.second;
        if (map.At(x,y).state != autonomy::navigation::CellState::kFree) continue;
        bool quadrants[4]{false,false,false,false};
        bool frontier = false;
        std::uint64_t contour_id = 0;
        for (std::int64_t dy = -halo; dy <= halo; ++dy)
            for (std::int64_t dx = -halo; dx <= halo; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const auto nx = x+dx, ny = y+dy;
            if (nx < 0 || ny < 0 || nx >= map.width() || ny >= map.height()) continue;
            const auto& neighbour = map.At(static_cast<int>(nx),static_cast<int>(ny));
            if (neighbour.state == autonomy::navigation::CellState::kOccupied) {
                quadrants[(dx >= 0 ? 1 : 0)+(dy >= 0 ? 2 : 0)] = true;
                const auto label = labels[static_cast<std::size_t>(ny) * map.width() + nx];
                if (label && (!contour_id || label < contour_id)) contour_id = label;
            } else if (neighbour.state == autonomy::navigation::CellState::kUnknown) {
                frontier = true;
            }
        }
        const int count = static_cast<int>(quadrants[0])+quadrants[1]+quadrants[2]+quadrants[3];
        const bool boundary = x <= 1 || y <= 1 || x >= map.width()-2 || y >= map.height()-2;
        // Retain corners, doorway connections and a coarse contour sampling fallback;
        // frontier-only cells keep the graph usable where the map still ends.
        // Corners are always kept. Pillars and frontier cells use a coarse
        // stride: one obstacle cell must not add a whole neighbourhood of
        // vertices, which would reconnect much of the graph on a local change.
        const bool sampled = x % halo == 0 && y % halo == 0;
        const bool pillar = count == 2 &&
            ((quadrants[0] && quadrants[3]) || (quadrants[1] && quadrants[2]));
        const bool keep = count == 1 || count == 3 ||
            (sampled && pillar) || (sampled && count == 0 && frontier);
        if (!keep) continue;
        auto p = map.CellCenter(x,y);
        if (!validator.PoseSafe(p)) continue;
        ContourVertex vertex;
        vertex.pose = p;
        vertex.direct = Classify(quadrants);
        vertex.contour_id = contour_id;
        vertex.frontier = frontier;
        vertex.boundary = boundary;
        result.push_back(vertex);
    }
    return result;
}
}  // namespace autonomy::planning::planner::vista
