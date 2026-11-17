/**
 * @file goal_regions.cc
 * @brief Internal Opt goal regions implementation reused by native navigation.
 * @details Keeps the geometry implementation independent of ROS and map ownership.
 */
#include "opt/navigation/goal_regions.h"

#include <algorithm>
#include <queue>
#include <stdexcept>

namespace opt::navigation {

std::vector<GeometricRegion> GeometricSemanticParser::Parse(const GridMap& map) const {
  std::vector<GeometricRegion> regions;
  std::vector<bool> seen(map.size(), false);
  constexpr int dx[] = {1, -1, 0, 0};
  constexpr int dy[] = {0, 0, 1, -1};
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (seen[map.Index(x, y)] || map.At(x, y).state == CellState::kUnknown) continue;
      GeometricRegion region;
      region.obstacle = map.At(x, y).state == CellState::kOccupied;
      region.id = std::string(region.obstacle ? "obstacle_" : "free_region_") +
                  std::to_string(map.Index(x, y));
      std::queue<CellIndex> pending;
      pending.push({x, y});
      seen[map.Index(x, y)] = true;
      while (!pending.empty()) {
        const auto cell = pending.front();
        pending.pop();
        region.cells.push_back(cell);
        const auto pose = map.CellCenter(cell.x, cell.y);
        region.center.x += pose.x;
        region.center.y += pose.y;
        for (int k = 0; k < 4; ++k) {
          const int nx = cell.x + dx[k], ny = cell.y + dy[k];
          if (!map.Contains(nx, ny) || seen[map.Index(nx, ny)] ||
              map.At(nx, ny).state != map.At(x, y).state) continue;
          seen[map.Index(nx, ny)] = true;
          pending.push({nx, ny});
        }
      }
      region.center.x /= region.cells.size();
      region.center.y /= region.cells.size();
      for (const auto cell : region.cells) {
        region.radius = std::max(region.radius,
            Distance(region.center, map.CellCenter(cell.x, cell.y)) +
            map.resolution() / std::sqrt(2.0));
      }
      regions.push_back(std::move(region));
    }
  }
  return regions;
}

ConnectedGoalRegionProvider::ConnectedGoalRegionProvider(double search_radius)
    : search_radius_(search_radius) {
  if (!std::isfinite(search_radius) || search_radius <= 0.0) {
    throw std::invalid_argument("Invalid goal-region search radius");
  }
}

std::optional<GoalRegion> ConnectedGoalRegionProvider::Resolve(const GoalRequest& goal,
                                                              const GridMap& map) const {
  if (goal.region) return goal.region;
  if (!IsFinite(goal.original) || goal.frame_id != map.frame_id()) return std::nullopt;
  const auto cell = map.WorldToCell(goal.original);
  if (!cell || map.At(cell->x, cell->y).state == CellState::kUnknown) return std::nullopt;
  for (const auto& region : GeometricSemanticParser().Parse(map)) {
    if (std::any_of(region.cells.begin(), region.cells.end(), [&](const CellIndex& index) {
          return index.x == cell->x && index.y == cell->y;
        })) {
      return GoalRegion{region.id, goal.original, search_radius_};
    }
  }
  return std::nullopt;
}

}  // namespace opt::navigation
