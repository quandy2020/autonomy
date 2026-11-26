// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_obstacles.cpp
 * @brief Live occupancy-grid obstacle extraction implementation. */
#include "autoviz/ui/navigation/coverage_obstacles.hpp"

#include <automsgs/msgs/map_msgs/occupancy_grid_cells.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

namespace autoviz::navigation {
namespace {

/** @brief Connected component bounds in grid-cell coordinates. */
struct Component {
  int min_x{0};
  int min_y{0};
  int max_x{0};
  int max_y{0};
  std::size_t cells{0};
};

/** @brief Rotation and translation of the grid origin. */
struct GridFrame {
  double origin_x{0.0};
  double origin_y{0.0};
  double yaw{0.0};
  double resolution{0.0};
};

GridFrame ReadFrame(
    const automsgs::msgs::map_msgs::OccupancyGrid& grid) {
  GridFrame frame;
  const auto& info = grid.info();
  frame.resolution = static_cast<double>(info.resolution());
  frame.origin_x = info.origin().position().x();
  frame.origin_y = info.origin().position().y();
  const auto& q = info.origin().orientation();
  frame.yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                         1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  return frame;
}

std::pair<double, double> ToMap(const GridFrame& frame, double x, double y) {
  const double c = std::cos(frame.yaw);
  const double s = std::sin(frame.yaw);
  return {frame.origin_x + c * x - s * y, frame.origin_y + s * x + c * y};
}

std::size_t Index(const automsgs::msgs::map_msgs::MapMetaData& info, int x,
                  int y) {
  return static_cast<std::size_t>(y) * info.width() + static_cast<std::size_t>(x);
}

bool Occupied(const automsgs::msgs::map_msgs::OccupancyGrid& grid, int x,
              int y, int threshold) {
  const auto& info = grid.info();
  if (x < 0 || y < 0 || x >= static_cast<int>(info.width()) ||
      y >= static_cast<int>(info.height())) {
    return false;
  }
  const int value = automsgs::msgs::map_msgs::OccupancyGridCell(
      grid, static_cast<int>(Index(info, x, y)));
  return value >= threshold;
}

void Mix(std::uint64_t* hash, std::uint64_t value) {
  *hash ^= value;
  *hash *= 1099511628211ULL;
}

std::uint64_t Bits(double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(double));
  return bits;
}

}  // namespace

std::vector<ObstacleRing> CoverageObstacleExtractor::Extract(
    const automsgs::msgs::map_msgs::OccupancyGrid& grid,
    const ObstacleExtractionOptions& options) {
  const auto& info = grid.info();
  const int width = static_cast<int>(info.width());
  const int height = static_cast<int>(info.height());
  if (width <= 0 || height <= 0 || info.resolution() <= 0.f ||
      automsgs::msgs::map_msgs::OccupancyGridCellCount(grid) !=
          width * height) {
    return {};
  }

  const GridFrame frame = ReadFrame(grid);
  std::vector<unsigned char> visited(static_cast<std::size_t>(width * height), 0);
  std::vector<ObstacleRing> rings;
  std::deque<std::pair<int, int>> frontier;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const std::size_t index = Index(info, x, y);
      if (visited[index] ||
          !Occupied(grid, x, y, options.lethal_threshold)) {
        continue;
      }
      Component component{x, y, x, y, 0};
      frontier.clear();
      frontier.emplace_back(x, y);
      visited[index] = 1;
      while (!frontier.empty()) {
        const auto [cx, cy] = frontier.front();
        frontier.pop_front();
        ++component.cells;
        component.min_x = std::min(component.min_x, cx);
        component.max_x = std::max(component.max_x, cx);
        component.min_y = std::min(component.min_y, cy);
        component.max_y = std::max(component.max_y, cy);
        const int neighbours[4][2] = {
            {cx - 1, cy}, {cx + 1, cy}, {cx, cy - 1}, {cx, cy + 1}};
        for (const auto& neighbour : neighbours) {
          const int nx = neighbour[0];
          const int ny = neighbour[1];
          if (!Occupied(grid, nx, ny, options.lethal_threshold)) {
            continue;
          }
          const std::size_t next = Index(info, nx, ny);
          if (visited[next]) {
            continue;
          }
          visited[next] = 1;
          frontier.emplace_back(nx, ny);
        }
      }
      if (component.cells == 0 ||
          component.cells > options.max_component_cells) {
        continue;
      }
      const double min_x =
          static_cast<double>(component.min_x) * frame.resolution -
          options.margin;
      const double max_x =
          static_cast<double>(component.max_x + 1) * frame.resolution +
          options.margin;
      const double min_y =
          static_cast<double>(component.min_y) * frame.resolution -
          options.margin;
      const double max_y =
          static_cast<double>(component.max_y + 1) * frame.resolution +
          options.margin;
      ObstacleRing ring;
      ring.cells = component.cells;
      ring.points = {ToMap(frame, min_x, min_y), ToMap(frame, max_x, min_y),
                     ToMap(frame, max_x, max_y), ToMap(frame, min_x, max_y)};
      rings.push_back(std::move(ring));
    }
  }

  // Static walls occupy far more cells than movable obstacles. Keeping the
  // largest blobs first makes a size cap deterministic when a busy map
  // produces many small returns.
  std::stable_sort(rings.begin(), rings.end(),
                   [](const ObstacleRing& a, const ObstacleRing& b) {
                     return a.cells > b.cells;
                   });
  if (rings.size() > options.max_rings) {
    rings.resize(options.max_rings);
  }
  return rings;
}

std::uint64_t CoverageObstacleExtractor::Signature(
    const automsgs::msgs::map_msgs::OccupancyGrid& grid,
    int lethal_threshold) {
  const auto& info = grid.info();
  std::uint64_t hash = 1469598103934665603ULL;
  Mix(&hash, info.width());
  Mix(&hash, info.height());
  Mix(&hash, Bits(static_cast<double>(info.resolution())));
  Mix(&hash, Bits(info.origin().position().x()));
  Mix(&hash, Bits(info.origin().position().y()));
  Mix(&hash, Bits(ReadFrame(grid).yaw));
  const int width = static_cast<int>(info.width());
  const int height = static_cast<int>(info.height());
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (Occupied(grid, x, y, lethal_threshold)) {
        Mix(&hash, static_cast<std::uint64_t>(Index(info, x, y)));
      }
    }
  }
  return hash;
}

}  // namespace autoviz::navigation
