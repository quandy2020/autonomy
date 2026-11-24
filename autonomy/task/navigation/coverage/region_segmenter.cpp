/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file region_segmenter.cpp
 * @brief Grid region segmentation implementation.
 */

#include "autonomy/task/navigation/coverage/region_segmenter.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>

#include "autonomy/common/logging.hpp"
#include <limits>
#include <utility>

#include "autonomy/task/navigation/coverage/generator_utils.hpp"

#include <opencv2/imgproc.hpp>

namespace autonomy::task::coverage {


double RegionSegmenter::Area(const F2CCells& cells) {
  double area = 0.0;
  for (const auto& cell : cells) {
    area += cell.Area();
  }
  return area;
}

namespace {

/**
 * @brief Tests a cell-corner point against a rectilinear boundary loop.
 */
bool PointInLoop(int x, int y, const std::vector<std::pair<int, int>>& loop) {
  bool inside = false;
  for (std::size_t i = 0, j = loop.size() - 1; i < loop.size(); j = i++) {
    const int xi = loop[i].first;
    const int yi = loop[i].second;
    const int xj = loop[j].first;
    const int yj = loop[j].second;
    if (((yi > y) != (yj > y)) &&
        (x < (xj - xi) * (y - yi) / static_cast<double>(yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace

std::vector<F2CCells> RegionSegmenter::MapSplit(
    const proto::OccupancyMap& map, const MapOptions& options) {
  const int width = static_cast<int>(map.width());
  const int height = static_cast<int>(map.height());
  if (width <= 0 || height <= 0 || map.resolution() <= 0.f ||
      map.data_size() != width * height) {
    throw RequestException("Map segmentation requires a full occupancy grid");
  }
  const std::size_t count =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  std::vector<unsigned char> free(count, 0);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int value = map.data(y * width + x);
      if (value >= 0 && value < options.occupied_threshold) {
        free[static_cast<std::size_t>(y * width + x)] = 1;
      }
    }
  }

  // Room seeds are found on a slightly eroded mask. Real maps carry one-cell
  // free slivers along walls and under door frames, and those slivers would
  // otherwise seed their own regions. The erosion is only used to find the
  // seeds: the regions themselves grow over the full free space below.
  std::vector<unsigned char> core_free = free;
  for (int pass = 0; pass < 2; ++pass) {
    std::vector<unsigned char> eroded = core_free;
    for (int row = 0; row < height; ++row) {
      for (int column = 0; column < width; ++column) {
        const std::size_t index =
            static_cast<std::size_t>(row * width + column);
        if (core_free[index] == 0) {
          continue;
        }
        const bool solid =
            row > 0 && row + 1 < height && column > 0 && column + 1 < width &&
            core_free[index - 1] != 0 && core_free[index + 1] != 0 &&
            core_free[index - static_cast<std::size_t>(width)] != 0 &&
            core_free[index + static_cast<std::size_t>(width)] != 0;
        if (!solid) {
          eroded[index] = 0;
        }
      }
    }
    core_free.swap(eroded);
  }

  const double resolution = static_cast<double>(map.resolution());
  // Rooms linked by a doorway are one connected free-space component.
  // Eroding by half the minimum passage width pinches those passages shut
  // so each room seeds its own label.
  const int erosion = std::max(
      1, static_cast<int>(std::ceil(0.5 * options.min_passage_width /
                                    resolution)));
  std::vector<int> core_distance(count, erosion + 1);
  std::deque<int> pending;
  for (std::size_t index = 0; index < count; ++index) {
    if (core_free[index] == 0) {
      core_distance[index] = 0;
      pending.push_back(static_cast<int>(index));
    }
  }
  while (!pending.empty()) {
    const int index = pending.front();
    pending.pop_front();
    if (core_distance[static_cast<std::size_t>(index)] >= erosion + 1) {
      continue;
    }
    const int row = index / width;
    const int column = index % width;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int next_column = column + dx;
        const int next_row = row + dy;
        if (next_column < 0 || next_row < 0 || next_column >= width ||
            next_row >= height) {
          continue;
        }
        const int next = next_row * width + next_column;
        if (core_distance[static_cast<std::size_t>(next)] >
            core_distance[static_cast<std::size_t>(index)] + 1) {
          core_distance[static_cast<std::size_t>(next)] =
              core_distance[static_cast<std::size_t>(index)] + 1;
          pending.push_back(next);
        }
      }
    }
  }

  std::vector<int> label(count, -1);
  std::deque<int> frontier;
  int next_label = 0;
  for (std::size_t index = 0; index < count; ++index) {
    if (core_free[index] == 0 || core_distance[index] <= erosion ||
        label[index] >= 0) {
      continue;
    }
    // Spread one label across the whole eroded core of this room.
    label[index] = next_label;
    std::vector<int> stack{static_cast<int>(index)};
    while (!stack.empty()) {
      const int current = stack.back();
      stack.pop_back();
      const int row = current / width;
      const int column = current % width;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int next_column = column + dx;
          const int next_row = row + dy;
          if (next_column < 0 || next_row < 0 || next_column >= width ||
              next_row >= height) {
            continue;
          }
          const std::size_t next =
              static_cast<std::size_t>(next_row * width + next_column);
          if (core_free[next] == 0 || core_distance[next] <= erosion ||
              label[next] >= 0) {
            continue;
          }
          label[next] = next_label;
          stack.push_back(static_cast<int>(next));
        }
      }
    }
    ++next_label;
  }
  // Narrow maps without a surviving core keep a single connected room.
  if (next_label == 0) {
    for (std::size_t index = 0; index < count; ++index) {
      if (free[index] != 0) {
        label[index] = 0;
      }
    }
    next_label = count > 0 ? 1 : 0;
  }

  // The planned path keeps the robot body clear of walls, so cells closer
  // than the robot radius to a wall are left out of every region. The
  // coverage generator would otherwise place a swath on the wall itself.
  // A zero clearance means the caller wants the complete free-space region;
  // the coverage planner will keep the robot away from walls with a headland
  // band.  A positive clearance keeps the old inset behaviour for callers
  // that do not generate a headland.
  const int clearance_cells =
      options.clearance <= 0.0
          ? 0
          : std::max(1, static_cast<int>(std::ceil(
                            options.clearance / resolution - 1e-9)));
  std::vector<int> distance(count, clearance_cells);
  for (std::size_t index = 0; index < count; ++index) {
    if (free[index] == 0) {
      distance[index] = 0;
      pending.push_back(static_cast<int>(index));
    }
  }
  while (!pending.empty()) {
    const int index = pending.front();
    pending.pop_front();
    if (distance[static_cast<std::size_t>(index)] >= clearance_cells) {
      continue;
    }
    const int row = index / width;
    const int column = index % width;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int next_column = column + dx;
        const int next_row = row + dy;
        if (next_column < 0 || next_row < 0 || next_column >= width ||
            next_row >= height) {
          continue;
        }
        const int next = next_row * width + next_column;
        if (distance[static_cast<std::size_t>(next)] >
            distance[static_cast<std::size_t>(index)] + 1) {
          distance[static_cast<std::size_t>(next)] =
              distance[static_cast<std::size_t>(index)] + 1;
          pending.push_back(next);
        }
      }
    }
  }

  // Grow every seed over the full free space, but only through cells that
  // keep the robot clear of the walls. The frontier starts on the whole
  // core, so each room regains its full extent; where two rooms meet in a
  // doorway the two labels split the passage in the middle.
  for (std::size_t index = 0; index < count; ++index) {
    if (label[index] >= 0) {
      frontier.push_back(static_cast<int>(index));
    }
  }
  while (!frontier.empty()) {
    const int index = frontier.front();
    frontier.pop_front();
    const int row = index / width;
    const int column = index % width;
    const int current = label[static_cast<std::size_t>(index)];
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (std::abs(dx) + std::abs(dy) != 1) {
          continue;
        }
        const int next_column = column + dx;
        const int next_row = row + dy;
        if (next_column < 0 || next_row < 0 || next_column >= width ||
            next_row >= height) {
          continue;
        }
        const std::size_t next =
            static_cast<std::size_t>(next_row * width + next_column);
        if (free[next] == 0 || distance[next] < clearance_cells ||
            label[next] >= 0) {
          continue;
        }
        label[next] = current;
        frontier.push_back(static_cast<int>(next));
      }
    }
  }

  // The primary growth deliberately stops at the robot clearance so rooms
  // linked by a doorway do not merge.  Extend each label over the remaining
  // wall-adjacent strip, but only for the same short distance.  The coverage
  // planner uses this strip as a headland band; without it the strip is a
  // permanent grey hole in the visualisation.
  if (clearance_cells > 0) {
    std::vector<int> fill_distance(count, clearance_cells + 1);
    std::deque<int> fill;
    for (std::size_t index = 0; index < count; ++index) {
      if (label[index] >= 0) {
        fill_distance[index] = 0;
        fill.push_back(static_cast<int>(index));
      }
    }
    while (!fill.empty()) {
      const int index = fill.front();
      fill.pop_front();
      if (fill_distance[static_cast<std::size_t>(index)] >= clearance_cells) {
        continue;
      }
      const int row = index / width;
      const int column = index % width;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (std::abs(dx) + std::abs(dy) != 1) {
            continue;
          }
          const int next_column = column + dx;
          const int next_row = row + dy;
          if (next_column < 0 || next_row < 0 || next_column >= width ||
              next_row >= height) {
            continue;
          }
          const std::size_t next =
              static_cast<std::size_t>(next_row * width + next_column);
          if (free[next] == 0 || label[next] >= 0 ||
              fill_distance[next] <= fill_distance[static_cast<std::size_t>(index)] + 1) {
            continue;
          }
          fill_distance[next] =
              fill_distance[static_cast<std::size_t>(index)] + 1;
          label[next] = label[static_cast<std::size_t>(index)];
          fill.push_back(static_cast<int>(next));
        }
      }
    }
  }

  // A free component without a core is a narrow room or a corridor that is
  // still wide enough for the robot. Seed it at its widest cells and grow it
  // the same way, so it becomes its own region instead of being lost.
  std::vector<unsigned char> seen(count, 0);
  for (std::size_t seed = 0; seed < count; ++seed) {
    if (free[seed] == 0 || label[seed] >= 0 || seen[seed] != 0) {
      continue;
    }
    std::vector<int> component;
    std::deque<int> pending_component{static_cast<int>(seed)};
    seen[seed] = 1;
    while (!pending_component.empty()) {
      const int index = pending_component.front();
      pending_component.pop_front();
      component.push_back(index);
      const int row = index / width;
      const int column = index % width;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (std::abs(dx) + std::abs(dy) != 1) {
            continue;
          }
          const int next_column = column + dx;
          const int next_row = row + dy;
          if (next_column < 0 || next_row < 0 || next_column >= width ||
              next_row >= height) {
            continue;
          }
          const std::size_t next =
              static_cast<std::size_t>(next_row * width + next_column);
          if (free[next] == 0 || label[next] >= 0 || seen[next] != 0) {
            continue;
          }
          seen[next] = 1;
          pending_component.push_back(static_cast<int>(next));
        }
      }
    }
    int best_distance = 0;
    for (const int index : component) {
      best_distance = std::max(
          best_distance, distance[static_cast<std::size_t>(index)]);
    }
    if (best_distance < clearance_cells) {
      continue;
    }
    const int new_label = next_label++;
    std::deque<int> fill;
    for (const int index : component) {
      if (distance[static_cast<std::size_t>(index)] == best_distance) {
        label[index] = new_label;
        fill.push_back(index);
      }
    }
    while (!fill.empty()) {
      const int index = fill.front();
      fill.pop_front();
      const int row = index / width;
      const int column = index % width;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (std::abs(dx) + std::abs(dy) != 1) {
            continue;
          }
          const int next_column = column + dx;
          const int next_row = row + dy;
          if (next_column < 0 || next_row < 0 || next_column >= width ||
              next_row >= height) {
            continue;
          }
          const std::size_t next =
              static_cast<std::size_t>(next_row * width + next_column);
          if (free[next] == 0 || distance[next] < clearance_cells ||
              label[next] >= 0) {
            continue;
          }
          label[next] = new_label;
          fill.push_back(static_cast<int>(next));
        }
      }
    }
  }

  // Majority filtering removes the one-cell ragged edge the flood fill leaves
  // along walls.
  for (int pass = 0; pass < 2; ++pass) {
    std::vector<int> smoothed = label;
    for (int row = 0; row < height; ++row) {
      for (int column = 0; column < width; ++column) {
        const std::size_t index =
            static_cast<std::size_t>(row * width + column);
        if (label[index] < 0) {
          continue;
        }
        int counts[64] = {0};
        int total = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nc = column + dx;
            const int nr = row + dy;
            if (nc < 0 || nr < 0 || nc >= width || nr >= height) {
              continue;
            }
            const std::size_t neighbour =
                static_cast<std::size_t>(nr * width + nc);
            if (label[neighbour] < 0 || label[neighbour] >= 64) {
              continue;
            }
            ++counts[label[neighbour]];
            ++total;
          }
        }
        int winner = label[index];
        int best = 0;
        for (int candidate = 0; candidate < 64; ++candidate) {
          if (counts[candidate] > best) {
            best = counts[candidate];
            winner = candidate;
          }
        }
        if (best >= 5 && best * 10 >= total * 6) {
          smoothed[index] = winner;
        }
      }
    }
    label.swap(smoothed);
  }

  // A cell pair touching only at a corner makes the boundary walk ambiguous
  // and produces diagonal jumps. Fill one of the two free corners of every
  // such 2x2 pattern so each region stays 4-connected.
  for (int pass = 0; pass < 3; ++pass) {
    bool fixed = false;
    for (int row = 0; row + 1 < height; ++row) {
      for (int column = 0; column + 1 < width; ++column) {
        const std::size_t a =
            static_cast<std::size_t>(row * width + column);
        const std::size_t b = a + 1;
        const std::size_t d = a + static_cast<std::size_t>(width);
        const std::size_t e = d + 1;
        if (label[a] < 0 || label[a] != label[e] || label[b] == label[a] ||
            label[d] == label[a]) {
          continue;
        }
        if (free[b] != 0) {
          label[b] = label[a];
          fixed = true;
        } else if (free[d] != 0) {
          label[d] = label[a];
          fixed = true;
        }
      }
    }
    if (!fixed) {
      break;
    }
  }

  const double c = std::cos(map.origin_yaw());
  const double s = std::sin(map.origin_yaw());
  struct Loop {
    std::vector<std::pair<int, int>> points;
    double signed_area{0.0};
  };
  struct Room {
    double area;
    F2CCells cells;
  };
  std::vector<Room> rooms;
  for (int label_index = 0; label_index < next_label; ++label_index) {
    // Trace the boundary on cell corners so every edge is axis aligned.
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> edges;
    auto inside = [&](int column, int row) {
      if (column < 0 || row < 0 || column >= width || row >= height) {
        return false;
      }
      return label[static_cast<std::size_t>(row * width + column)] ==
             label_index;
    };
    bool any = false;
    for (int row = 0; row < height; ++row) {
      for (int column = 0; column < width; ++column) {
        if (!inside(column, row)) {
          continue;
        }
        any = true;
        if (!inside(column, row + 1)) {
          edges[{column, row + 1}].emplace_back(column + 1, row + 1);
        }
        if (!inside(column + 1, row)) {
          edges[{column + 1, row + 1}].emplace_back(column + 1, row);
        }
        if (!inside(column, row - 1)) {
          edges[{column + 1, row}].emplace_back(column, row);
        }
        if (!inside(column - 1, row)) {
          edges[{column, row}].emplace_back(column, row + 1);
        }
      }
    }
    if (!any) {
      continue;
    }
    std::vector<Loop> loops;
    while (!edges.empty()) {
      Loop loop;
      const auto start_point = edges.begin()->first;
      auto current = start_point;
      std::pair<int, int> heading{0, 0};
      while (true) {
        auto it = edges.find(current);
        if (it == edges.end() || it->second.empty()) {
          break;
        }
        // Prefer the sharpest right turn so the walk keeps the room on its
        // right side when a corner has more than one outgoing edge.
        std::size_t chosen = 0;
        int chosen_rank = 5;
        for (std::size_t candidate = 0; candidate < it->second.size();
             ++candidate) {
          const int dx = it->second[candidate].first - current.first;
          const int dy = it->second[candidate].second - current.second;
          int rank = 0;
          if (heading.first == 0 && heading.second == 0) {
            rank = 0;
          } else if (dx == heading.second && dy == -heading.first) {
            rank = 0;  // right turn
          } else if (dx == heading.first && dy == heading.second) {
            rank = 1;  // straight
          } else if (dx == -heading.second && dy == heading.first) {
            rank = 2;  // left turn
          } else {
            rank = 3;  // reverse
          }
          if (rank < chosen_rank) {
            chosen_rank = rank;
            chosen = candidate;
          }
        }
        const auto next = it->second[chosen];
        it->second.erase(it->second.begin() +
                         static_cast<std::ptrdiff_t>(chosen));
        if (it->second.empty()) {
          edges.erase(it);
        }
        heading = {next.first - current.first, next.second - current.second};
        loop.points.push_back(current);
        current = next;
        if (current == start_point) {
          break;
        }
      }
      if (loop.points.size() >= 4) {
        double twice_area = 0.0;
        for (std::size_t point = 0; point < loop.points.size(); ++point) {
          const auto& a = loop.points[point];
          const auto& b = loop.points[(point + 1) % loop.points.size()];
          twice_area += static_cast<double>(a.first) * b.second -
                        static_cast<double>(b.first) * a.second;
        }
        loop.signed_area = 0.5 * twice_area;
        loops.push_back(std::move(loop));
      }
    }
    if (loops.empty()) {
      continue;
    }
    const auto build_ring = [&](const std::vector<std::pair<int, int>>& points) {
      F2CLinearRing ring;
      for (std::size_t point = 0; point < points.size(); ++point) {
        const auto& previous = points[(point + points.size() - 1) % points.size()];
        const auto& current = points[point];
        const auto& next = points[(point + 1) % points.size()];
        const long long cross =
            (static_cast<long long>(current.first) - previous.first) *
                (static_cast<long long>(next.second) - current.second) -
            (static_cast<long long>(current.second) - previous.second) *
                (static_cast<long long>(next.first) - current.first);
        if (cross == 0) {
          continue;
        }
        const double x = static_cast<double>(current.first) * resolution;
        const double y = static_cast<double>(current.second) * resolution;
        ring.AddPoint(map.origin_x() + c * x - s * y,
                      map.origin_y() + s * x + c * y);
      }
      if (ring.size() >= 3) {
        ring.CloseRing();
      }
      return ring;
    };
    // The boundary walk keeps the region on its right, so outer loops run
    // clockwise (negative area) and holes run counter-clockwise. One cell is
    // emitted per outer loop and every contained hole is attached so the
    // planner never sweeps through a pillar or a wall stub.
    for (const auto& outer : loops) {
      if (outer.signed_area >= 0.0) {
        continue;
      }
      const double area = -outer.signed_area * resolution * resolution;
      if (area < options.min_area) {
        continue;
      }
      F2CLinearRing ring = build_ring(outer.points);
      if (ring.size() < 4) {
        continue;
      }
      F2CCell cell(ring);
      for (const auto& hole : loops) {
        if (hole.signed_area <= 0.0) {
          continue;
        }
        if (!PointInLoop(hole.points.front().first,
                         hole.points.front().second, outer.points)) {
          continue;
        }
        F2CLinearRing hole_ring = build_ring(hole.points);
        if (hole_ring.size() >= 4) {
          cell.AddRing(hole_ring);
        }
      }
      rooms.push_back(Room{area, F2CCells(cell)});
    }
  }

  std::sort(rooms.begin(), rooms.end(),
            [](const Room& a, const Room& b) { return a.area > b.area; });
  if (rooms.size() > static_cast<std::size_t>(options.max_regions)) {
    rooms.resize(static_cast<std::size_t>(options.max_regions));
  }
  std::vector<F2CCells> result;
  result.reserve(rooms.size());
  for (const auto& room : rooms) {
    result.push_back(room.cells);
  }
  if (result.empty()) {
    throw RequestException("Map segmentation found no free-space rooms");
  }
  return result;
}

std::vector<F2CCells> RegionSegmenter::GridSplit(const F2CCells& field,
                                                 const Options& options) {
  if (options.columns < 1 || options.columns > 8 || options.rows < 1 ||
      options.rows > 8) {
    throw RequestException("Region grid must be 1..8 by 1..8");
  }
  if (field.IsEmpty() || Area(field) <= 1e-6) {
    throw RequestException("Region split requires a non-empty field");
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (std::size_t index = 0; index < field.size(); ++index) {
    for (const auto& point : field.GetCellBorder(index)) {
      min_x = std::min(min_x, point.GetX());
      min_y = std::min(min_y, point.GetY());
      max_x = std::max(max_x, point.GetX());
      max_y = std::max(max_y, point.GetY());
    }
  }
  const double width = (max_x - min_x) / static_cast<double>(options.columns);
  const double height = (max_y - min_y) / static_cast<double>(options.rows);
  if (width <= 1e-6 || height <= 1e-6) {
    throw RequestException("Field is too small for the requested region grid");
  }

  std::vector<F2CCells> regions;
  regions.reserve(static_cast<std::size_t>(options.columns * options.rows));
  for (int row = 0; row < options.rows; ++row) {
    for (int column = 0; column < options.columns; ++column) {
      const double left = min_x + width * static_cast<double>(column);
      const double bottom = min_y + height * static_cast<double>(row);
      F2CLinearRing ring;
      ring.AddPoint(left, bottom);
      ring.AddPoint(left + width, bottom);
      ring.AddPoint(left + width, bottom + height);
      ring.AddPoint(left, bottom + height);
      ring.CloseRing();
      regions.push_back(F2CCells(F2CCell(ring)));
    }
  }
  if (regions.empty()) {
    throw RequestException("Region split produced no regions");
  }
  return regions;
}

}  // namespace autonomy::task::coverage
