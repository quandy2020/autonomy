// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file region_segmenter.cpp
 * @brief Grid region segmentation implementation. */
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

  // Remove one-cell free ledges: real maps carry slivers along walls and under
  // door frames, and a 0.1 m ledge becomes a polygon edge that no
  // bounded-curvature turn can follow.
  for (int pass = 0; pass < 2; ++pass) {
    std::vector<unsigned char> eroded = free;
    for (int row = 0; row < height; ++row) {
      for (int column = 0; column < width; ++column) {
        const std::size_t index =
            static_cast<std::size_t>(row * width + column);
        if (free[index] == 0) {
          continue;
        }
        const bool solid =
            row > 0 && row + 1 < height && column > 0 && column + 1 < width &&
            free[index - 1] != 0 && free[index + 1] != 0 &&
            free[index - static_cast<std::size_t>(width)] != 0 &&
            free[index + static_cast<std::size_t>(width)] != 0;
        if (!solid) {
          eroded[index] = 0;
        }
      }
    }
    free.swap(eroded);
  }

  // Rooms linked by a doorway are one connected free-space component. Eroding
  // by half the doorway width pinches those passages shut so each room seeds
  // its own label.
  const int erosion = std::max(
      1, static_cast<int>(std::ceil(0.5 * options.min_passage_width /
                                    map.resolution())));
  std::vector<int> distance(count, erosion + 1);
  std::deque<int> pending;
  for (std::size_t index = 0; index < count; ++index) {
    if (free[index] == 0) {
      distance[index] = 0;
      pending.push_back(static_cast<int>(index));
    }
  }
  while (!pending.empty()) {
    const int index = pending.front();
    pending.pop_front();
    if (distance[static_cast<std::size_t>(index)] >= erosion + 1) {
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

  std::vector<int> label(count, -1);
  std::deque<int> frontier;
  int next_label = 0;
  for (std::size_t index = 0; index < count; ++index) {
    if (free[index] == 0 || distance[index] <= erosion || label[index] >= 0) {
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
          if (free[next] == 0 || distance[next] <= erosion ||
              label[next] >= 0) {
            continue;
          }
          label[next] = next_label;
          stack.push_back(static_cast<int>(next));
        }
      }
    }
    ++next_label;
    frontier.push_back(static_cast<int>(index));
  }
  // Narrow maps without a surviving core keep a single connected room.
  if (next_label == 0) {
    for (std::size_t index = 0; index < count; ++index) {
      if (free[index] != 0) {
        label[index] = 0;
        frontier.push_back(static_cast<int>(index));
      }
    }
    next_label = frontier.empty() ? 0 : 1;
  }
  // Grow every core back into the free space, but never further than the
  // erosion radius, otherwise a label runs through the doorway and drags a
  // tongue of one room into its neighbour.
  std::vector<int> steps(count, 0);
  while (!frontier.empty()) {
    const int index = frontier.front();
    frontier.pop_front();
    if (steps[static_cast<std::size_t>(index)] >= erosion) {
      continue;
    }
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
        if (free[next] == 0 || label[next] >= 0) {
          continue;
        }
        label[next] = current;
        steps[next] = steps[static_cast<std::size_t>(index)] + 1;
        frontier.push_back(static_cast<int>(next));
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
        if (free[index] == 0 || label[index] < 0) {
          continue;
        }
        int counts[32] = {0};
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
            if (free[neighbour] == 0 || label[neighbour] < 0 ||
                label[neighbour] >= 32) {
              continue;
            }
            ++counts[label[neighbour]];
            ++total;
          }
        }
        int winner = label[index];
        int best = 0;
        for (int candidate = 0; candidate < 32; ++candidate) {
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

  const double resolution = static_cast<double>(map.resolution());
  const double c = std::cos(map.origin_yaw());
  const double s = std::sin(map.origin_yaw());
  struct Room {
    double area;
    std::vector<std::pair<double, double>> ring;
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
    std::vector<std::vector<std::pair<int, int>>> loops;
    while (!edges.empty()) {
      std::vector<std::pair<int, int>> loop;
      const auto start_point = edges.begin()->first;
      auto current = start_point;
      while (true) {
        auto it = edges.find(current);
        if (it == edges.end() || it->second.empty()) {
          break;
        }
        const auto next = it->second.back();
        it->second.pop_back();
        if (it->second.empty()) {
          edges.erase(it);
        }
        loop.push_back(current);
        current = next;
        if (current == start_point) {
          break;
        }
      }
      if (loop.size() >= 4) {
        loops.push_back(std::move(loop));
      }
    }
    if (loops.empty()) {
      continue;
    }
    std::size_t best = 0;
    double best_area = -1.0;
    for (std::size_t index = 0; index < loops.size(); ++index) {
      double pixel_area = 0.0;
      for (std::size_t point = 0; point < loops[index].size(); ++point) {
        const auto& a = loops[index][point];
        const auto& b = loops[index][(point + 1) % loops[index].size()];
        pixel_area += static_cast<double>(a.first) * b.second -
                      static_cast<double>(b.first) * a.second;
      }
      if (std::abs(pixel_area) > best_area) {
        best_area = std::abs(pixel_area);
        best = index;
      }
    }
    const double area = best_area * 0.5 * resolution * resolution;
    if (area < options.min_area) {
      continue;
    }

    // Collapse stair-steps shorter than the turning radius. Dropping both
    // corners of a step keeps the outline rectilinear, whereas a
    // Douglas-Peucker pass introduces diagonal edges the bounded-curvature
    // connector cannot follow.
    std::vector<std::pair<int, int>> step_loop = loops[best];
    const long long step_limit = std::max<long long>(
        2, static_cast<long long>(std::llround(0.6 / resolution)));
    bool collapsed = true;
    while (collapsed && step_loop.size() > 4) {
      collapsed = false;
      const std::size_t loop_size = step_loop.size();
      for (std::size_t i = 0; i < loop_size; ++i) {
        const auto& a = step_loop[(i + loop_size - 1) % loop_size];
        const auto& b = step_loop[i];
        const auto& d = step_loop[(i + 1) % loop_size];
        const auto& e = step_loop[(i + 2) % loop_size];
        const long long abx = b.first - a.first;
        const long long aby = b.second - a.second;
        const long long bdx = d.first - b.first;
        const long long bdy = d.second - b.second;
        const long long dex = e.first - d.first;
        const long long dey = e.second - d.second;
        if (abx * bdx + aby * bdy != 0 || bdx * dex + bdy * dey != 0 ||
            abx * dey - aby * dex != 0) {
          continue;
        }
        const long long length =
            std::max(bdx < 0 ? -bdx : bdx, bdy < 0 ? -bdy : bdy);
        if (length > step_limit) {
          continue;
        }
        std::vector<std::pair<int, int>> next;
        next.reserve(loop_size - 2);
        for (std::size_t k = 0; k < loop_size; ++k) {
          if (k == i || k == (i + 1) % loop_size) {
            continue;
          }
          next.push_back(step_loop[k]);
        }
        step_loop.swap(next);
        collapsed = true;
        break;
      }
    }
    if (step_loop.size() < 4) {
      continue;
    }
    std::vector<std::pair<double, double>> ring;
    ring.reserve(step_loop.size());
    for (std::size_t point = 0; point < step_loop.size(); ++point) {
      const auto& previous =
          step_loop[(point + step_loop.size() - 1) % step_loop.size()];
      const auto& current = step_loop[point];
      const auto& next = step_loop[(point + 1) % step_loop.size()];
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
      ring.emplace_back(map.origin_x() + c * x - s * y,
                        map.origin_y() + s * x + c * y);
    }
    if (ring.size() < 4) {
      continue;
    }
    LOG(INFO) << "map split room " << label_index << " area " << area
              << " ring " << ring.size() << " first " << ring.front().first
              << "," << ring.front().second;
    rooms.push_back(Room{area, std::move(ring)});
  }

  std::sort(rooms.begin(), rooms.end(),
            [](const Room& a, const Room& b) { return a.area > b.area; });
  if (rooms.size() > static_cast<std::size_t>(options.max_regions)) {
    rooms.resize(static_cast<std::size_t>(options.max_regions));
  }
  std::vector<F2CCells> result;
  result.reserve(rooms.size());
  for (const auto& room : rooms) {
    F2CLinearRing ring;
    for (const auto& point : room.ring) {
      ring.AddPoint(point.first, point.second);
    }
    ring.CloseRing();
    result.push_back(F2CCells(F2CCell(ring)));
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
