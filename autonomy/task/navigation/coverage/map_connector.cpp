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
 * @file map_connector.cpp
 * @brief Occupancy-grid connectivity implementation.
 */

#include "autonomy/task/navigation/coverage/map_connector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_map>

namespace autonomy::task::coverage {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::max() / 4.0;

/**
 * @brief Eight-connected neighbourhood used for metric grid searches.
 */
constexpr int kSteps[8][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1},
                              {1, 1},  {1, -1}, {-1, 1}, {-1, -1}};

}  // namespace

MapConnector::MapConnector(const proto::OccupancyMap& map,
                           const Options& options) {
  width_ = static_cast<int>(map.width());
  height_ = static_cast<int>(map.height());
  resolution_ = map.resolution();
  origin_x_ = map.origin_x();
  origin_y_ = map.origin_y();
  origin_yaw_ = map.origin_yaw();
  if (width_ <= 0 || height_ <= 0 || resolution_ <= 0.0 ||
      map.data_size() < width_ * height_) {
    width_ = 0;
    height_ = 0;
    return;
  }
  free_.assign(static_cast<std::size_t>(width_ * height_), 1);
  for (int row = 0; row < height_; ++row) {
    for (int column = 0; column < width_; ++column) {
      // Occupancy grids run bottom-up in row-major order.
      const int value = map.data(row * width_ + column);
      const bool free = value >= 0 && value < options.occupied_threshold;
      free_[static_cast<std::size_t>(row * width_ + column)] =
          free ? 1 : 0;
    }
  }
  Inflate(options.clearance);
}

bool MapConnector::CellFree(int column, int row) const {
  if (column < 0 || row < 0 || column >= width_ || row >= height_) {
    return false;
  }
  return free_[static_cast<std::size_t>(row * width_ + column)] != 0;
}

std::pair<int, int> MapConnector::Cell(double x, double y) const {
  const double dx = x - origin_x_;
  const double dy = y - origin_y_;
  const double cosine = std::cos(origin_yaw_);
  const double sine = std::sin(origin_yaw_);
  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;
  return {static_cast<int>(std::floor(local_x / resolution_)),
          static_cast<int>(std::floor(local_y / resolution_))};
}

std::pair<double, double> MapConnector::World(int column, int row) const {
  const double local_x = (static_cast<double>(column) + 0.5) * resolution_;
  const double local_y = (static_cast<double>(row) + 0.5) * resolution_;
  const double cosine = std::cos(origin_yaw_);
  const double sine = std::sin(origin_yaw_);
  return {origin_x_ + cosine * local_x - sine * local_y,
          origin_y_ + sine * local_x + cosine * local_y};
}

bool MapConnector::Free(double x, double y) const {
  const auto cell = Cell(x, y);
  return CellFree(cell.first, cell.second);
}

void MapConnector::Inflate(double clearance) {
  if (clearance <= 0.0 || free_.empty()) {
    return;
  }
  const int radius = static_cast<int>(std::ceil(clearance / resolution_));
  if (radius <= 0) {
    return;
  }
  std::vector<int> distance(free_.size(), radius + 1);
  std::queue<int> frontier;
  for (std::size_t index = 0; index < free_.size(); ++index) {
    if (free_[index] == 0) {
      distance[index] = 0;
      frontier.push(static_cast<int>(index));
    }
  }
  while (!frontier.empty()) {
    const int index = frontier.front();
    frontier.pop();
    const int row = index / width_;
    const int column = index % width_;
    for (const auto& step : kSteps) {
      const int next_column = column + step[0];
      const int next_row = row + step[1];
      if (next_column < 0 || next_row < 0 || next_column >= width_ ||
          next_row >= height_) {
        continue;
      }
      const int next = next_row * width_ + next_column;
      if (distance[next] > distance[index] + 1) {
        distance[next] = distance[index] + 1;
        frontier.push(next);
      }
    }
  }
  for (std::size_t index = 0; index < free_.size(); ++index) {
    if (static_cast<double>(distance[index]) * resolution_ <= clearance) {
      free_[index] = 0;
    }
  }
}

std::pair<double, double> MapConnector::NearestFree(
    const std::pair<double, double>& point) const {
  if (!Valid() || Free(point.first, point.second)) {
    return point;
  }
  const auto start = Cell(point.first, point.second);
  const int limit = std::max(width_, height_);
  for (int radius = 1; radius <= limit; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) {
          continue;
        }
        if (CellFree(start.first + dx, start.second + dy)) {
          return World(start.first + dx, start.second + dy);
        }
      }
    }
  }
  return point;
}

MapConnector::Visit MapConnector::Connect(
    const std::pair<double, double>& from,
    const std::pair<double, double>& to) const {
  Visit visit;
  if (!Valid()) {
    return visit;
  }
  const auto snapped_from = NearestFree(from);
  const auto snapped_to = NearestFree(to);
  const auto start = Cell(snapped_from.first, snapped_from.second);
  const auto goal = Cell(snapped_to.first, snapped_to.second);
  if (!CellFree(start.first, start.second) ||
      !CellFree(goal.first, goal.second)) {
    return visit;
  }
  const std::size_t total = free_.size();
  std::vector<double> best(total, kInfinity);
  std::vector<int> parent(total, -1);
  const auto start_index =
      static_cast<std::size_t>(start.second * width_ + start.first);
  const auto goal_index =
      static_cast<std::size_t>(goal.second * width_ + goal.first);
  best[start_index] = 0.0;
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  queue.emplace(0.0, start_index);
  while (!queue.empty()) {
    const auto [cost, index] = queue.top();
    queue.pop();
    if (cost > best[index] + 1e-12) {
      continue;
    }
    if (index == goal_index) {
      break;
    }
    const int row = static_cast<int>(index) / width_;
    const int column = static_cast<int>(index) % width_;
    for (const auto& step : kSteps) {
      const int next_column = column + step[0];
      const int next_row = row + step[1];
      if (!CellFree(next_column, next_row)) {
        continue;
      }
      const auto next = static_cast<std::size_t>(next_row * width_ + next_column);
      const bool diagonal = step[0] != 0 && step[1] != 0;
      const double candidate =
          cost + (diagonal ? resolution_ * std::sqrt(2.0) : resolution_);
      if (candidate + 1e-12 < best[next]) {
        best[next] = candidate;
        parent[next] = static_cast<int>(index);
        queue.emplace(candidate, next);
      }
    }
  }
  if (best[goal_index] >= kInfinity) {
    return visit;
  }
  std::vector<std::pair<double, double>> reversed;
  for (int index = static_cast<int>(goal_index); index >= 0;
       index = parent[static_cast<std::size_t>(index)]) {
    reversed.push_back(
        World(index % width_, index / width_));
    if (static_cast<std::size_t>(index) == start_index) {
      break;
    }
  }
  std::reverse(reversed.begin(), reversed.end());
  // Collapse collinear runs so the exported path stays compact.
  visit.points.push_back(snapped_from);
  for (std::size_t index = 1; index + 1 < reversed.size(); ++index) {
    const auto& previous = reversed[index - 1];
    const auto& current = reversed[index];
    const auto& next = reversed[index + 1];
    const double cross = (current.first - previous.first) *
                             (next.second - current.second) -
                         (current.second - previous.second) *
                             (next.first - current.first);
    if (std::abs(cross) > 1e-9) {
      visit.points.push_back(current);
    }
  }
  visit.points.push_back(snapped_to);
  for (std::size_t index = 1; index < visit.points.size(); ++index) {
    visit.distance +=
        std::hypot(visit.points[index].first - visit.points[index - 1].first,
                   visit.points[index].second - visit.points[index - 1].second);
  }
  return visit;
}

std::vector<double> MapConnector::Distances(
    const std::pair<double, double>& from,
    const std::vector<std::pair<double, double>>& targets) const {
  std::vector<double> result(targets.size(), kInfinity);
  if (!Valid()) {
    for (std::size_t i = 0; i < targets.size(); ++i) {
      result[i] = std::hypot(targets[i].first - from.first,
                             targets[i].second - from.second);
    }
    return result;
  }
  const auto snapped_from = NearestFree(from);
  const auto start = Cell(snapped_from.first, snapped_from.second);
  if (!CellFree(start.first, start.second)) {
    return result;
  }
  const std::size_t total = free_.size();
  std::unordered_map<std::size_t, std::size_t> wanted;
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const auto snapped = NearestFree(targets[i]);
    const auto cell = Cell(snapped.first, snapped.second);
    if (!CellFree(cell.first, cell.second)) {
      continue;
    }
    wanted.emplace(static_cast<std::size_t>(cell.second * width_ + cell.first), i);
  }
  std::vector<double> best(total, kInfinity);
  const std::size_t start_index =
      static_cast<std::size_t>(start.second * width_ + start.first);
  best[start_index] = 0.0;
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  queue.emplace(0.0, start_index);
  std::size_t remaining = wanted.size();
  while (!queue.empty() && remaining > 0) {
    const auto [cost, index] = queue.top();
    queue.pop();
    if (cost > best[index] + 1e-12) {
      continue;
    }
    const auto hit = wanted.find(index);
    if (hit != wanted.end()) {
      result[hit->second] = cost;
      --remaining;
    }
    const int row = static_cast<int>(index) / width_;
    const int column = static_cast<int>(index) % width_;
    for (const auto& step : kSteps) {
      const int next_column = column + step[0];
      const int next_row = row + step[1];
      if (!CellFree(next_column, next_row)) {
        continue;
      }
      const auto next =
          static_cast<std::size_t>(next_row * width_ + next_column);
      const bool diagonal = step[0] != 0 && step[1] != 0;
      const double candidate =
          cost + (diagonal ? resolution_ * std::sqrt(2.0) : resolution_);
      if (candidate + 1e-12 < best[next]) {
        best[next] = candidate;
        queue.emplace(candidate, next);
      }
    }
  }
  return result;
}

double MapConnector::Distance(const std::pair<double, double>& from,
                              const std::pair<double, double>& to) const {
  const double direct = std::hypot(to.first - from.first, to.second - from.second);
  if (!Valid()) {
    return direct;
  }
  const auto visit = Connect(from, to);
  return visit.points.empty() ? kInfinity : visit.distance;
}

}  // namespace autonomy::task::coverage
