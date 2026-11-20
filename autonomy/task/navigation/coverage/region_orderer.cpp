// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file region_orderer.cpp
 * @brief TSP ordering implementation. */
#include "autonomy/task/navigation/coverage/region_orderer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomy::task::coverage {
namespace {

double Distance(const std::pair<double, double>& a,
                const std::pair<double, double>& b) {
  return std::hypot(a.first - b.first, a.second - b.second);
}

double TourDistance(const std::vector<std::pair<double, double>>& points,
                    const std::vector<std::size_t>& order, double start_x,
                    double start_y) {
  double total = 0.0;
  double current_x = start_x;
  double current_y = start_y;
  for (const std::size_t index : order) {
    total += std::hypot(points[index].first - current_x,
                        points[index].second - current_y);
    current_x = points[index].first;
    current_y = points[index].second;
  }
  return total;
}

}  // namespace

RegionOrderer::Result RegionOrderer::Order(
    const std::vector<std::pair<double, double>>& centroids, double start_x,
    double start_y) {
  Result result;
  const std::size_t count = centroids.size();
  if (count == 0) {
    return result;
  }
  if (count == 1) {
    result.order = {0};
    result.distance = Distance(centroids.front(), {start_x, start_y});
    return result;
  }

  std::vector<bool> visited(count, false);
  result.order.reserve(count);
  double current_x = start_x;
  double current_y = start_y;
  for (std::size_t step = 0; step < count; ++step) {
    std::size_t best = count;
    double best_distance = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < count; ++index) {
      if (visited[index]) {
        continue;
      }
      const double distance = Distance(centroids[index], {current_x, current_y});
      if (distance < best_distance) {
        best_distance = distance;
        best = index;
      }
    }
    visited[best] = true;
    result.order.push_back(best);
    current_x = centroids[best].first;
    current_y = centroids[best].second;
  }

  double best_total = TourDistance(centroids, result.order, start_x, start_y);
  bool improved = true;
  while (improved) {
    improved = false;
    for (std::size_t i = 0; i < count - 1; ++i) {
      for (std::size_t j = i + 1; j < count; ++j) {
        std::vector<std::size_t> candidate = result.order;
        std::reverse(candidate.begin() + static_cast<std::ptrdiff_t>(i),
                     candidate.begin() + static_cast<std::ptrdiff_t>(j + 1));
        const double total =
            TourDistance(centroids, candidate, start_x, start_y);
        if (total + 1e-9 < best_total) {
          best_total = total;
          result.order = std::move(candidate);
          improved = true;
        }
      }
    }
  }
  result.distance = best_total;
  return result;
}


RegionOrderer::Result RegionOrderer::Order(
    const std::vector<std::vector<double>>& costs,
    const std::vector<double>& from_start) {
  Result result;
  const std::size_t count = costs.size();
  if (count == 0) {
    return result;
  }
  if (from_start.size() != count) {
    return result;
  }
  for (const auto& row : costs) {
    if (row.size() != count) {
      return result;
    }
  }
  auto total = [&costs, &from_start](const std::vector<std::size_t>& order) {
    if (order.empty()) {
      return 0.0;
    }
    double sum = from_start[order.front()];
    for (std::size_t index = 1; index < order.size(); ++index) {
      sum += costs[order[index - 1]][order[index]];
    }
    return sum;
  };

  std::vector<bool> visited(count, false);
  std::size_t current = count;
  double best_start = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < count; ++index) {
    if (from_start[index] < best_start) {
      best_start = from_start[index];
      current = index;
    }
  }
  result.order.push_back(current);
  visited[current] = true;
  for (std::size_t step = 1; step < count; ++step) {
    std::size_t best = count;
    double best_cost = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < count; ++index) {
      if (visited[index]) {
        continue;
      }
      const double cost = costs[current][index];
      if (cost < best_cost) {
        best_cost = cost;
        best = index;
      }
    }
    if (best == count) {
      break;
    }
    visited[best] = true;
    result.order.push_back(best);
    current = best;
  }

  double best_total = total(result.order);
  bool improved = true;
  while (improved) {
    improved = false;
    for (std::size_t i = 0; i < count; ++i) {
      for (std::size_t j = i + 1; j < count; ++j) {
        std::vector<std::size_t> candidate = result.order;
        std::reverse(candidate.begin() + static_cast<std::ptrdiff_t>(i),
                     candidate.begin() + static_cast<std::ptrdiff_t>(j + 1));
        const double candidate_total = total(candidate);
        if (candidate_total + 1e-9 < best_total) {
          best_total = candidate_total;
          result.order = std::move(candidate);
          improved = true;
        }
      }
    }
  }
  result.distance = best_total;
  return result;
}

}  // namespace autonomy::task::coverage
