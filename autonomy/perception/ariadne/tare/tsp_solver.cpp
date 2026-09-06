/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/tsp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace autonomy::perception::exploration {

TspSolver::TspSolver(DataModel data, int max_exact_nodes)
    : data_(std::move(data)), max_exact_nodes_(max_exact_nodes) {}

int TspSolver::ArcCost(int from, int to) const {
  if (from < 0 || to < 0 ||
      from >= static_cast<int>(data_.distance_matrix.size()) ||
      to >= static_cast<int>(data_.distance_matrix.size())) {
    return kInfCost;
  }
  return data_.distance_matrix[static_cast<size_t>(from)]
                            [static_cast<size_t>(to)];
}

std::vector<int> TspSolver::SolveGreedy() const {
  const int n = static_cast<int>(data_.distance_matrix.size());
  if (n == 0) {
    return {};
  }
  std::vector<bool> visited(static_cast<size_t>(n), false);
  std::vector<int> order;
  order.reserve(static_cast<size_t>(n));
  int current = data_.depot;
  visited[static_cast<size_t>(current)] = true;
  order.push_back(current);
  for (int step = 1; step < n; ++step) {
    int best = -1;
    int best_cost = kInfCost;
    for (int i = 0; i < n; ++i) {
      if (visited[static_cast<size_t>(i)]) {
        continue;
      }
      const int cost = ArcCost(current, i);
      if (cost < best_cost) {
        best_cost = cost;
        best = i;
      }
    }
    if (best < 0) {
      break;
    }
    visited[static_cast<size_t>(best)] = true;
    order.push_back(best);
    current = best;
  }
  return order;
}

std::vector<int> TspSolver::SolveExact() const {
  const int n = static_cast<int>(data_.distance_matrix.size());
  if (n <= 1) {
    return n == 1 ? std::vector<int>{0} : std::vector<int>{};
  }
  const int full_mask = (1 << n) - 1;
  const int start = data_.depot;
  std::vector<std::vector<int>> dp(static_cast<size_t>(1 << n),
                                   std::vector<int>(static_cast<size_t>(n),
                                                    kInfCost));
  std::vector<std::vector<int>> parent(
      static_cast<size_t>(1 << n),
      std::vector<int>(static_cast<size_t>(n), -1));
  dp[static_cast<size_t>(1 << start)][static_cast<size_t>(start)] = 0;
  for (int mask = 0; mask <= full_mask; ++mask) {
    for (int u = 0; u < n; ++u) {
      if ((mask & (1 << u)) == 0) {
        continue;
      }
      const int base = dp[static_cast<size_t>(mask)][static_cast<size_t>(u)];
      if (base >= kInfCost) {
        continue;
      }
      for (int v = 0; v < n; ++v) {
        if (mask & (1 << v)) {
          continue;
        }
        const int next_mask = mask | (1 << v);
        const int cost = base + ArcCost(u, v);
        if (cost < dp[static_cast<size_t>(next_mask)][static_cast<size_t>(v)]) {
          dp[static_cast<size_t>(next_mask)][static_cast<size_t>(v)] = cost;
          parent[static_cast<size_t>(next_mask)][static_cast<size_t>(v)] = u;
        }
      }
    }
  }
  int best_end = start;
  int best_cost = kInfCost;
  for (int v = 0; v < n; ++v) {
    const int cost =
        dp[static_cast<size_t>(full_mask)][static_cast<size_t>(v)] +
        ArcCost(v, start);
    if (cost < best_cost) {
      best_cost = cost;
      best_end = v;
    }
  }
  std::vector<int> order;
  int mask = full_mask;
  int node = best_end;
  while (node != -1) {
    order.push_back(node);
    const int prev = parent[static_cast<size_t>(mask)][static_cast<size_t>(node)];
    mask &= ~(1 << node);
    node = prev;
  }
  std::reverse(order.begin(), order.end());
  if (!order.empty() && order.front() != start) {
    order.insert(order.begin(), start);
  }
  return order;
}

std::vector<int> TspSolver::SolveTwoOpt(const std::vector<int>& order) const {
  if (order.size() < 4) {
    return order;
  }
  std::vector<int> best = order;
  bool improved = true;
  while (improved) {
    improved = false;
    for (size_t i = 1; i + 1 < best.size(); ++i) {
      for (size_t k = i + 1; k < best.size(); ++k) {
        const int a = best[i - 1];
        const int b = best[i];
        const int c = best[k];
        const int d = best[(k + 1) % best.size()];
        const int before = ArcCost(a, b) + ArcCost(c, d);
        const int after = ArcCost(a, c) + ArcCost(b, d);
        if (after < before) {
          std::reverse(best.begin() + static_cast<std::ptrdiff_t>(i),
                       best.begin() + static_cast<std::ptrdiff_t>(k + 1));
          improved = true;
        }
      }
    }
  }
  return best;
}

#ifndef AUTONOMY_HAS_ORTOOLS
std::vector<int> TspSolver::SolveOrtools() const { return {}; }
#endif

std::vector<int> TspSolver::Solve(const SolveOptions& options) const {
  const int n = static_cast<int>(data_.distance_matrix.size());
  if (n <= 1) {
    return n == 1 ? std::vector<int>{0} : std::vector<int>{};
  }

  if (options.use_ortools) {
    std::vector<int> ortools_order = SolveOrtools();
    if (!ortools_order.empty()) {
      if (options.use_two_opt && ortools_order.size() > 2) {
        ortools_order = SolveTwoOpt(ortools_order);
      }
      return ortools_order;
    }
  }

  std::vector<int> order;
  if (n <= options.max_exact_nodes) {
    order = SolveExact();
  } else {
    order = SolveGreedy();
  }
  if (options.use_two_opt && order.size() > 2) {
    order = SolveTwoOpt(order);
  }
  return order;
}

}  // namespace autonomy::perception::exploration
