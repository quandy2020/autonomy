/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <vector>

namespace autonomy::perception::exploration {

// OR-Tools-free TSP on a precomputed distance matrix.
class TspSolver {
 public:
  static constexpr int kInfCost = 1'000'000'000;

  struct DataModel {
    std::vector<std::vector<int>> distance_matrix;
    int depot{0};
  };

  struct SolveOptions {
    int max_exact_nodes{12};
    bool use_two_opt{true};
    bool use_ortools{false};
  };

  explicit TspSolver(DataModel data, int max_exact_nodes = 12);

  std::vector<int> Solve() const { return Solve(SolveOptions{}); }
  std::vector<int> Solve(const SolveOptions& options) const;

 private:
  std::vector<int> SolveExact() const;
  std::vector<int> SolveGreedy() const;
  std::vector<int> SolveTwoOpt(const std::vector<int>& order) const;
  std::vector<int> SolveOrtools() const;
  int ArcCost(int from, int to) const;

  DataModel data_;
  int max_exact_nodes_;
};

}  // namespace autonomy::perception::exploration
