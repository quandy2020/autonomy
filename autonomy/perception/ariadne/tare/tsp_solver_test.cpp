/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/tsp_solver.hpp"

#include "gtest/gtest.h"

namespace autonomy::perception::exploration {
namespace {

TEST(TspSolverTest, SolvesSmallTour) {
  TspSolver::DataModel model;
  model.depot = 0;
  model.distance_matrix = {
      {0, 10, 15, 20},
      {10, 0, 35, 25},
      {15, 35, 0, 30},
      {20, 25, 30, 0},
  };
  TspSolver solver(std::move(model), 12);
  const std::vector<int> order = solver.Solve();
  ASSERT_EQ(order.size(), 4u);
  EXPECT_EQ(order.front(), 0);
}

TEST(TspSolverTest, GreedyFallbackForLargeGraph) {
  TspSolver::DataModel model;
  model.depot = 0;
  model.distance_matrix.assign(20, std::vector<int>(20, 100));
  for (int i = 0; i < 20; ++i) {
    model.distance_matrix[static_cast<size_t>(i)]
                         [static_cast<size_t>(i)] = 0;
  }
  TspSolver solver(std::move(model), 8);
  const std::vector<int> order = solver.Solve();
  EXPECT_EQ(order.size(), 20u);
  EXPECT_EQ(order.front(), 0);
}

}  // namespace
}  // namespace autonomy::perception::exploration
