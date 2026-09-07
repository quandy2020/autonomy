/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/tsp_solver.hpp"

#ifdef AUTONOMY_HAS_ORTOOLS

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace autonomy::perception::exploration {

std::vector<int> TspSolver::SolveOrtools() const {
  const int n = static_cast<int>(data_.distance_matrix.size());
  if (n <= 1) {
    return n == 1 ? std::vector<int>{0} : std::vector<int>{};
  }

  operations_research::RoutingIndexManager manager(n, 1, data_.depot);
  operations_research::RoutingModel routing(manager);

  const int transit_callback_index = routing.RegisterTransitCallback(
      [this, &manager](int64_t from_index, int64_t to_index) -> int64_t {
        const int from_node =
            manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return data_.distance_matrix[static_cast<size_t>(from_node)]
                                    [static_cast<size_t>(to_node)];
      });
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  operations_research::RoutingSearchParameters search_parameters =
      operations_research::DefaultRoutingSearchParameters();
  search_parameters.set_first_solution_strategy(
      operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  const operations_research::Assignment* solution =
      routing.SolveWithParameters(search_parameters);
  if (solution == nullptr) {
    return {};
  }

  std::vector<int> order;
  int64_t index = routing.Start(0);
  while (!routing.IsEnd(index)) {
    order.push_back(static_cast<int>(manager.IndexToNode(index).value()));
    index = solution->Value(routing.NextVar(index));
  }
  return order;
}

}  // namespace autonomy::perception::exploration

#endif  // AUTONOMY_HAS_ORTOOLS
