/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <limits>
#include <vector>

namespace autonomy {
namespace exploration {

/**
 * @file tsp_solver.hpp
 * @brief In-process TSP solver (OR-Tools-compatible usage, no or-tools link).
 */

/**
 * @brief Distance-matrix TSP input (TARE tsp_solver DataModel shape).
 */
struct TspDataModel {
    std::vector<std::vector<int>> distance_matrix;  //!< @brief arc costs
    int depot{0};  //!< @brief start / depot node index
};

/**
 * @class TspSolver
 * @brief Solves asymmetric/symmetric TSP on an integer distance matrix.
 *
 * For n ≤ max_exact_n uses Held–Karp DP; otherwise PATH_CHEAPEST_ARC + 2-opt.
 * Dummy-node open-tour extraction mirrors TARE getSolutionNodeIndex(has_dummy).
 */
class TspSolver
{
public:
    /**
     * @brief Construct with distance model.
     * @param data Distance matrix + depot
     * @param max_exact_n Held–Karp threshold (default 12)
     */
    explicit TspSolver(TspDataModel data, int max_exact_n = 12);

    /**
     * @brief Solve the TSP (writes tour_ and path_length_).
     */
    void Solve();

    /**
     * @brief Extract node indices along the solved tour.
     * @param node_index Output node indices
     * @param has_dummy Whether the last matrix row/col is a dummy open-path node
     */
    void GetSolutionNodeIndex(std::vector<int>* node_index,
                              bool has_dummy) const;

    /**
     * @brief Path length in meters (matrix costs are 10× meters).
     * @return Tour length [m]
     */
    double GetPathLength() const { return path_length_; }

    /**
     * @brief Convert meters to TARE-style integer arc cost.
     * @param path_length_m Path length [m]
     * @return Integer cost
     */
    static int MetersToCost(double path_length_m);

private:
    /**
     * @brief Exact Held–Karp DP for small n.
     */
    void SolveHeldKarp();

    /**
     * @brief PATH_CHEAPEST_ARC constructive heuristic.
     */
    void SolvePathCheapestArc();

    /**
     * @brief 2-opt local improvement on tour_ (excluding optional close).
     */
    void Improve2Opt();

    /**
     * @brief Cost of directed arc i → j (large if invalid).
     * @param from From node
     * @param to To node
     * @return Arc cost
     */
    int ArcCost(int from, int to) const;

    /**
     * @brief Sum of consecutive arc costs along tour_ (no return to depot).
     * @return Path cost in matrix units
     */
    int TourCost() const;

    TspDataModel data_;
    int max_exact_n_{12};
    std::vector<int> tour_;  //!< @brief visit order including depot, not closed
    double path_length_{0.0};
    static constexpr int kInfCost = std::numeric_limits<int>::max() / 4;
};

}  // namespace exploration
}  // namespace autonomy
