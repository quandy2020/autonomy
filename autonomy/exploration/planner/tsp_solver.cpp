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

#include "autonomy/exploration/planner/tsp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace autonomy {
namespace exploration {

TspSolver::TspSolver(TspDataModel data, int max_exact_n)
    : data_(std::move(data)), max_exact_n_(std::max(2, max_exact_n))
{
}

int TspSolver::MetersToCost(double path_length_m)
{
    if (!std::isfinite(path_length_m) || path_length_m < 0.0) {
        return kInfCost / 2;
    }
    return static_cast<int>(std::lround(10.0 * path_length_m));
}

int TspSolver::ArcCost(int from, int to) const
{
    if (from < 0 || to < 0 ||
        from >= static_cast<int>(data_.distance_matrix.size()) ||
        to >= static_cast<int>(data_.distance_matrix[static_cast<size_t>(from)]
                                   .size())) {
        return kInfCost;
    }
    return data_.distance_matrix[static_cast<size_t>(from)]
                                [static_cast<size_t>(to)];
}

int TspSolver::TourCost() const
{
    if (tour_.size() < 2) {
        return 0;
    }
    int cost = 0;
    for (size_t i = 1; i < tour_.size(); ++i) {
        const int c = ArcCost(tour_[i - 1], tour_[i]);
        if (c >= kInfCost / 2) {
            return kInfCost;
        }
        cost += c;
    }
    return cost;
}

void TspSolver::SolveHeldKarp()
{
    const int n = static_cast<int>(data_.distance_matrix.size());
    const int depot = std::clamp(data_.depot, 0, n - 1);
    const int full = 1 << n;

    // dp[mask][j] = min cost of path visiting mask ending at j (depot always in)
    std::vector<std::vector<int>> dp(static_cast<size_t>(full),
                                     std::vector<int>(static_cast<size_t>(n),
                                                      kInfCost));
    std::vector<std::vector<int>> parent(
        static_cast<size_t>(full),
        std::vector<int>(static_cast<size_t>(n), -1));

    dp[static_cast<size_t>(1 << depot)][static_cast<size_t>(depot)] = 0;

    for (int mask = 0; mask < full; ++mask) {
        if ((mask & (1 << depot)) == 0) {
            continue;
        }
        for (int j = 0; j < n; ++j) {
            if ((mask & (1 << j)) == 0) {
                continue;
            }
            const int cur = dp[static_cast<size_t>(mask)][static_cast<size_t>(j)];
            if (cur >= kInfCost) {
                continue;
            }
            for (int k = 0; k < n; ++k) {
                if ((mask & (1 << k)) != 0) {
                    continue;
                }
                const int nd = cur + ArcCost(j, k);
                const int nmask = mask | (1 << k);
                if (nd < dp[static_cast<size_t>(nmask)][static_cast<size_t>(k)]) {
                    dp[static_cast<size_t>(nmask)][static_cast<size_t>(k)] = nd;
                    parent[static_cast<size_t>(nmask)][static_cast<size_t>(k)] = j;
                }
            }
        }
    }

    // Open tour: end at any node (do not force return to depot).
    int best_end = depot;
    int best_cost = 0;
    const int all = full - 1;
    best_cost = kInfCost;
    for (int j = 0; j < n; ++j) {
        const int c = dp[static_cast<size_t>(all)][static_cast<size_t>(j)];
        if (c < best_cost) {
            best_cost = c;
            best_end = j;
        }
    }

    tour_.clear();
    if (best_cost >= kInfCost) {
        tour_.push_back(depot);
        path_length_ = 0.0;
        return;
    }

    int mask = all;
    int cur = best_end;
    std::vector<int> rev;
    while (cur >= 0) {
        rev.push_back(cur);
        const int p = parent[static_cast<size_t>(mask)][static_cast<size_t>(cur)];
        mask ^= (1 << cur);
        cur = p;
        if (mask == 0) {
            break;
        }
    }
    std::reverse(rev.begin(), rev.end());
    tour_ = std::move(rev);
    path_length_ = static_cast<double>(best_cost) / 10.0;
}

void TspSolver::SolvePathCheapestArc()
{
    const int n = static_cast<int>(data_.distance_matrix.size());
    const int depot = std::clamp(data_.depot, 0, std::max(0, n - 1));
    tour_.clear();
    if (n <= 0) {
        path_length_ = 0.0;
        return;
    }

    std::vector<bool> used(static_cast<size_t>(n), false);
    tour_.push_back(depot);
    used[static_cast<size_t>(depot)] = true;

    while (static_cast<int>(tour_.size()) < n) {
        const int last = tour_.back();
        int best = -1;
        int best_cost = kInfCost;
        for (int j = 0; j < n; ++j) {
            if (used[static_cast<size_t>(j)]) {
                continue;
            }
            const int c = ArcCost(last, j);
            if (c < best_cost) {
                best_cost = c;
                best = j;
            }
        }
        if (best < 0) {
            break;
        }
        used[static_cast<size_t>(best)] = true;
        tour_.push_back(best);
    }
    path_length_ = static_cast<double>(TourCost()) / 10.0;
}

void TspSolver::Improve2Opt()
{
    if (tour_.size() < 4) {
        return;
    }
    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 1; i + 2 < tour_.size(); ++i) {
            for (size_t k = i + 1; k + 1 < tour_.size(); ++k) {
                const int a = tour_[i - 1];
                const int b = tour_[i];
                const int c = tour_[k];
                const int d = tour_[k + 1];
                const int before = ArcCost(a, b) + ArcCost(c, d);
                const int after = ArcCost(a, c) + ArcCost(b, d);
                if (after < before) {
                    std::reverse(tour_.begin() + static_cast<std::ptrdiff_t>(i),
                                 tour_.begin() + static_cast<std::ptrdiff_t>(k) +
                                     1);
                    improved = true;
                }
            }
        }
    }
    path_length_ = static_cast<double>(TourCost()) / 10.0;
}

void TspSolver::Solve()
{
    const int n = static_cast<int>(data_.distance_matrix.size());
    if (n <= 0) {
        tour_.clear();
        path_length_ = 0.0;
        return;
    }
    if (n == 1) {
        tour_ = {std::clamp(data_.depot, 0, 0)};
        path_length_ = 0.0;
        return;
    }
    if (n <= max_exact_n_) {
        SolveHeldKarp();
    } else {
        SolvePathCheapestArc();
        Improve2Opt();
    }
}

void TspSolver::GetSolutionNodeIndex(std::vector<int>* node_index,
                                     bool has_dummy) const
{
    if (!node_index) {
        return;
    }
    node_index->clear();
    if (tour_.empty()) {
        return;
    }
    *node_index = tour_;

    if (!has_dummy || node_index->size() < 2) {
        return;
    }

    // Mirror TARE TSPSolver::getSolutionNodeIndex(has_dummy).
    const int dummy_node_index =
        static_cast<int>(data_.distance_matrix.size()) - 1;
    if ((*node_index)[1] == dummy_node_index) {
        node_index->erase(node_index->begin() + 1);
        node_index->push_back((*node_index)[0]);
        node_index->erase(node_index->begin());
        std::reverse(node_index->begin(), node_index->end());
    } else if (node_index->back() == dummy_node_index) {
        node_index->pop_back();
    } else {
        // Dummy may sit elsewhere; strip all dummy occurrences.
        node_index->erase(
            std::remove(node_index->begin(), node_index->end(), dummy_node_index),
            node_index->end());
    }
}

}  // namespace exploration
}  // namespace autonomy
