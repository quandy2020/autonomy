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

/**
 * @file kdtree.hpp
 * @brief 3D nearest-neighbor index with a KD-tree-style front end (PCL-free).
 */

#ifndef ARGUS_COMMON_KDTREE_HPP_
#define ARGUS_COMMON_KDTREE_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <vector>

namespace argus {
namespace math {

/**
 * @brief In-memory 3D nearest-neighbor search for small or medium point sets.
 * The public API mirrors a typical KD-tree (`SetInput` / `NearestKSearch`).
 * The current implementation is brute-force `partial_sort` and is intended for
 * BTC / ICP paths where cloud size stays modest.
 */
class KdTree {
 public:
  /**
   * @brief Replace the indexed point set.
   * @param[in] points Points in Cartesian coordinates (metres). Copied
   *                   internally; may be empty.
   */
  void SetInput(const std::vector<Eigen::Vector3d>& points) { pts_ = points; }

  /**
   * @brief Find the @p k nearest neighbors of query @p query.
   * @param[in]  query   Query point in the same frame as @ref SetInput.
   * @param[in]  k       Requested neighbor count; clamped to the cloud size.
   * @param[out] indices Receives neighbor indices into the input cloud.
   *                     Cleared then filled. Must be non-null.
   * @param[out] squared_distances Receives squared Euclidean distances
   *                     aligned with @p indices. Must be non-null.
   * @return Number of neighbors written (in `[0, k]`). Returns `0` if either
   *         output pointer is null, the cloud is empty, or @p k ≤ 0.
   */
  int NearestKSearch(const Eigen::Vector3d& query, int k,
                     std::vector<int>* indices,
                     std::vector<float>* squared_distances) const {
    if (indices == nullptr || squared_distances == nullptr || pts_.empty() ||
        k <= 0) {
      return 0;
    }
    indices->clear();
    squared_distances->clear();
    struct Hit {
      float d2;
      int i;
    };
    std::vector<Hit> hits;
    hits.reserve(pts_.size());
    for (int i = 0; i < static_cast<int>(pts_.size()); ++i) {
      hits.push_back({static_cast<float>(
                          (pts_[static_cast<size_t>(i)] - query).squaredNorm()),
                      i});
    }
    const int n = std::min(k, static_cast<int>(hits.size()));
    std::partial_sort(hits.begin(), hits.begin() + n, hits.end(),
                      [](const Hit& a, const Hit& b) { return a.d2 < b.d2; });
    for (int i = 0; i < n; ++i) {
      indices->push_back(hits[static_cast<size_t>(i)].i);
      squared_distances->push_back(hits[static_cast<size_t>(i)].d2);
    }
    return n;
  }

 private:
  std::vector<Eigen::Vector3d> pts_;
};

}  // namespace math
}  // namespace argus

#endif  // ARGUS_COMMON_KDTREE_HPP_
