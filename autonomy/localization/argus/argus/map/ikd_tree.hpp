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
 * @file ikd_tree.hpp
 * @brief Incremental k-d tree for LiDAR nearest-neighbor (FAST-LIO style API).
 *
 * Not a line-by-line port of HKU ikd-Tree; provides Build / AddPoints /
 * NearestSearch with periodic rebuild for balance.
 */

#ifndef ARGUS_MAP_IKD_TREE_HPP_
#define ARGUS_MAP_IKD_TREE_HPP_

#include <cstddef>
#include <mutex>
#include <vector>

#include <Eigen/Core>

#include "argus/map/ikd_tree_config.hpp"
#include "argus/map/map_base.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace map {

/**
 * @brief Incremental k-d tree for LiDAR nearest-neighbor queries.
 */
class IkdTree : public MapBase
{
public:
    /**
     * @brief Construct with leaf / rebuild settings.
     * @param config Tree parameters.
     */
    explicit IkdTree(IkdTreeConfig config = {});
    ~IkdTree() override;

    IkdTree(const IkdTree&) = delete;
    IkdTree& operator=(const IkdTree&) = delete;

    /** @brief Remove all points and free nodes. */
    void Clear() override;
    /** @return Number of stored points. */
    std::size_t size() const override;
    /** @return True if the tree holds no points. */
    bool empty() const override {
        return size() == 0;
    }

    /**
     * @brief Rebuild the tree from a full cloud.
     * @param cloud Input points (world or body; caller convention).
     */
    void Build(const sensor::PointCloudXYZI& cloud);

    /**
     * @brief Incremental insert; may trigger rebuild.
     * @param cloud Points to insert (optionally downsampled).
     */
    void AddPoints(const sensor::PointCloudXYZI& cloud);

    /**
     * @brief k-NN search.
     * @param query Query position.
     * @param k Maximum neighbors to return.
     * @param[out] points Neighbor positions (cleared then filled).
     * @param[out] dists_sq Squared distances (same order as points).
     * @return Number of neighbors written (≤ k).
     */
    int NearestSearch(const Eigen::Vector3d& query, int k,
                      std::vector<Eigen::Vector3d>* points,
                      std::vector<float>* dists_sq) const;

    /**
     * @brief Radius search.
     * @param query Query position.
     * @param radius Search radius (m); implementation uses squared radius.
     * @param[out] points Neighbor positions.
     * @param[out] dists_sq Squared distances.
     * @return Number of neighbors written.
     */
    int RadiusSearch(const Eigen::Vector3d& query, float radius,
                     std::vector<Eigen::Vector3d>* points,
                     std::vector<float>* dists_sq) const;

    /**
     * @brief Flatten all stored points (for map serialization).
     * @param[out] points Destination buffer (cleared then filled).
     */
    void ExportPoints(std::vector<Eigen::Vector3d>* points) const;

    const IkdTreeConfig& config() const {
        return config_;
    }

private:
    struct Node {
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        int axis = 0;
        Node* left = nullptr;
        Node* right = nullptr;
    };

    struct HeapItem {
        float dist_sq = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        bool operator<(const HeapItem& o) const {
            return dist_sq < o.dist_sq;
        }  // max-heap via priority_queue
    };

    void Destroy(Node* n);
    Node* BuildRecursive(std::vector<Eigen::Vector3d>* pts, int begin, int end,
                         int depth);
    void NearestRecursive(const Node* n, const Eigen::Vector3d& q, int k,
                          std::vector<HeapItem>* heap) const;
    void RadiusRecursive(const Node* n, const Eigen::Vector3d& q, float r2,
                         std::vector<HeapItem>* out) const;
    void Flatten(const Node* n, std::vector<Eigen::Vector3d>* out) const;
    static Eigen::Vector3d ToVec(const sensor::PointType& p);

    IkdTreeConfig config_;
    Node* root_ = nullptr;
    std::size_t size_ = 0;
    std::size_t size_at_build_ = 0;
    mutable std::mutex mutex_;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_IKD_TREE_HPP_
