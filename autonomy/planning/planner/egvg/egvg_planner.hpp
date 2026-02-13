/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>
#include <vector>

#include "autonomy/planning/planner/egvg/gvg.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace egvg {

// A* / 拓扑规划器，基于 GVG 图进行路径搜索。
class Planner
{
public:
    Planner();
    ~Planner();

    void init(std::shared_ptr<GVG> gvg);

    void reset();

    bool serachPath(GraphNode::Ptr start_node, GraphNode::Ptr goal_node);

    void retrievePath(GraphNode::Ptr end_node);

    std::vector<IntPoint> getPath();

    std::vector<IntPoint> getFullPath();

    std::vector<IntPoint> getFullPathNodes();

    std::vector<IntPoint> getVisitedNodes();

    std::vector<IntPoint> AstarOnVoronoi(const IntPoint& pt1, const IntPoint& pt2, int sizeX, int sizeY, int graph_id);

    std::vector<std::vector<IntPoint>> expand_voronoi_grid(const IntPoint& voronoi_grid,
                                                           const std::vector<IntPoint> strong_nodes, const int& sizeX,
                                                           const int& sizeY);

    void DFSSearch(std::vector<GraphNode::Ptr>& vis, GraphNode::Ptr goal, int max_path_num);

    std::vector<std::vector<IntPoint>> searchTopoPaths(GraphNode::Ptr start_node, GraphNode::Ptr goal_node,
                                                       int max_path_num);

private:
    // 和A*规划相关的
    std::vector<GraphNode::Ptr> path_node_pool_;
    int use_node_num_ = 0;
    int iter_num_ = 0;
    NodeHashTable0 expanded_nodes_;
    std::priority_queue<GraphNode::Ptr, std::vector<GraphNode::Ptr>, NodeComparator0> open_set_;
    std::vector<GraphNode::Ptr> path_nodes_;
    double lambda_heu_ = 1.0;
    int allocate_num_ = 10000;
    double tie_breaker_ = 1.0 + 1.0 / 10000;
    std::shared_ptr<GVG> gvg_;

    // 和 topo 规划相关的
    std::vector<std::vector<GraphNode::Ptr>> raw_topo_paths_;

    /* heuristic function */
    double getDiagHeu(const IntPoint& p1, const IntPoint& p2);

    double getManhHeu(const IntPoint& p1, const IntPoint& p2);

    double getEuclHeu(const IntPoint& p1, const IntPoint& p2);
};

}  // namespace egvg
}  // namespace planner
}  // namespace planning
}  // namespace autonomy
