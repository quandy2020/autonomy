/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <map>
#include <set>
#include "autonomy/planning/planner/vista/visibility_graph.hpp"

namespace autonomy::planning::planner::vista {
class EdgeSpatialIndex {
public:
    explicit EdgeSpatialIndex(double bucket_size = 2.0);
    void Insert(const VisibilityEdge& edge);
    void Erase(const EdgeId& edge);
    std::set<EdgeId> Query(const Bounds2d& region) const;
    void Clear();
private:
    double bucket_size_;
    std::map<std::pair<int,int>, std::set<EdgeId>> buckets_;
    std::map<EdgeId, Bounds2d> bounds_;
};
}  // namespace autonomy::planning::planner::vista
