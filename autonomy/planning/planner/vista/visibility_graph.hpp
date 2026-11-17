/* Copyright 2026 The Openbot Authors. */
#pragma once

#include <cstdint>
#include <algorithm>
#include <map>
#include <vector>
#include "autonomy/perception/data/map_reader.hpp"

namespace autonomy::planning::planner::vista {
using Pose2d = opt::navigation::Pose2d;
using Bounds2d = perception::Bounds2d;
using NodeId = std::uint64_t;
using EdgeId = std::pair<NodeId, NodeId>;
inline EdgeId EdgeKey(NodeId a, NodeId b) { return {std::min(a,b),std::max(a,b)}; }
struct VisibilityNode { NodeId id = 0; Pose2d pose; std::uint64_t validation_revision = 0; };
struct VisibilityEdge {
    EdgeId id;
    double length = 0, clearance = 0;
    Bounds2d bounds;
    bool valid = false;
    std::uint64_t validation_revision = 0;
};
struct VisibilityGraph {
    std::map<NodeId, VisibilityNode> nodes;
    std::map<EdgeId, VisibilityEdge> edges;
    std::uint64_t map_epoch = 0, geometry_revision = 0, observation_watermark = 0;
    perception::ObservationCut observation_cut{};
};
struct GraphUpdateStats {
    std::size_t new_nodes = 0, removed_nodes = 0, revalidated_edges = 0;
    std::size_t invalidated_edges = 0, reconnected_edges = 0;
    bool candidate_limit_reached = false;
};
}  // namespace autonomy::planning::planner::vista
