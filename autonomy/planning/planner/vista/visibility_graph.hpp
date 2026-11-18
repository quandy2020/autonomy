/* Copyright 2026 The Openbot Authors. */
/** @file visibility_graph.hpp
 * @brief Incremental route graph identities and source-specific evidence provenance.
 */
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
/** @brief Canonical undirected edge identity, independent of endpoint order. */
inline EdgeId EdgeKey(NodeId a, NodeId b) { return {std::min(a,b),std::max(a,b)}; }
/** @brief Persistent obstacle-contour vertex and its latest geometric validation. */
struct VisibilityNode {
    NodeId id = 0;  ///< Stable planner-owned vertex identity.
    Pose2d pose;  ///< World-grid vertex pose.
    std::uint64_t validation_revision = 0;  ///< Geometry revision used by the last validation.
};
/** @brief Retained route edge, including blocked edges needed for future restoration. */
struct VisibilityEdge {
    EdgeId id;  ///< Canonical endpoint pair.
    double length = 0;  ///< Route chord length, meters.
    double clearance = 0;  ///< Validated route-model clearance, meters.
    Bounds2d bounds;  ///< World region used for dirty-region intersection queries.
    bool valid = false;  ///< Current route-model admission; not a motion safety permit.
    std::uint64_t validation_revision = 0;  ///< Last route-model validation revision.
};
/** @brief Planner-owned graph; route connectivity is independent of execution safety. */
struct VisibilityGraph {
    std::map<NodeId, VisibilityNode> nodes;  ///< Stable vertex registry.
    std::map<EdgeId, VisibilityEdge> edges;  ///< Valid and blocked retained connections.
    std::uint64_t map_epoch = 0;  ///< Coordinate-alignment generation.
    std::uint64_t geometry_revision = 0;  ///< Route-model geometry consumed by this graph.
    std::uint64_t observation_watermark = 0;  ///< Producer diagnostic watermark.
    perception::ObservationCut observation_cut{};  ///< Actual source prefix received from the map owner.
    perception::ObservationCut observation_baseline{};  ///< Historical prefix excluded by a verified recovery.
    std::uint64_t observation_recovery_generation = 0;  ///< Baseline forwarded by the map integration owner.
};
/** @brief Per-update work counters; reaching limits does not prove exploration completion. */
struct GraphUpdateStats {
    std::size_t new_nodes = 0;  ///< Newly created contour vertices.
    std::size_t removed_nodes = 0;  ///< Removed obsolete vertices.
    std::size_t revalidated_edges = 0;  ///< Connections checked against current geometry.
    std::size_t invalidated_edges = 0;  ///< Previously valid connections now blocked.
    std::size_t reconnected_edges = 0;  ///< Previously blocked connections now valid.
    bool candidate_limit_reached = false;  ///< Some candidate vertices could not be inserted.
    bool full_refresh = false;  ///< All edges were reconsidered because delta continuity was unavailable.
};
}  // namespace autonomy::planning::planner::vista
