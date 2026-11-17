/* Copyright 2026 The Openbot Authors. */
#include "autonomy/planning/planner/vista/graph_updater.hpp"
#include <algorithm>
#include <set>
#include <stdexcept>

namespace autonomy::planning::planner::vista {
namespace {
using PositionKey = std::pair<long long,long long>;
PositionKey Key(const Pose2d& p) { return {std::llround(p.x*1e6),std::llround(p.y*1e6)}; }
}  // namespace
GraphUpdater::GraphUpdater(std::size_t nodes, double length)
    : maximum_nodes_(nodes), maximum_edge_length_(length) {
    if (!nodes || !std::isfinite(length) || length <= 0)
        throw std::invalid_argument("invalid visibility graph limits");
}
GraphUpdateStats GraphUpdater::Update(const perception::MapReadView& view,
                                      const EdgeValidator& validator, VisibilityGraph& graph) {
    GraphUpdateStats stats;
    bool reset = graph.map_epoch != view.map_epoch;
    if (reset) { graph = {}; index_.Clear(); }
    if (!reset && graph.geometry_revision == view.geometry_revision) {
        graph.observation_watermark = view.observation_watermark;
        graph.observation_cut = view.observation_cut; return stats;
    }
    auto regions = view.dirty_regions;
    const auto& map = *view.grid;
    if (reset) regions = {{map.origin_x(),map.origin_y(),
                            map.origin_x()+map.width()*map.resolution(),
                            map.origin_y()+map.height()*map.resolution()}};
    const double halo = validator.safety().options().robot_radius+
                        validator.safety().options().horizontal_margin+2*map.resolution();
    for (auto& b : regions) { b.min_x -= halo; b.min_y -= halo; b.max_x += halo; b.max_y += halo; }
    const auto candidates = ObstacleContour{}.Extract(view,regions,validator);
    std::map<PositionKey,Pose2d> wanted;
    for (const auto& p : candidates) wanted[Key(p)] = p;
    std::set<NodeId> affected_nodes, new_nodes;
    for (auto it = graph.nodes.begin(); it != graph.nodes.end();) {
        const bool affected = std::any_of(regions.begin(),regions.end(),
            [&](const auto& b){return b.Contains(it->second.pose);});
        if (!affected) { ++it; continue; }
        auto matching = wanted.find(Key(it->second.pose));
        if (matching == wanted.end()) {
            affected_nodes.insert(it->first); it = graph.nodes.erase(it); ++stats.removed_nodes;
        } else {
            it->second.validation_revision = view.geometry_revision;
            wanted.erase(matching); ++it;
        }
    }
    for (const auto& p : wanted) {
        if (graph.nodes.size() >= maximum_nodes_) { stats.candidate_limit_reached = true; break; }
        const NodeId id = next_id_++;
        graph.nodes[id] = {id,p.second,view.geometry_revision}; new_nodes.insert(id); ++stats.new_nodes;
    }
    std::set<EdgeId> affected;
    for (const auto& region : regions) {
        auto edges = index_.Query(region); affected.insert(edges.begin(),edges.end());
    }
    for (auto it = graph.edges.begin(); it != graph.edges.end();) {
        if (affected_nodes.count(it->first.first) || affected_nodes.count(it->first.second)) {
            index_.Erase(it->first); affected.erase(it->first); it = graph.edges.erase(it);
        } else ++it;
    }
    // Includes blocked edges; removing an obstacle must reconsider them.
    for (const auto& id : affected) {
        auto& edge = graph.edges.at(id); bool before = edge.valid;
        edge = validator.Validate(id.first,graph.nodes.at(id.first).pose,
                                  id.second,graph.nodes.at(id.second).pose);
        ++stats.revalidated_edges;
        if (before && !edge.valid) ++stats.invalidated_edges;
        if (!before && edge.valid) ++stats.reconnected_edges;
    }
    for (NodeId id : new_nodes) for (const auto& other : graph.nodes) {
        if (id == other.first) continue;
        const auto key = EdgeKey(id,other.first);
        if (graph.edges.count(key)) continue;
        if (opt::navigation::Distance(graph.nodes.at(id).pose,other.second.pose) > maximum_edge_length_)
            continue;
        auto edge = validator.Validate(id,graph.nodes.at(id).pose,other.first,other.second.pose);
        index_.Insert(edge); graph.edges[key] = edge; ++stats.revalidated_edges;
    }
    graph.map_epoch = view.map_epoch; graph.geometry_revision = view.geometry_revision;
    graph.observation_watermark = view.observation_watermark;
    graph.observation_cut = view.observation_cut;
    return stats;
}
}  // namespace autonomy::planning::planner::vista
