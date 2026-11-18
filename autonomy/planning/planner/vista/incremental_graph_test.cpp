/* Copyright 2026 The Openbot Authors. */
/** @file incremental_graph_test.cpp
 * @brief Regression scenarios for skipped map deltas and bounded spatial queries.
 */
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "autonomy/planning/planner/vista/route_planner.hpp"

namespace v = autonomy::planning::planner::vista;
namespace p = autonomy::perception;
namespace n = opt::navigation;
namespace {
/** @brief Keep assertions active in release builds. */
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
/** @brief Invalid input must explicitly reject before replacing accepted state. */
template <typename Operation> void Reject(Operation operation) {
    bool rejected = false;
    try { operation(); } catch (const std::exception&) { rejected = true; }
    Require(rejected, "invalid spatial input was accepted");
}
/** @brief Two obstacles expose retained long edges away from both endpoints. */
n::GridMap Scene() {
    n::GridMap map(45, 30, .2);
    map.Fill({n::CellState::kFree, 0});
    for (int y = 8; y <= 11; ++y) for (int x = 8; x <= 11; ++x)
        map.Set(x, y, {n::CellState::kOccupied, 0});
    for (int y = 18; y <= 21; ++y) for (int x = 30; x <= 33; ++x)
        map.Set(x, y, {n::CellState::kOccupied, 0});
    return map;
}
/** @brief Locate a genuinely admitted chord which a new midpoint obstacle blocks. */
n::CellIndex CrossingCell(const v::VisibilityGraph& graph, const n::GridMap& map) {
    for (const auto& entry : graph.edges) {
        if (!entry.second.valid || entry.second.length < 4) continue;
        const auto& a = graph.nodes.at(entry.first.first).pose;
        const auto& b = graph.nodes.at(entry.first.second).pose;
        const auto cell = map.WorldToCell({(a.x + b.x) / 2, (a.y + b.y) / 2, 0});
        if (cell && cell->x > 5 && cell->x < 39 && cell->y > 4 && cell->y < 25 &&
            map.At(cell->x, cell->y).state == n::CellState::kFree) return *cell;
    }
    throw std::runtime_error("scene supplied no interior crossing edge");
}
/** @brief Independently validate every retained valid edge against the current map. */
void CheckCurrentEdges(const v::VisibilityGraph& graph, const p::MapReadView& map,
                       const n::SafetyOptions& options) {
    n::GeometrySafety safety(map.grid, options);
    for (const auto& entry : graph.edges) if (entry.second.valid) {
        const auto checked = safety.VerifySegment(graph.nodes.at(entry.first.first).pose,
                                                  graph.nodes.at(entry.first.second).pose);
        if (!checked.safe) throw std::runtime_error(
            "retained visibility edge " + std::to_string(entry.first.first) + ":" +
            std::to_string(entry.first.second) + " revision=" +
            std::to_string(entry.second.validation_revision) + " graph=" +
            std::to_string(graph.geometry_revision) + " view=" +
            std::to_string(map.geometry_revision) + " " + checked.reason + " reversed_safe=" +
            std::to_string(safety.VerifySegment(graph.nodes.at(entry.first.second).pose,
                                                graph.nodes.at(entry.first.first).pose).safe));
        Require(safety.VerifySegment(graph.nodes.at(entry.first.second).pose,
                                     graph.nodes.at(entry.first.first).pose).safe,
                "undirected edge did not certify the reverse traversal");
    }
}
/** @brief Skipping a producer revision cannot make its obstacle disappear from graph validation. */
void MissedDelta(bool missing_regions) {
    auto map = Scene(); p::MapReader reader;
    n::SafetyOptions options; options.robot_radius = .05; options.horizontal_margin = 0;
    v::VistaRoutePlanner planner(options, 150);
    const auto initial = reader.Read(map, 1, 0, 1); planner.Update(initial);
    const auto before = planner.GraphSnapshot();
    const auto crossing = CrossingCell(before, map);
    map.Set(crossing.x, crossing.y, {n::CellState::kOccupied, 0});
    auto latest = reader.Read(map, 2, 0, 2);
    if (missing_regions) {
        latest.dirty_regions.clear();
    } else {
        // Revision 2 is intentionally not delivered. Revision 3 reports only
        // a distant change, while the immutable grid still includes the obstacle.
        map.Set(42, 2, {n::CellState::kOccupied, 0});
        latest = reader.Read(map, 3, 0, 3);
    }
    const auto stats = planner.Update(latest);
    Require(stats.full_refresh, "lost difference continuity did not trigger full refresh");
    const auto graph = planner.GraphSnapshot();
    CheckCurrentEdges(graph, latest, options);
    std::size_t retained = 0;
    for (const auto& entry : before.nodes) retained += graph.nodes.count(entry.first);
    Require(retained > 0, "full refresh unnecessarily replaced every contour identity");
    planner.Update(initial);
    Require(planner.GraphSnapshot().geometry_revision == latest.geometry_revision,
            "late producer revision rolled back the graph");
}
/** @brief Consecutive deltas retain locality, blocked edges reconnect, and repeats only forward evidence. */
void ConsecutiveDelta() {
    auto map = Scene(); p::MapReader reader;
    n::SafetyOptions options; options.robot_radius = .05; options.horizontal_margin = 0;
    v::VistaRoutePlanner planner(options, 150);
    const auto initial = reader.Read(map, 1, 0, 1); planner.Update(initial);
    const auto before = planner.GraphSnapshot();
    const auto crossing = CrossingCell(before, map);
    map.Set(crossing.x, crossing.y, {n::CellState::kOccupied, 0});
    const auto blocked = reader.Read(map, 2, 0, 2);
    const auto changed = planner.Update(blocked);
    Require(!changed.full_refresh && changed.invalidated_edges > 0,
            "consecutive bounded difference lost local edge invalidation");
    Require(changed.revalidated_edges < before.edges.size(),
            "local update needlessly validated the entire original graph");
    CheckCurrentEdges(planner.GraphSnapshot(), blocked, options);
    map.Set(crossing.x, crossing.y, {n::CellState::kFree, 0});
    const auto restored = reader.Read(map, 3, 0, 3);
    Require(planner.Update(restored).reconnected_edges > 0,
            "removing an obstacle did not reconsider blocked edges");
    CheckCurrentEdges(planner.GraphSnapshot(), restored, options);
    const auto repeated = reader.Read(map, 4, 0, 4);
    const auto unchanged = planner.Update(repeated);
    Require(!unchanged.full_refresh && unchanged.revalidated_edges == 0 &&
            planner.GraphSnapshot().observation_watermark == 4,
            "unchanged geometry did not preserve the evidence-only update path");
}
/** @brief A changing same-epoch window removes vanished vertices and keeps surviving identities. */
void ChangingExtent() {
    auto map = Scene(); p::MapReader reader;
    n::SafetyOptions options; options.robot_radius = .05; options.horizontal_margin = 0;
    v::VistaRoutePlanner planner(options, 150);
    auto view = reader.Read(map, 1, 0, 1); planner.Update(view);
    const auto original = planner.GraphSnapshot();
    auto smaller = std::make_shared<n::GridMap>(20, 30, .2);
    for (int y = 0; y < smaller->height(); ++y)
        for (int x = 0; x < smaller->width(); ++x) smaller->Set(x, y, map.At(x, y));
    view.grid = smaller; ++view.geometry_revision; view.dirty_regions.clear();
    Require(planner.Update(view).full_refresh, "window shrink did not force full geometry refresh");
    const auto shrunk = planner.GraphSnapshot();
    std::size_t retained = 0;
    for (const auto& entry : shrunk.nodes) {
        Require(smaller->WorldToCell(entry.second.pose).has_value(), "vanished-window vertex survived");
        retained += original.nodes.count(entry.first);
    }
    Require(retained > 0, "window shrink reset surviving contour identities");
    CheckCurrentEdges(shrunk, view, options);
}
/** @brief A rejected large replacement must preserve the previously indexed edge. */
void RejectedIndexReplacement() {
    v::EdgeSpatialIndex index;
    v::VisibilityEdge edge; edge.id = {1, 2}; edge.bounds = {.1, .1, .5, .5};
    index.Insert(edge);
    auto invalid = edge; invalid.bounds = {0, 0, 10000, 10000};
    Reject([&] { index.Insert(invalid); });
    Require(index.Query(edge.bounds).count(edge.id) == 1,
            "rejected replacement removed the accepted edge");
}
/** @brief Large queries remain exact without iterating an unbounded rectangle of empty buckets. */
void SpatialNumerics() {
    v::EdgeSpatialIndex index;
    v::VisibilityEdge edge; edge.id = {1, 2}; edge.bounds = {-4, -3, -2, -1};
    index.Insert(edge);
    Require(index.Query({-2, -1, 0, 0}).count(edge.id) == 1, "closed boundary lost a negative-coordinate edge");
    Require(index.Query({-1e100, -1e100, 1e100, 1e100}).count(edge.id) == 1,
            "wide query did not preserve exact intersections");
    auto invalid = edge; invalid.bounds.min_x = std::numeric_limits<double>::quiet_NaN();
    Reject([&] { index.Insert(invalid); });
    Reject([&] { index.Query({1, 0, -1, 1}); });
    invalid.bounds = {1e100, 1e100, 1e100, 1e100};
    Reject([&] { index.Insert(invalid); });
    Require(index.Query(edge.bounds).count(edge.id) == 1, "numerical rejection corrupted accepted index");
    edge.bounds = {1, 1, 2, 2}; index.Insert(edge);
    Require(index.Query({-4, -3, -2, -1}).empty(), "edge replacement retained old bucket references");
    index.Erase(edge.id); Require(index.Query({1, 1, 2, 2}).empty(), "erased edge survived query");
}
/** @brief Malformed deltas are rejected before changing a usable graph. */
void DeltaPreflight() {
    auto map = Scene(); p::MapReader reader;
    n::SafetyOptions options; options.robot_radius = .05; options.horizontal_margin = 0;
    v::VistaRoutePlanner planner(options, 150);
    const auto initial = reader.Read(map, 1, 0, 1); planner.Update(initial);
    auto invalid = initial; ++invalid.geometry_revision;
    invalid.dirty_regions = {{1, 1, std::numeric_limits<double>::infinity(), 2}};
    Reject([&] { planner.Update(invalid); });
    Require(planner.GraphSnapshot().geometry_revision == initial.geometry_revision,
            "malformed delta changed graph provenance");
    invalid = initial; invalid.frame_id = "other_map";
    Reject([&] { planner.Update(invalid); });
    invalid = initial; invalid.footprint_inflated = !initial.footprint_inflated;
    Reject([&] { planner.Update(invalid); });
    invalid = initial;
    invalid.grid = std::make_shared<n::GridMap>(45, 30, .25);
    Reject([&] { planner.Update(invalid); });
    invalid = initial;
    invalid.grid = std::make_shared<n::GridMap>(45, 30, .2, .01, 0);
    Reject([&] { planner.Update(invalid); });
    const auto graph = planner.GraphSnapshot();
    Require(graph.nodes.size() > 0 && graph.geometry_revision == initial.geometry_revision,
            "rejected geometry contract destroyed accepted graph");
    CheckCurrentEdges(graph, initial, options);
    auto next_epoch = initial; ++next_epoch.map_epoch;
    next_epoch.footprint_inflated = true;
    Require(planner.Update(next_epoch).full_refresh &&
            planner.GraphSnapshot().map_epoch == next_epoch.map_epoch,
            "new epoch did not permit a legitimate footprint contract change");
    planner.Update(initial);
    Require(planner.GraphSnapshot().map_epoch == next_epoch.map_epoch,
            "late old-epoch view rolled back footprint admission");
}
}  // namespace

/** @brief Run portable production algorithms; baseline selects only bounded known failures. */
int main(int argc, char** argv) {
    const bool baseline = argc > 1 && std::string(argv[1]) == "--baseline";
    int failures = 0;
    std::vector<std::pair<const char*, std::function<void()>>> tests{
        {"skipped_delta", [] { MissedDelta(false); }},
        {"missing_dirty_regions", [] { MissedDelta(true); }},
        {"rejected_index_replacement", RejectedIndexReplacement}};
    if (!baseline) {
        tests.emplace_back("spatial_numerics", SpatialNumerics);
        tests.emplace_back("delta_preflight", DeltaPreflight);
        tests.emplace_back("consecutive_delta", ConsecutiveDelta);
        tests.emplace_back("changing_extent", ChangingExtent);
    }
    for (const auto& test : tests) {
        try { test.second(); std::cout << "PASS " << test.first << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << test.first << ": " << error.what() << '\n'; }
    }
    return failures ? 1 : 0;
}
