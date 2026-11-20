/* Copyright 2026 The Openbot Authors. */
/** @file route_planner.hpp
 * @brief Serialized incremental visibility routing over immutable map read views.
 */
#pragma once
#include <functional>
#include <mutex>
#include "autonomy/planning/planner/vista/graph_updater.hpp"
#include "autonomy/common/navigation/reachability.hpp"

namespace autonomy::planning::planner::vista {
/** @brief Small immutable certificate copied from the graph owner under its lock.
 * @details Geometry revision is the graph's actual consumed version. Neither a
 * query nor wall-clock time creates observation or mapper-recovery evidence.
 */
struct VistaMapEvidence {
    std::uint64_t map_epoch = 0;  ///< Coordinate generation of the graph.
    std::uint64_t geometry_revision = 0;  ///< Geometry consumed by the graph.
    std::uint64_t watermark = 0;  ///< Diagnostic upstream watermark.
    perception::ObservationCut observation_cut{};  ///< Actual incorporated source prefixes.
    perception::ObservationCut observation_baseline{};  ///< Excluded recovered history.
    std::uint64_t observation_recovery_generation = 0;  ///< Upstream recovery identity.
    std::string frame_id;  ///< Explicit frame; never used to encode hidden metadata.
};
/** @brief Route result and its source geometry; execution safety is a separate authority. */
struct VistaRoute {
    autonomy::navigation::Path path;  ///< Complete world-frame route.
    double length = 0;  ///< Sum of route chord lengths, meters.
    std::uint64_t map_epoch = 0;  ///< Coordinate generation used by this result.
    std::uint64_t geometry_revision = 0;  ///< Consumed map geometry revision.
    bool success = false;  ///< A checked route connects start and goal.
    bool used_grid_fallback = false;  ///< Sparse graph required independent grid routing.
    bool canceled = false;  ///< Caller requested cancellation before completion.
    bool execution_verified = false;  ///< Never asserted by this planar route planner.
    std::optional<VistaMapEvidence> evidence;  ///< Certificate from the same snapshot as this route.
};
/** @brief Own one persistent graph and prevent routes from observing partial updates. */
class VistaRoutePlanner {
public:
    /** @brief Configure footprint checks and the contour vertex budget. */
    explicit VistaRoutePlanner(autonomy::navigation::SafetyOptions options = {},
                                std::size_t maximum_nodes = 4096);
    /** @brief Apply consecutive deltas, or fully refresh after a missing revision.
     * @throws std::invalid_argument The view violates geometry or epoch contracts.
     * @details Malformed views preserve accepted state. An exception during graph
     * mutation clears the graph and suspends routing until a successful refresh.
     */
    GraphUpdateStats Update(const perception::MapReadView& map);
    /** @brief Find a checked route with cancelable graph search and grid fallback. */
    VistaRoute CreateRoute(const Pose2d& start, const Pose2d& goal,
                            const std::function<bool()>& canceled = {}) const;
    /** @brief Return an owned graph copy serialized against updates. */
    VisibilityGraph GraphSnapshot() const;
    /** @brief Read provenance without copying graph vertices and edges.
     * @return No evidence when an update failed or no map has been admitted.
     */
    std::optional<VistaMapEvidence> Evidence() const;
private:
    mutable std::mutex mutex_;  ///< Protect accepted map, graph and updater state.
    autonomy::navigation::SafetyOptions options_;  ///< Configured robot geometry.
    GraphUpdater updater_;  ///< Persistent contour and edge spatial update owner.
    VisibilityGraph graph_;  ///< Current graph, inaccessible for routing during mutations.
    std::optional<perception::MapReadView> map_;  ///< Last accepted view and epoch contract.
    bool ready_ = false;  ///< False after an update failure; old geometry cannot authorize routing.
};
}  // namespace autonomy::planning::planner::vista
