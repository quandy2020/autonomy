/* Copyright 2026 The Openbot Authors. */
/**
 * @file vista_planner.hpp
 * @brief Native global planner backed by a shared incremental visibility graph.
 * @details Global route success does not certify three-dimensional execution.
 */
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/planner/vista/native_costmap_reader.hpp"
#include "autonomy/planning/planner/vista/route_planner.hpp"

namespace autonomy::planning::planner::vista {

/** @brief Geometry and capacity settings for one native Vista instance. */
struct VistaPlannerSettings {
    opt::navigation::SafetyOptions safety;  ///< Platform clearance and floor limits, in metres.
    std::size_t maximum_nodes = 4096;       ///< Maximum persistent visibility nodes.
    std::size_t maximum_cells = 2000000;    ///< Maximum cells in accumulated read history.
    bool costmap_footprint_inflated = false; ///< Input already accounts for the robot footprint.
};

/**
 * @brief Convert native Protobuf settings without losing explicit zero margins.
 * @param options Optional scalar settings; omitted fields use documented defaults.
 * @return Validated settings; robot radius is subsequently derived from Costmap.
 * @throws std::invalid_argument Invalid clearance or zero capacity.
 * @throws std::length_error Capacity cannot be represented by this process.
 */
VistaPlannerSettings ReadVistaPlannerSettings(const proto::VistaPlannerOptions& options);

/**
 * @brief GlobalPlanner implementation owning one shared route graph.
 * @details Public accessors are synchronized. External callbacks run outside the
 *          state mutex and stale requests cannot commit a path after reconfiguration.
 */
class VistaPlanner final : public common::GlobalPlanner {
public:
    /** @brief Provider returning an owned, immutable map view or no available map. */
    using MapViewSource = std::function<std::optional<perception::MapReadView>()>;
    /** @brief Construct an unconfigured plugin for the native plugin manager. */
    VistaPlanner() = default;
    /**
     * @brief Construct from the server's native configuration and shared Costmap.
     * @param options Includes the Vista settings and native planner configuration.
     * @param name Plugin instance id, for example vista_planner.
     * @param costmap Native owner supplying footprint and synchronized geometry.
     */
    VistaPlanner(const proto::PlannerOptions& options, const std::string& name,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap);
    /**
     * @brief Construct with explicitly supplied settings, primarily for embedding.
     * @param options Native planner options.
     * @param name Plugin instance id.
     * @param costmap Shared native Costmap; required and retained for its lifetime.
     * @param settings Validated geometry and capacity limits.
     */
    VistaPlanner(const proto::PlannerOptions& options, const std::string& name,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
        VistaPlannerSettings settings);

    /**
     * @brief Configure a factory-created plugin with a map consumer source.
     * @param options Native options retained by the GlobalPlanner base.
     * @param name Nonempty native instance id.
     * @param source Sole immutable map-update source, shared with exploration.
     * @param settings Geometry and capacity limits for this instance.
     * @throws std::logic_error The plugin has already been configured.
     */
    void Configure(const proto::PlannerOptions& options, const std::string& name,
                   MapViewSource source, VistaPlannerSettings settings = {});
    /** @brief Replace the immutable map provider and invalidate in-flight requests. */
    void SetMapViewSource(MapViewSource source);
    /** @brief Set genuine alignment/observation metadata; reads do not manufacture it. */
    void SetCostmapReadMetadata(CostmapReadMetadata metadata);
    /** @return Shared graph owner used by both ordinary navigation and exploration. */
    std::shared_ptr<VistaRoutePlanner> SharedRoutePlanner() const;
    /** @return Last accepted owned map view, if a valid read has completed. */
    std::optional<perception::MapReadView> LatestMapView() const;
    /** @return Last successful route and its version evidence, or no committed route. */
    std::optional<VistaRoute> LastRoute() const;

    /**
     * @brief Query the configured graph without superseding a navigation action.
     * @param start Exact source pose; occupied starts are never silently moved.
     * @param goal Exact destination and final orientation.
     * @param[out] route Owned result with evidence from its actual graph snapshot.
     * @param evidence_only Refresh the source and return provenance, with no path.
     * @param cancel_checker Optional bounded-work predicate, called outside locks.
     * @return Native PlannerResultCode; evidence-only success does not set route.success.
     * @details Reconfiguration or concurrent geometry replacement rejects a stale
     * result. A query never reads LastRoute from another request.
     */
    uint32 QueryRoute(const automsgs::msgs::geometry_msgs::PoseStamped& start,
        const automsgs::msgs::geometry_msgs::PoseStamped& goal, VistaRoute& route,
        bool evidence_only = false, std::function<bool()> cancel_checker = {});

    /**
     * @brief Generate a native path without exposing stale or partial output.
     * @param start Valid start pose in the map view's frame.
     * @param goal Valid goal pose, preserving the requested final orientation.
     * @param[out] plan Cleared on failure, cancellation and callback exceptions.
     * @param cancel_checker Optional cancellation predicate, invoked without state locks.
     * @return A native PlannerResultCode; success is a planar route certificate only.
     */
    uint32 CreatePlan(const automsgs::msgs::geometry_msgs::PoseStamped& start,
        const automsgs::msgs::geometry_msgs::PoseStamped& goal,
        automsgs::msgs::nav_msgs::Path& plan,
        std::function<bool()> cancel_checker) override;

private:
    /** @brief Validate and install internal owners; caller holds the configuration lock. */
    void Initialize(VistaPlannerSettings settings);
    mutable std::mutex mutex_; ///< Protects configuration and committed request evidence.
    VistaPlannerSettings settings_; ///< Installed geometry and capacity constraints.
    CostmapReadMetadata metadata_; ///< Truthful upstream source/alignment evidence.
    std::shared_ptr<NativeCostmapReader> costmap_reader_; ///< Locked native read bridge.
    std::shared_ptr<VistaRoutePlanner> route_planner_; ///< Sole persistent route graph owner.
    MapViewSource map_source_; ///< Optional externally owned immutable view source.
    std::optional<perception::MapReadView> latest_map_; ///< Last version-checked read.
    std::optional<VistaRoute> last_route_; ///< Last version-checked successful route.
    std::uint64_t configuration_version_ = 0; ///< Incremented whenever input configuration changes.
    std::uint64_t latest_request_ = 0; ///< Latest request allowed to publish a path.
};

}  // namespace autonomy::planning::planner::vista
