/* Copyright 2026 The Openbot Authors. */
/** @file map_reader.hpp
 * @brief Owned read views, changed geometry and independently supplied fusion
 * evidence.
 */
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "autonomy/common/navigation/grid_map.hpp"
#include "autonomy/perception/data/observed_volume.hpp"
#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::perception {
using Pose2d = autonomy::navigation::Pose2d;
/** @brief Whether an unknown cell is absent rolling-window evidence or an
 * authoritative invalidation. */
enum class UnknownCellPolicy { kRetainHistory, kReplace };
/** @brief Closed world-coordinate region used for incremental invalidation. */
struct Bounds2d {
    double min_x = 0;  ///< Minimum world X, meters.
    double min_y = 0;  ///< Minimum world Y, meters.
    double max_x = 0;  ///< Maximum world X, meters.
    double max_y = 0;  ///< Maximum world Y, meters.
    /** @brief Test closed-region overlap, including touching boundaries. */
    bool Intersects(const Bounds2d& b) const {
        return min_x <= b.max_x && max_x >= b.min_x && min_y <= b.max_y &&
               max_y >= b.min_y;
    }
    /** @brief Test a world-coordinate pose position against the closed bounds.
     */
    bool Contains(const Pose2d& p) const {
        return p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y;
    }
};
/** @brief Immutable consumer snapshot; no field writes back to the map owner.
 */
struct MapReadView {
    std::uint64_t map_epoch = 0;          ///< Coordinate-alignment generation.
    std::uint64_t geometry_revision = 0;  ///< Changes to route-model geometry.
    std::uint64_t observation_watermark =
        0;  ///< Diagnostic producer watermark, not fusion proof.
    double source_stamp =
        0;  ///< Original acquisition time in the configured clock.
    std::string frame_id;  ///< World frame of every grid cell.
    std::shared_ptr<const autonomy::navigation::GridMap>
        grid;  ///< Owned immutable route-model copy.
    std::vector<Bounds2d>
        dirty_regions;  ///< Changed world regions before safety halo expansion.
    bool footprint_inflated =
        false;  ///< Robot inflation already applied by the source map.
    perception::ObservationCut
        observation_cut{};  ///< Actual upstream source fusion prefix.
    bool mapping_synchronized =
        false;  ///< Set only by an independently verified fusion barrier.
    bool has_height_evidence =
        true;  ///< False for planar Costmap projection; raw 3D execution checks
               ///< remain separate.
    std::optional<autonomy::navigation::StartupSupportRegion>
        startup_support;  ///< Start-only measured support hint; never map
                          ///< state.
    std::shared_ptr<const ObservedVolumeView>
        observed_volume;  ///< Synchronized measured 3-D evidence; null for
                          ///< planar sources.
    ObservationCut observation_baseline{};  ///< Excluded historical prefix for
                                            ///< an explicitly recovered suffix.
    std::uint64_t observation_recovery_generation =
        0;  ///< Actual upstream replacement-baseline identity.
};
/** @brief Copies bounded snapshots and tracks changes without modifying
 * autonomy/map. */
class MapReader
{
public:
    /** @brief Bound the number of copied cells and retained comparison entries.
     */
    explicit MapReader(std::size_t maximum_cells = 2000000);
    /** @brief Compare an owned snapshot obtained through the map's locked
     * reader.
     * @param alignment_id Changes on a known realignment, not on publication.
     * @param observation_watermark Diagnostic input watermark, not a fusion
     * receipt.
     * @param footprint_inflated Prevents applying the robot footprint margin
     * twice.
     */
    MapReadView Read(
        const autonomy::navigation::GridMap& snapshot, double source_stamp,
        std::uint64_t alignment_id, std::uint64_t observation_watermark,
        bool footprint_inflated = false,
        UnknownCellPolicy unknowns = UnknownCellPolicy::kRetainHistory);
    /** @brief Last owned snapshot; caller must serialize use of this reader. */
    const std::optional<MapReadView>& latest() const {
        return latest_;
    }

private:
    std::size_t maximum_cells_;  ///< Copy/comparison memory bound.
    std::map<std::pair<int, int>, autonomy::navigation::GridCell>
        history_;  ///< Prior cells by stable world-grid key.
    std::optional<MapReadView> latest_;  ///< Latest published owned view.
    std::uint64_t alignment_id_ =
        0;  ///< Last external localization alignment identity.
    std::uint64_t epoch_ = 0;     ///< Consumer coordinate generation.
    std::uint64_t revision_ = 0;  ///< Consumer geometric revision.
    double resolution_ = 0;       ///< Prior grid scale, meters.
    double anchor_x_ = 0;   ///< Stable world-grid X reference for comparisons.
    double anchor_y_ = 0;   ///< Stable world-grid Y reference for comparisons.
    std::string frame_id_;  ///< Prior snapshot frame.
};
}  // namespace autonomy::perception
