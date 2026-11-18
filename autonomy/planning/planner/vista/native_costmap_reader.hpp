/* Copyright 2026 The Openbot Authors. */
/** @file native_costmap_reader.hpp
 * @brief Consumer-owned bridge from the existing Costmap to immutable route geometry.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include "autonomy/perception/data/map_reader.hpp"

namespace autonomy::map::costmap_2d { class Costmap2D; }

namespace autonomy::planning::planner::vista {

/** @brief Source evidence supplied by the localization/mapping integration owner. */
struct CostmapReadMetadata {
    std::uint64_t alignment_id = 1; ///< Map/odom alignment generation; publication is not a reset.
    double observation_source_stamp = 0; ///< Actual observation time, or zero when unavailable.
    perception::ObservationCut observation_cut{}; ///< Independent processed sequence per sensor.
    bool fusion_confirmed = false; ///< True only for verified upstream fusion acknowledgement.
    perception::ObservationCut observation_baseline{}; ///< Excluded prefix of a verified replacement baseline.
    std::uint64_t observation_recovery_generation = 0; ///< Genuine mapper recovery identity, not publication time.
};

/** @brief Serialize map reads without modifying native map ownership or interfaces. */
class NativeCostmapReader {
public:
    /** @param maximum_cells Positive capacity of the accumulated consumer map history. */
    explicit NativeCostmapReader(std::size_t maximum_cells = 2000000);
    /**
     * @brief Copy native geometry under its mutex, then convert outside that lock.
     * @param costmap Existing native map owner; remains unmodified.
     * @param frame_id Coordinate frame of the planar map.
     * @param metadata Genuine observation/fusion evidence, if supplied upstream.
     * @param footprint_inflated Input collision geometry already accounts for footprint.
     * @return Immutable view; has_height_evidence is false for this planar source.
     * @details Internal mutex serializes history commits. Read/publication time
     *          never replaces missing observation time or fusion evidence.
     */
    perception::MapReadView Read(map::costmap_2d::Costmap2D& costmap,
        const std::string& frame_id, const CostmapReadMetadata& metadata = {},
        bool footprint_inflated = false);
private:
    std::size_t maximum_cells_; ///< Maximum admissible native snapshot/history extent.
    std::mutex mutex_; ///< Serializes independent caller reads and history publication.
    perception::MapReader reader_; ///< Consumer-owned revision and rolling-window history.
};

}  // namespace autonomy::planning::planner::vista
