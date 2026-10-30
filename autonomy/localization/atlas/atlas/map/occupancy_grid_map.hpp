/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "atlas/map/map_types.hpp"

namespace autonomy::localization::atlas::map {

/** Sparse two-dimensional log-odds occupancy map with ray clearing. */
class OccupancyGridMap {
public:
    explicit OccupancyGridMap(OccupancyGridOptions options);

    OccupancyGridMap(const OccupancyGridMap&) = delete;
    OccupancyGridMap& operator=(const OccupancyGridMap&) = delete;
    OccupancyGridMap(OccupancyGridMap&&) noexcept = default;
    OccupancyGridMap& operator=(OccupancyGridMap&&) noexcept = default;

    /**
     * Applies one scan. Miss cells are de-duplicated and updated before hit
     * cells, so an endpoint remains occupied when several rays cross it.
     */
    std::size_t Integrate(
        const Eigen::Vector3f& sensor_origin,
        const std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>&
            points);
    /**
     * Integrates explicit navigation semantics. Ground is a miss, obstacle is
     * a hit, and unknown performs no update (it is never encoded as log-odds
     * zero, which Atlas would otherwise render as occupied).
     */
    std::size_t Integrate(const Eigen::Vector3f& sensor_origin,
                          const SemanticPointCloud& points);
    OccupancyGridSnapshot Snapshot(double timestamp,
                                   std::uint64_t generation) const;
    void Reset();

    std::size_t size() const { return cells_.size(); }
    bool ProbabilityAt(double x, double y, double* probability) const;
    const OccupancyGridOptions& options() const { return options_; }

private:
    struct CellKey {
        std::int32_t x = 0;
        std::int32_t y = 0;

        bool operator==(const CellKey& other) const {
            return x == other.x && y == other.y;
        }
    };

    struct CellKeyHash {
        std::size_t operator()(const CellKey& key) const;
    };

    bool KeyForPoint(double x, double y, CellKey* key) const;
    void AddRay(const CellKey& begin, const CellKey& end,
                bool endpoint_is_occupied,
                std::unordered_set<CellKey, CellKeyHash>* free_cells) const;
    bool Update(const CellKey& key, double delta);
    void ApplyInflation(OccupancyGridSnapshot* snapshot) const;

    OccupancyGridOptions options_;
    double hit_log_odds_ = 0.0;
    double miss_log_odds_ = 0.0;
    double minimum_log_odds_ = 0.0;
    double maximum_log_odds_ = 0.0;
    std::unordered_map<CellKey, float, CellKeyHash> cells_;
};

}  // namespace autonomy::localization::atlas::map
