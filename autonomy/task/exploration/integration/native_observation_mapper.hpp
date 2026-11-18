/* Copyright 2026 The Openbot Authors. */
/** @file native_observation_mapper.hpp
 * @brief Native measured-ray map fusion and generation-specific persistence services.
 */
#pragma once

#include <filesystem>
#include <memory>
#include <automsgs/msgs/map_msgs/grid_map.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include "autonomy/perception/observation/observation_ingress.hpp"
#include "autonomy/task/exploration/exploration_context.hpp"

namespace autonomy::task::exploration {

/** @brief Explicit bounded geometry and sensor installation policy for native mapping. */
struct NativeObservationMapperOptions {
    perception::SensorMode mode = perception::SensorMode::kRgbdOnly;  ///< Actual source mode; no synthetic sensor inputs.
    std::string frame_id;  ///< Same navigation world frame as acquisition localization.
    Bounds2d region;  ///< Finite allocated work domain, never inferred from a GT map.
    double resolution = .1;  ///< Shared grid_map and ESDF voxel size, metres.
    double minimum_height = -.2;  ///< Allocated world Z minimum, metres.
    double maximum_height = 2.5;  ///< Allocated world Z maximum, metres.
    double robot_height = .25;  ///< Actual chassis height for conservative navigable-column classification.
    double vertical_margin = .05;  ///< Additional required vertical clearance, metres.
    double ground_tolerance = .03;  ///< Ground classifier's actual measured-height tolerance, metres.
    std::array<double, 2> sensor_heights{0, 0};  ///< Actual LiDAR/RGB-D installation heights above the ground prior.
    std::size_t maximum_voxels = 2000000;  ///< Allocation bound for existing ESDF and observed-voxel bookkeeping.
    std::size_t maximum_rays = 100000;  ///< Actual per-packet measured-ray work bound.
    std::size_t maximum_pending_ordinals = 4096;  ///< Bounded out-of-order completion bookkeeping per source/generation.
    std::filesystem::path output_directory;  ///< Explicit native map persistence directory.
};

/** @brief Reuses existing Atlas ground/occupancy algorithms, map ESDF and grid_map storage.
 * No autonomy/map source is changed. Received measurements retain their task
 * generation and sensor ordinal; a queue count never becomes a fusion receipt.
 * Generation zero contributes geometry only. Missing ordinals prevent a finite
 * prefix from being certified, and persistence is acknowledged only after real
 * native messages are written and synchronized to disk.
 */
class NativeObservationMapper {
public:
    /** @brief Allocate a bounded native map from explicit scope and installation. */
    NativeObservationMapper(NativeObservationMapperOptions options, perception::ObservationIngress::Clock clock);
    ~NativeObservationMapper();
    NativeObservationMapper(const NativeObservationMapper&) = delete;
    NativeObservationMapper& operator=(const NativeObservationMapper&) = delete;
    /** @brief Fuse actual validated rays on the task owner and return immutable geometry.
     * @details Replaying one generation/sensor ordinal does not integrate it twice.
     * No-hit beams remain measured free-space rays; invalid/missing beams are not
     * fabricated. ESDF's unobserved-free default is masked by actual ray bookkeeping.
     */
    std::optional<MapReadView> Process(std::uint64_t generation,
        const perception::ObservationPacket& packet, double now);
    /** @brief Create independent versioned refresh/save/cancel operations for one task generation. */
    ExplorationServices Services(std::uint64_t generation);
    /** @brief Shared immutable source for PlannerServer's existing Vista graph; creates no second graph. */
    std::optional<MapReadView> LatestView() const;
    /** @brief Read the actual native occupancy output; absence is not a successful fusion result. */
    std::optional<::automsgs::msgs::map_msgs::OccupancyGrid> Occupancy() const;
    /** @brief Read actual grid_map layers, preserving unknown elevations and clearances as NaN. */
    std::optional<::automsgs::msgs::map_msgs::GridMap> Grid() const;
private:
    class Impl;  ///< Existing map algorithms, measured-voxel mask and generation-specific finite prefixes.
    std::shared_ptr<Impl> impl_;  ///< Retained by outstanding mapping operations until their actual retirement.
};
}  // namespace autonomy::task::exploration
