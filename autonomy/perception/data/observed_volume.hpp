/* Copyright 2026 The Openbot Authors. */
/** @file observed_volume.hpp
 * @brief Immutable measured 3-D occupancy evidence for exploration consumers.
 */
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::perception {

/** @brief Occupancy state backed by at least one actual measured ray. */
enum class ObservedVoxelState : std::uint8_t {
    kFree = 0,
    kOccupied = 1,
};

/** @brief One measured voxel in the fixed world-aligned volume lattice. */
struct ObservedVoxel {
    Eigen::Vector3i index{0, 0, 0};  ///< Zero-based lattice index.
    ObservedVoxelState state =
        ObservedVoxelState::kFree;  ///< Latest fused state.
};

/** @brief Owned immutable sparse view of every measured voxel in one map epoch.
 *
 * Unlisted voxels are unknown. They must never be interpreted as free space or
 * as measured coverage. The lattice metadata is immutable and sufficient to
 * recover each voxel's world-space bounds without sharing mapper storage.
 */
struct ObservedVolumeView {
    std::uint64_t map_epoch = 0;  ///< Coordinate-alignment generation.
    std::uint64_t geometry_revision =
        0;                    ///< Matching 2-D route geometry revision.
    double source_stamp = 0;  ///< Latest included acquisition time, seconds.
    std::string
        frame_id;  ///< World frame shared with acquisition localization.
    Eigen::Vector3d origin{0, 0, 0};  ///< World minimum of lattice index zero.
    Eigen::Vector3i dimensions{0, 0, 0};  ///< Allocated lattice dimensions.
    double resolution = 0;             ///< Isotropic voxel edge length, metres.
    ObservationCut observation_cut{};  ///< Actual fused source prefix.
    bool mapping_synchronized =
        false;  ///< True only for a task generation cut.
    std::shared_ptr<const std::vector<ObservedVoxel>>
        voxels;  ///< Sparse measured cells.
};

}  // namespace autonomy::perception
