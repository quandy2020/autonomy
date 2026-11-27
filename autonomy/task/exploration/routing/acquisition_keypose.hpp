/* Copyright 2026 The Openbot Authors. */
/**
 * @file acquisition_keypose.hpp
 * @brief Provenance-complete acquisition keyposes without route-graph state.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "autonomy/common/navigation/grid_map.hpp"
#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::task::exploration {

/** @brief Configuration for bounded acquisition-keypose retention. */
struct AcquisitionKeyposeLedgerOptions {
    double minimum_spacing =
        1.0;  ///< Minimum XY distance from every retained keypose.
    std::size_t maximum_entries =
        4096;  ///< Maximum retained immutable records.
};

/** @brief Exact admitted acquisition evidence before ledger identity
 * assignment. */
struct AcquisitionKeyposeEvidence {
    autonomy::navigation::Pose2d
        base_pose;  ///< Acquisition-localized base pose in frame_id.
    perception::SensorKind source_sensor = perception::SensorKind::kLidar;
    std::uint64_t source_sequence =
        0;                        ///< Nonzero ordinal of the source packet.
    std::uint64_t map_epoch = 0;  ///< Localization/map alignment identity.
    std::uint64_t geometry_revision = 0;  ///< Map geometry used when retained.
    std::uint64_t calibration_revision =
        0;                         ///< Acquisition calibration identity.
    double acquisition_stamp = 0;  ///< Exact source acquisition time.
    std::string frame_id;  ///< World frame shared by pose and map evidence.
};

/** @brief One retained acquisition pose; it has no edges or route semantics. */
struct AcquisitionKeypose : AcquisitionKeyposeEvidence {
    std::uint64_t id =
        0;  ///< Epoch-scoped, strictly increasing ledger identity.
};

/** @brief Result of one evidence admission attempt. */
enum class AcquisitionKeyposeRecordResult {
    kRecorded,
    kSpacingFiltered,
    kInvalidEvidence,
    kWrongEpoch,
    kStaleProvenance,
};

}  // namespace autonomy::task::exploration
