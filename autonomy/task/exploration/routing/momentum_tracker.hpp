/* Copyright 2026 The Openbot Authors. */
/**
 * @file momentum_tracker.hpp
 * @brief Measured-motion hysteresis for relocation ordering only.
 */
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>

#include "autonomy/common/navigation/grid_map.hpp"

namespace autonomy::task::exploration {

/** @brief Fixed measured-motion sampling and hysteresis policy. */
struct MomentumTrackerOptions {
    double sampling_distance =
        0.5;  ///< Minimum measured displacement per sample.
    unsigned activation_count_threshold =
        4;  ///< Reversals must strictly exceed this.
    unsigned release_count_threshold =
        4;  ///< Stable samples must strictly exceed this.
};

/** @brief Identity accompanying one authoritative localization sample. */
struct MomentumSampleIdentity {
    std::uint64_t session_id = 0;
    std::uint64_t map_epoch = 0;
    std::uint64_t localization_generation = 0;
    std::string frame_id;
};

/** @brief Read-only momentum state for global relocation tie-breaking. */
struct MomentumState {
    bool active = false;
    unsigned reversal_count = 0;
    unsigned stable_count = 0;
    std::optional<Eigen::Vector2d> preferred_direction;
};

/**
 * @brief Tracks direction oscillation exclusively from measured robot motion.
 * @details This class cannot authorize routes, alter their cost, or certify
 * coverage. Consumers may use an active preferred direction only when ordering
 * otherwise-valid global relocation alternatives.
 */
class MomentumTracker
{
public:
    /** @brief Construct with finite positive displacement sampling. */
    explicit MomentumTracker(MomentumTrackerOptions options = {});

    /** @brief Clear all measured-motion history and hysteresis state. */
    void Reset() noexcept;

    /**
     * @brief Admit one acquisition-localized robot pose.
     * @return Current state after processing or resetting on discontinuity.
     */
    MomentumState Update(const autonomy::navigation::Pose2d& measured_pose,
                         double acquisition_stamp,
                         const MomentumSampleIdentity& identity) noexcept;

    /** @return Current measured-motion hysteresis state. */
    MomentumState state() const noexcept;

private:
    bool SameIdentity(const MomentumSampleIdentity& identity) const noexcept;

    MomentumTrackerOptions options_;
    std::optional<MomentumSampleIdentity> identity_;
    std::optional<autonomy::navigation::Pose2d> sample_pose_;
    std::optional<Eigen::Vector2d> movement_direction_;
    double latest_stamp_ = 0;
    bool active_ = false;
    unsigned reversal_count_ = 0;
    unsigned stable_count_ = 0;
};

}  // namespace autonomy::task::exploration
