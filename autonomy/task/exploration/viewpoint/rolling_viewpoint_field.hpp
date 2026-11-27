/* Copyright 2026 The Openbot Authors. */
/** @file rolling_viewpoint_field.hpp
 * @brief Persistent rolling 3-D clearance evidence for sampled viewpoints.
 */
#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>

#include "autonomy/perception/data/observed_volume.hpp"
#include "autonomy/task/exploration/integration/map_reader.hpp"

namespace autonomy::task::exploration {

/** @brief Stored state of one voxel in the rolling field. */
enum class ViewpointVoxelState : std::uint8_t {
    kUnknown = 0,
    kFree = 1,
    kOccupied = 2,
};

/** @brief Fail-closed result of checking one executable viewpoint column. */
enum class ViewpointFieldEligibility : std::uint8_t {
    kEligible = 0,
    kEpochMismatch,
    kGeometryRevisionMismatch,
    kFrameMismatch,
    kOutsideWindow,
    kUnknownClearance,
    kOccupiedClearance,
};

/** @brief Outcome of one transactional field update. */
enum class ViewpointFieldUpdateStatus : std::uint8_t {
    kPublished = 0,
    kUnchanged,
    kRejectedInvalid,
    kRejectedStale,
    kCanceled,
};

/** @brief Fixed memory and rollover policy for the field. */
struct RollingViewpointFieldOptions {
    Eigen::Vector3i dimensions{0, 0,
                               0};  ///< Window dimensions in source voxels.
    Eigen::Vector3i rollover_step{0, 0, 0};  ///< Origin quantization in voxels.
    std::size_t maximum_cells = 0;  ///< Hard storage and update-work bound.
    std::size_t maximum_input_voxels = 0;  ///< Sparse-input admission bound.
    bool require_mapping_synchronized =
        true;  ///< Reject unbarriered map views.
};

/** @brief Diagnostics for one field update attempt. */
struct ViewpointFieldUpdateResult {
    ViewpointFieldUpdateStatus status =
        ViewpointFieldUpdateStatus::kRejectedInvalid;
    std::uint64_t generation = 0;     ///< Published geometry generation.
    std::size_t measured_voxels = 0;  ///< Accepted sparse voxel count.
    std::size_t changed_voxels = 0;  ///< Changed states in an unchanged layout.
    bool epoch_reset = false;  ///< Coordinate identity required a hard reset.
    bool rolled = false;       ///< The rolling window origin changed.
};

/** @brief Result of checking one robot body column. */
struct ViewpointFieldQuery {
    ViewpointFieldEligibility eligibility =
        ViewpointFieldEligibility::kOutsideWindow;
    std::size_t unknown_voxels = 0;
    std::size_t occupied_voxels = 0;

    /** @brief Whether the complete queried body column is measured free. */
    bool eligible() const {
        return eligibility == ViewpointFieldEligibility::kEligible;
    }
};

/** @brief Immutable field snapshot safe for concurrent sampling readers. */
struct RollingViewpointFieldView {
    std::uint64_t map_epoch = 0;
    std::uint64_t geometry_revision = 0;
    std::uint64_t generation = 0;
    double source_stamp = 0;
    std::string frame_id;
    perception::ObservationCut observation_cut{};
    Eigen::Vector3d lattice_origin{0, 0, 0};
    Eigen::Vector3i source_dimensions{0, 0, 0};
    Eigen::Vector3i window_origin{0, 0, 0};
    Eigen::Vector3i window_dimensions{0, 0, 0};
    double resolution = 0;

    /** @brief Checks whether this field belongs to the supplied route map. */
    bool Compatible(std::uint64_t expected_epoch,
                    std::uint64_t expected_geometry_revision,
                    std::string_view expected_frame) const noexcept;

    /** @brief Checks measured 3-D clearance above a certified floor.
     *
     * The voxel containing the floor is excluded because floor support is
     * certified by the route map. Every intersecting voxel above that support
     * voxel through the requested robot height must be measured free.
     */
    ViewpointFieldQuery CheckViewpoint(
        const Pose2d& pose, double floor_height, double robot_radius,
        double robot_height, double horizontal_margin, double vertical_margin,
        double clearance_reserve = 0) const noexcept;

private:
    struct Storage;
    std::shared_ptr<const Storage> storage;
    friend class RollingViewpointField;
};

/** @brief Bounded persistent 3-D field without route or lifecycle ownership. */
class RollingViewpointField final
{
public:
    /** @brief Creates a field and validates its fixed allocation policy.
     * @throws std::invalid_argument If dimensions or work limits are invalid.
     */
    explicit RollingViewpointField(RollingViewpointFieldOptions options);

    RollingViewpointField(const RollingViewpointField&) = delete;
    RollingViewpointField& operator=(const RollingViewpointField&) = delete;

    /** @brief Transactionally updates the field from a complete sparse volume.
     *
     * Invalid, stale, or canceled input never changes the published snapshot.
     * Callers may read concurrently; callers must not invoke Update and Clear
     * concurrently with each other.
     */
    ViewpointFieldUpdateResult Update(
        const perception::ObservedVolumeView& volume,
        const Eigen::Vector3d& focus_position,
        const std::function<bool()>& cancel_checker = {});

    /** @brief Returns the current immutable snapshot, or null before
     * publication.
     */
    std::shared_ptr<const RollingViewpointFieldView> Read() const noexcept;

    /** @brief Removes published geometry and advances the field generation. */
    void Clear() noexcept;

private:
    RollingViewpointFieldOptions options_;
    mutable std::mutex mutex_;
    std::shared_ptr<const RollingViewpointFieldView> published_;
    std::uint64_t generation_ = 0;
};

}  // namespace autonomy::task::exploration
