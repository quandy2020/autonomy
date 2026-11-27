/* Copyright 2026 The Openbot Authors. */
/** @file region_manager.hpp
 * @brief Stable regional scheduling metadata over measured coverage units.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "autonomy/task/exploration/viewpoint/viewpoint.hpp"

namespace autonomy::task::exploration {

/** @brief Scheduling state derived only from measured coverage requirements. */
enum class ExplorationRegionStatus {
    kExploring,  ///< Measured pending work exists or empty state is
                 ///< unconfirmed.
    kCovered,  ///< No measured pending work survived the empty-state debounce.
};

/** @brief Same-epoch membership and measured scheduling history for one region.
 */
struct ExplorationRegion {
    std::pair<int, int> id;
    std::vector<CoverageId> member_units;
    std::vector<CoverageId> pending_units;
    double remaining_area = 0;
    std::size_t measured_visit_count = 0;
    double last_measured_entry = 0;
    std::optional<std::uint64_t> last_representative_view_id;
    ExplorationRegionStatus status =
        ExplorationRegionStatus::kCovered;  ///< Debounced scheduling state.
    std::size_t consecutive_empty_updates =
        0;  ///< Empty refreshes since the latest measured pending work.
    bool remote_relocation_eligible =
        false;  ///< Scheduling-only admission for a coarse remote move.
    std::size_t consecutive_low_value_passes =
        0;  ///< Compatible complete passes without an admissible remote view.
};

/** @brief Maintains stable region records without owning completion or routes.
 */
class RegionManager
{
public:
    /** @brief Construct square world-space bins of the requested side length.
     */
    explicit RegionManager(double region_size = 4.0);
    /** @brief Refresh measured membership, preserving same-epoch history.
     *
     * Measured pending work reopens a region immediately. Two consecutive
     * empty refreshes close only its scheduling state; they do not complete
     * coverage or replace the strategy's exhaustive sweep proof.
     */
    void Update(const CoverageState& coverage);
    /** @brief Reopen remote scheduling on one admissible observed candidate. */
    void ObserveAdmissibleRemoteRegion(const std::pair<int, int>& region);
    /** @brief Debounce low-value remote regions after one complete pass.
     *
     * Candidate predictions update coarse scheduling admission only; they never
     * change measured status, CoverageState, or completion evidence.
     */
    void CompleteRemoteCandidatePass(
        const std::set<std::pair<int, int>>& admissible_regions);
    /** @brief Count only measured robot crossings into a different region. */
    void ObserveRobotPose(const Pose2d& pose, double stamp);
    /** @brief Record the selected representative for scheduling continuity. */
    void SetRepresentative(const std::pair<int, int>& region,
                           std::uint64_t view_id);
    /** @brief Return the floor-binned region containing a world pose. */
    std::pair<int, int> RegionFor(const Pose2d& pose) const;
    /** @brief Expose immutable region scheduling diagnostics. */
    const std::map<std::pair<int, int>, ExplorationRegion>& regions() const {
        return regions_;
    }

private:
    static constexpr std::size_t kEmptyUpdatesToCovered = 2;
    static constexpr std::size_t kLowValuePassesToSuppressRemote = 2;
    double region_size_;
    std::uint64_t epoch_ = 0;
    std::optional<std::pair<int, int>> robot_region_;
    std::map<std::pair<int, int>, ExplorationRegion> regions_;
    std::map<std::pair<int, int>, std::map<CoverageId, std::size_t>>
        pending_requirements_;
};

}  // namespace autonomy::task::exploration
