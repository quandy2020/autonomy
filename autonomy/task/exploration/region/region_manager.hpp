/* Copyright 2026 The Openbot Authors. */
/** @file region_manager.hpp
 * @brief Stable regional scheduling metadata over measured coverage units.
 */
#pragma once

#include <map>
#include <optional>

#include "autonomy/task/exploration/viewpoint/viewpoint.hpp"

namespace autonomy::task::exploration {

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
    std::uint64_t last_requirements_revision = 0;
};

/** @brief Maintains stable region records without owning completion or routes.
 */
class RegionManager
{
public:
    /** @brief Construct square world-space bins of the requested side length.
     */
    explicit RegionManager(double region_size = 4.0);
    /** @brief Refresh measured membership, preserving same-epoch history. */
    void Update(const CoverageState& coverage);
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
    double region_size_;
    std::uint64_t epoch_ = 0;
    std::optional<std::pair<int, int>> robot_region_;
    std::map<std::pair<int, int>, ExplorationRegion> regions_;
};

}  // namespace autonomy::task::exploration
