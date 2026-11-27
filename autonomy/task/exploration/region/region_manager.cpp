/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/region/region_manager.hpp"

#include <stdexcept>

namespace autonomy::task::exploration {

RegionManager::RegionManager(double size) : region_size_(size) {
    if (!std::isfinite(size) || size <= 0)
        throw std::invalid_argument("invalid region size");
}

std::pair<int, int> RegionManager::RegionFor(const Pose2d& pose) const {
    return {static_cast<int>(std::floor(pose.x / region_size_)),
            static_cast<int>(std::floor(pose.y / region_size_))};
}

void RegionManager::Update(const CoverageState& coverage) {
    if (coverage.epoch() != epoch_) {
        regions_.clear();
        robot_region_.reset();
        epoch_ = coverage.epoch();
    }
    for (auto& entry : regions_) {
        entry.second.member_units.clear();
        entry.second.pending_units.clear();
        entry.second.remaining_area = 0;
    }
    for (const auto& entry : coverage.units()) {
        const auto id =
            RegionFor({entry.second.center.x, entry.second.center.y, 0});
        auto& region = regions_[id];
        region.id = id;
        region.member_units.push_back(entry.first);
        region.last_requirements_revision = coverage.requirements_revision();
        if (!entry.second.pending)
            continue;
        region.pending_units.push_back(entry.first);
        region.remaining_area +=
            entry.second.area * coverage.Remaining(entry.first);
    }
}

void RegionManager::ObserveRobotPose(const Pose2d& pose, double stamp) {
    if (!std::isfinite(stamp))
        throw std::invalid_argument("invalid measured region entry time");
    const auto region_id = RegionFor(pose);
    if (robot_region_ && *robot_region_ == region_id)
        return;
    robot_region_ = region_id;
    auto& region = regions_[region_id];
    region.id = region_id;
    ++region.measured_visit_count;
    region.last_measured_entry = stamp;
}

void RegionManager::SetRepresentative(const std::pair<int, int>& region_id,
                                      std::uint64_t view_id) {
    auto& region = regions_[region_id];
    region.id = region_id;
    region.last_representative_view_id = view_id;
}

}  // namespace autonomy::task::exploration
