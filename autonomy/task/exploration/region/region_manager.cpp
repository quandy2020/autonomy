/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/region/region_manager.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace autonomy::task::exploration {
namespace {

std::size_t RequirementFingerprint(const CoverageUnit& unit) {
    std::size_t seed = 0;
    const auto combine = [&seed](std::size_t value) {
        seed ^= value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    const std::hash<double> hash_double;
    combine(hash_double(unit.center.x));
    combine(hash_double(unit.center.y));
    combine(hash_double(unit.center.z));
    combine(hash_double(unit.normal.x));
    combine(hash_double(unit.normal.y));
    combine(hash_double(unit.normal.z));
    combine(hash_double(unit.area));
    combine(hash_double(unit.weight));
    combine(hash_double(unit.tolerance));
    combine(hash_double(unit.required_quality));
    combine(std::hash<perception::SensorMask>{}(unit.required_sensors));
    return seed;
}

}  // namespace

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
        pending_requirements_.clear();
        robot_region_.reset();
        epoch_ = coverage.epoch();
    }
    const auto previous_requirements = pending_requirements_;
    pending_requirements_.clear();
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
        if (!entry.second.pending)
            continue;
        region.pending_units.push_back(entry.first);
        pending_requirements_[id][entry.first] =
            RequirementFingerprint(entry.second);
        region.remaining_area +=
            entry.second.area * coverage.Remaining(entry.first);
    }
    for (auto& entry : regions_) {
        auto& region = entry.second;
        const auto previous = previous_requirements.find(entry.first);
        const auto& current = pending_requirements_[entry.first];
        const bool new_measured_work = std::any_of(
            current.begin(), current.end(), [&](const auto& requirement) {
                return previous == previous_requirements.end() ||
                       previous->second.find(requirement.first) ==
                           previous->second.end() ||
                       previous->second.at(requirement.first) !=
                           requirement.second;
            });
        if (!region.pending_units.empty()) {
            // Measured work reopens measured scheduling immediately. Remote
            // relocation eligibility is independent and may later be suppressed
            // only by compatible complete low-value candidate passes.
            region.status = ExplorationRegionStatus::kExploring;
            region.consecutive_empty_updates = 0;
            if (new_measured_work) {
                region.remote_relocation_eligible = true;
                region.consecutive_low_value_passes = 0;
            }
            continue;
        }
        if (region.status == ExplorationRegionStatus::kExploring &&
            ++region.consecutive_empty_updates >= kEmptyUpdatesToCovered) {
            // This is scheduling hysteresis only. Coverage completion remains
            // owned by measured CoverageState plus the strategy sweep proof.
            region.status = ExplorationRegionStatus::kCovered;
            region.consecutive_empty_updates = 0;
            region.remote_relocation_eligible = false;
            region.consecutive_low_value_passes = 0;
        }
    }
}

void RegionManager::ObserveAdmissibleRemoteRegion(
    const std::pair<int, int>& region_id) {
    const auto found = regions_.find(region_id);
    if (found == regions_.end() || found->second.pending_units.empty())
        return;
    found->second.remote_relocation_eligible = true;
    found->second.consecutive_low_value_passes = 0;
}

void RegionManager::CompleteRemoteCandidatePass(
    const std::set<std::pair<int, int>>& admissible_regions) {
    for (auto& entry : regions_) {
        auto& region = entry.second;
        if (region.pending_units.empty())
            continue;
        if (admissible_regions.count(entry.first)) {
            region.remote_relocation_eligible = true;
            region.consecutive_low_value_passes = 0;
            continue;
        }
        if (region.consecutive_low_value_passes <
            kLowValuePassesToSuppressRemote) {
            ++region.consecutive_low_value_passes;
        }
        if (region.consecutive_low_value_passes >=
            kLowValuePassesToSuppressRemote) {
            region.remote_relocation_eligible = false;
        }
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
