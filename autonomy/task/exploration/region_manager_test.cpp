/* Copyright 2026 The Openbot Authors. */
#include <iostream>
#include <stdexcept>

#include "autonomy/task/exploration/region/region_manager.hpp"

namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace {

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

e::CoverageUnit Unit(int x) {
    e::CoverageUnit unit;
    unit.id = {x, 0, 0, 0, e::CoverageKind::kSurface};
    unit.center = {static_cast<double>(x), 0, 0};
    unit.required_quality = 0.5;
    return unit;
}

void SameEpochPreservesMembershipAndHistory() {
    e::CoverageState coverage;
    coverage.Reset(4);
    auto first = Unit(1);
    auto second = Unit(2);
    coverage.Upsert(first);
    coverage.Upsert(second);
    e::RegionManager regions(4);
    regions.Update(coverage);
    regions.ObserveRobotPose({1, 0, 0}, 2);
    regions.SetRepresentative({0, 0}, 42);
    coverage.Confirm(first.id, p::SensorKind::kRgbd, 1, 3);
    regions.Update(coverage);
    const auto& region = regions.regions().at({0, 0});
    Require(region.member_units.size() == 2 && region.pending_units.size() == 1,
            "completed same-epoch units remain region members");
    Require(region.measured_visit_count == 1 &&
                region.last_representative_view_id == 42,
            "same-epoch updates preserve scheduling history");
    regions.ObserveRobotPose({2, 0, 0}, 4);
    Require(regions.regions().at({0, 0}).measured_visit_count == 1,
            "replanning inside a region does not create a visit");
    regions.ObserveRobotPose({5, 0, 0}, 5);
    regions.ObserveRobotPose({1, 0, 0}, 6);
    Require(regions.regions().at({0, 0}).measured_visit_count == 2,
            "a measured boundary crossing creates exactly one new visit");
}

/** @brief Empty-state debounce prevents status flapping and measured work
 * reopens.
 */
void MeasuredWorkStatusHysteresis() {
    e::CoverageState coverage;
    coverage.Reset(7);
    const auto unit = Unit(1);
    coverage.Upsert(unit);
    e::RegionManager regions(4);
    regions.Update(coverage);
    regions.ObserveRobotPose({1, 0, 0}, 1);
    regions.SetRepresentative({0, 0}, 77);
    Require(regions.regions().at({0, 0}).status ==
                e::ExplorationRegionStatus::kExploring,
            "measured pending work opens its region immediately");

    coverage.Confirm(unit.id, p::SensorKind::kRgbd, 1, 2);
    regions.Update(coverage);
    Require(regions.regions().at({0, 0}).status ==
                e::ExplorationRegionStatus::kExploring,
            "one empty update cannot close an exploring region");
    coverage.Invalidate(unit.id);
    regions.Update(coverage);
    Require(regions.regions().at({0, 0}).status ==
                e::ExplorationRegionStatus::kExploring,
            "reappearing measured work resets the empty-state debounce");

    coverage.Confirm(unit.id, p::SensorKind::kRgbd, 1, 3);
    regions.Update(coverage);
    Require(regions.regions().at({0, 0}).status ==
                e::ExplorationRegionStatus::kExploring,
            "the reset debounce still requires a second empty update");
    regions.Update(coverage);
    Require(regions.regions().at({0, 0}).status ==
                e::ExplorationRegionStatus::kCovered,
            "two consecutive empty updates close the scheduling region");

    coverage.Remove(unit.id);
    regions.Update(coverage);
    const auto& empty = regions.regions().at({0, 0});
    Require(empty.member_units.empty() && empty.pending_units.empty() &&
                empty.measured_visit_count == 1 &&
                empty.last_representative_view_id == 77,
            "same-epoch empty region records retain measured history");
    coverage.Upsert(unit);
    regions.Update(coverage);
    Require(regions.regions().at({0, 0}).status ==
                e::ExplorationRegionStatus::kExploring,
            "new measured pending work reopens a covered empty region");
}

/** @brief Complete low-value passes suppress only coarse remote relocation. */
void LowValuePassHysteresisIsSchedulingOnly() {
    e::CoverageState coverage;
    coverage.Reset(8);
    coverage.Upsert(Unit(1));
    e::RegionManager regions(4);
    regions.Update(coverage);
    Require(regions.regions().at({0, 0}).remote_relocation_eligible,
            "new measured work did not admit remote relocation");

    regions.CompleteRemoteCandidatePass({});
    Require(regions.regions().at({0, 0}).remote_relocation_eligible,
            "one complete low-value pass suppressed remote relocation");
    regions.ObserveAdmissibleRemoteRegion({0, 0});
    regions.CompleteRemoteCandidatePass({});
    regions.CompleteRemoteCandidatePass({});
    const auto& suppressed = regions.regions().at({0, 0});
    Require(!suppressed.remote_relocation_eligible &&
                suppressed.status == e::ExplorationRegionStatus::kExploring &&
                coverage.PendingCount() == 1,
            "low-value passes mutated measured exploration state");

    regions.Update(coverage);
    Require(!regions.regions().at({0, 0}).remote_relocation_eligible,
            "an unchanged requirement bypassed low-value hysteresis");
    coverage.Upsert(Unit(2));
    regions.Update(coverage);
    Require(regions.regions().at({0, 0}).remote_relocation_eligible,
            "new measured work did not immediately reopen remote relocation");
}

/** @brief Completion-only subtraction stays suppressed; changed work reopens.
 */
void RequirementDeltaDirectionControlsReopening() {
    e::CoverageState coverage;
    coverage.Reset(10);
    auto first = Unit(1);
    auto second = Unit(2);
    coverage.Upsert(first);
    coverage.Upsert(second);
    e::RegionManager regions(4);
    regions.Update(coverage);
    regions.CompleteRemoteCandidatePass({});
    regions.CompleteRemoteCandidatePass({});

    coverage.Confirm(first.id, p::SensorKind::kRgbd, 1, 1);
    regions.Update(coverage);
    Require(!regions.regions().at({0, 0}).remote_relocation_eligible,
            "completion-only pending subtraction reopened relocation");

    second.tolerance = .2;
    coverage.Upsert(second);
    regions.Update(coverage);
    Require(regions.regions().at({0, 0}).remote_relocation_eligible,
            "changed same-ID requirement did not reopen relocation");
}

/** @brief New work reopens only its own region's remote scheduling state. */
void UnrelatedRegionWorkDoesNotReopenSuppressedRelocation() {
    e::CoverageState coverage;
    coverage.Reset(9);
    coverage.Upsert(Unit(1));
    coverage.Upsert(Unit(5));
    e::RegionManager regions(4);
    regions.Update(coverage);
    regions.CompleteRemoteCandidatePass({});
    regions.CompleteRemoteCandidatePass({});
    Require(!regions.regions().at({0, 0}).remote_relocation_eligible &&
                !regions.regions().at({1, 0}).remote_relocation_eligible,
            "low-value regions were not suppressed before isolation check");

    coverage.Upsert(Unit(2));
    regions.Update(coverage);
    Require(regions.regions().at({0, 0}).remote_relocation_eligible,
            "new local work did not reopen its own region");
    Require(!regions.regions().at({1, 0}).remote_relocation_eligible,
            "unrelated local work reopened a suppressed remote region");
}

void EpochChangeClearsSchedulingHistory() {
    e::CoverageState coverage;
    coverage.Reset(1);
    coverage.Upsert(Unit(1));
    e::RegionManager regions(4);
    regions.Update(coverage);
    regions.ObserveRobotPose({1, 0, 0}, 1);
    coverage.Reset(2);
    coverage.Upsert(Unit(1));
    regions.Update(coverage);
    const auto& region = regions.regions().at({0, 0});
    Require(
        region.measured_visit_count == 0 && !region.last_representative_view_id,
        "alignment epoch change clears incompatible region history");
}

}  // namespace

int main() {
    try {
        SameEpochPreservesMembershipAndHistory();
        MeasuredWorkStatusHysteresis();
        LowValuePassHysteresisIsSchedulingOnly();
        RequirementDeltaDirectionControlsReopening();
        UnrelatedRegionWorkDoesNotReopenSuppressedRelocation();
        EpochChangeClearsSchedulingHistory();
        std::cout << "PASS region_manager\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL region_manager: " << error.what() << '\n';
        return 1;
    }
}
