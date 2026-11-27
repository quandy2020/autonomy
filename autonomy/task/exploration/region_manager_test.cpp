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
        EpochChangeClearsSchedulingHistory();
        std::cout << "PASS region_manager\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL region_manager: " << error.what() << '\n';
        return 1;
    }
}
