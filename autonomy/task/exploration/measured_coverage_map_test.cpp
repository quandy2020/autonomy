/* Copyright 2026 The Openbot Authors. */
/** @file measured_coverage_map_test.cpp
 * @brief Verify actual measured-ledger projection and multi-source/invalidation semantics.
 */
#include <iostream>
#include <stdexcept>
#include "autonomy/task/exploration/coverage/measured_coverage_map.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
/** @brief Fail an actual coverage/identity contract. */
void Require(bool value,const char* reason) { if (!value) throw std::runtime_error(reason); }
/** @brief Geometry is independently known; it must never manufacture measured coverage. */
p::MapReadView Map() {
    auto grid = std::make_shared<opt::navigation::GridMap>(4,4,1,0,0,"map");
    grid->Fill({opt::navigation::CellState::kFree,0,1});
    p::MapReadView view; view.grid = grid; view.map_epoch = 4; view.frame_id = "map";
    return view;
}
/** @brief Explicit target whose initial caller-provided quality is not imported into the owner ledger. */
e::CoverageUnit Target(int direction = 0,double area = 1) {
    e::CoverageUnit unit; unit.id = {1,1,0,direction,e::CoverageKind::kSurface};
    unit.center = {1.5,1.5,.5}; unit.required_sensors = p::Mask(p::SensorMode::kLidarRgbd);
    unit.required_quality = 1; unit.area = area; unit.observed_quality = {1,1};
    return unit;
}
/** @brief All required sources and every overlapping target must be confirmed before encoding 100. */
void MeasuredOnly() {
    auto map = Map(); e::CoverageState ledger; ledger.Reset(4);
    const auto empty = e::BuildMeasuredCoverageMap(ledger,map,1);
    for (int value : empty.data()) Require(value == -1,"known free geometry cannot invent target-ledger evidence");
    auto target = Target(); ledger.Upsert(target);
    auto projection = e::BuildMeasuredCoverageMap(ledger,map,1);
    Require(projection.data(5) == 0,"caller-provided observed quality is not a measurement");
    ledger.Confirm(target.id,p::SensorKind::kLidar,1,1);
    projection = e::BuildMeasuredCoverageMap(ledger,map,1.1);
    Require(projection.data(5) == 50,"one source cannot satisfy an actual two-source requirement");
    ledger.Confirm(target.id,p::SensorKind::kRgbd,.9999,1.1);
    projection = e::BuildMeasuredCoverageMap(ledger,map,1.2);
    Require(projection.data(5) < 100,"quantization cannot hide a still pending source requirement");
    ledger.Confirm(target.id,p::SensorKind::kRgbd,1,1.2);
    const auto confirmed = e::BuildMeasuredCoverageMap(ledger,map,1.3);
    Require(confirmed.data(5) == 100,"all actual measurements confirm this target cell");
    ledger.Upsert(Target(1,.001));
    projection = e::BuildMeasuredCoverageMap(ledger,map,1.4);
    Require(projection.data(5) < 100,"a small unobserved overlapping target cannot disappear in area averaging");
    ledger.Remove(Target(1).id); ledger.InvalidateSources(p::Mask(p::SensorKind::kRgbd));
    projection = e::BuildMeasuredCoverageMap(ledger,map,1.5);
    Require(projection.data(5) == 50 && confirmed.data(5) == 100,
        "source invalidation reopens current coverage while prior immutable output remains unchanged");
    ledger.Invalidate(target.id);
    Require(e::BuildMeasuredCoverageMap(ledger,map,1.6).data(5) == 0,"invalidated geometry clears measured target quality");
    Require(projection.info().width() == 4 && projection.info().resolution() == 1 && projection.header().frame_id() == "map",
        "actual native coverage output retains its map lattice and coordinate frame");
}
/** @brief Mismatched coordinates and unbounded input cannot be projected into plausible output. */
void IdentityAndBudget() {
    auto map = Map(); e::CoverageState ledger; ledger.Reset(3);
    bool rejected = false;
    try { (void)e::BuildMeasuredCoverageMap(ledger,map,1); } catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected,"mismatched map/ledger epochs fail");
    ledger.Reset(4); rejected = false;
    try { (void)e::BuildMeasuredCoverageMap(ledger,map,1,2); } catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected,"coverage projection respects its real cell work bound");
}
}  // namespace
/** @brief Run native measured projection contracts against the actual owner ledger implementation. */
int main() {
    try { MeasuredOnly(); IdentityAndBudget(); std::cout << "PASS measured_coverage_map" << std::endl; return 0; }
    catch (const std::exception& error) { std::cerr << "FAIL measured_coverage_map: " << error.what() << std::endl; return 1; }
}
