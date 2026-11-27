/* Copyright 2026 The Openbot Authors. */
/** @file goal_pose_adapter_test.cpp
 * @brief Exact-pose and route-provenance regressions for observation
 * adaptation.
 */
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "autonomy/task/exploration/integration/goal_pose_adapter.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace n = autonomy::navigation;
namespace p = autonomy::perception;
namespace v = autonomy::planning::planner::vista;

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

class VisibleSensor final : public e::SensorVisibilityModel
{
public:
    p::SensorKind sensor() const override {
        return p::SensorKind::kRgbd;
    }
    std::optional<e::CoverageContribution> Predict(
        const e::Pose2d&, const e::CoverageUnit& unit, const e::MapReadView&,
        const e::ObservationQuality&) const override {
        return e::CoverageContribution{unit.id, sensor(), 1.0};
    }
};

class CertifiedPaths final : public e::PathProvider
{
public:
    CertifiedPaths(std::uint64_t map_epoch, std::uint64_t geometry_revision)
        : map_epoch_(map_epoch), geometry_revision_(geometry_revision) {}
    v::VistaRoute Route(const e::Pose2d& start,
                        const e::Pose2d& goal) const override {
        v::VistaRoute route;
        route.success = true;
        route.length = n::Distance(start, goal);
        route.map_epoch = map_epoch_;
        route.geometry_revision = geometry_revision_;
        route.path.frame_id = "map";
        v::VistaMapEvidence evidence;
        evidence.map_epoch = route.map_epoch;
        evidence.geometry_revision = route.geometry_revision;
        evidence.frame_id = route.path.frame_id;
        route.evidence = std::move(evidence);
        return route;
    }

private:
    std::uint64_t map_epoch_;
    std::uint64_t geometry_revision_;
};

struct Fixture {
    e::MapReadView map;
    e::CoverageState coverage;
    e::CoverageUnit unit;
    std::shared_ptr<e::ViewpointEvaluator> evaluator;

    Fixture() {
        n::GridMap grid(20, 20, 0.1);
        grid.Fill({n::CellState::kFree, 0});
        e::MapReader reader;
        map = reader.Read(grid, 4, 0, 9, true);
        coverage.Reset(map.map_epoch);
        unit.id = {10, 10, 0, 0, e::CoverageKind::kSurface};
        unit.center = {1.0, 1.0, 0.0};
        unit.required_quality = 1.0;
        coverage.Upsert(unit);
        evaluator = std::make_shared<e::ViewpointEvaluator>(
            std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{
                std::make_shared<VisibleSensor>()});
    }

    e::ObservationGoal Goal() const {
        e::ObservationGoal goal;
        goal.goal_id = 17;
        goal.map_epoch = map.map_epoch;
        goal.original_pose = {1.0, 1.0, 0.4};
        goal.required_views = {0.4};
        goal.required_coverage_units = {unit.id};
        goal.minimum_gain = 0.5;
        return goal;
    }
};

void ExactRequestedPoseAndYawArePreserved() {
    Fixture fixture;
    auto paths = std::make_shared<CertifiedPaths>(
        fixture.map.map_epoch, fixture.map.geometry_revision);
    e::GoalPoseAdapter adapter(paths, fixture.evaluator, n::SafetyOptions{},
                               0.0);
    const auto goal = fixture.Goal();
    const auto result =
        adapter.Adapt(goal, {0.5, 0.5, 0}, fixture.map, fixture.coverage,
                      p::Mask(p::SensorKind::kRgbd));
    Require(result.has_value(),
            "an exact visible and reachable goal was rejected");
    Require(
        std::abs(result->execution_pose.x - goal.original_pose.x) < 1e-12 &&
            std::abs(result->execution_pose.y - goal.original_pose.y) < 1e-12 &&
            std::abs(result->execution_pose.yaw - goal.original_pose.yaw) <
                1e-12,
        "zero adaptation allowance changed the exact requested pose or yaw");
    Require(result->intent.original_pose.x == goal.original_pose.x &&
                result->intent.required_views == goal.required_views,
            "adaptation replaced immutable observation intent evidence");
}

void StaleVistaEvidenceAndMissingMeasuredUnitsFailClosed() {
    Fixture fixture;
    e::GoalPoseAdapter stale_adapter(
        std::make_shared<CertifiedPaths>(fixture.map.map_epoch,
                                         fixture.map.geometry_revision + 1),
        fixture.evaluator, n::SafetyOptions{}, 0.0);
    Require(!stale_adapter
                 .Adapt(fixture.Goal(), {0.5, 0.5, 0}, fixture.map,
                        fixture.coverage, p::Mask(p::SensorKind::kRgbd))
                 .has_value(),
            "a stale Vista geometry certificate was accepted");

    e::CoverageState missing;
    missing.Reset(fixture.map.map_epoch);
    e::GoalPoseAdapter adapter(
        std::make_shared<CertifiedPaths>(fixture.map.map_epoch,
                                         fixture.map.geometry_revision),
        fixture.evaluator, n::SafetyOptions{}, 0.0);
    Require(!adapter
                 .Adapt(fixture.Goal(), {0.5, 0.5, 0}, fixture.map, missing,
                        p::Mask(p::SensorKind::kRgbd))
                 .has_value(),
            "an intent without a measured coverage-ledger unit was accepted");
}

}  // namespace

int main() {
    try {
        ExactRequestedPoseAndYawArePreserved();
        StaleVistaEvidenceAndMissingMeasuredUnitsFailClosed();
        std::cout << "PASS goal_pose_adapter\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL goal_pose_adapter: " << error.what() << '\n';
        return 1;
    }
}
