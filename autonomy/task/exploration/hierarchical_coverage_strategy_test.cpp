/* Copyright 2026 The Openbot Authors. */
/** @file hierarchical_coverage_strategy_test.cpp
 * @brief Production regressions for bounded sampling and paged coverage sweeps.
 */
#include "autonomy/task/exploration/strategy/hierarchical_coverage_strategy.hpp"

#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "autonomy/task/exploration/integration/vista_path_provider.hpp"
#include "autonomy/task/exploration/observation/camera_visibility_model.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace n = autonomy::navigation;
namespace v = autonomy::planning::planner::vista;

/** @brief Fails a regression without disabling checks in release builds. */
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

/** @brief Real geometry, camera model and route provider for sampler regressions. */
struct Fixture {
    e::MapReadView map;  ///< Owned immutable geometry for this test.
    e::CoverageState coverage;  ///< Measured requirement ledger.
    e::Pose2d start{.1, .1, 0};  ///< Known free initial cell center.
    std::shared_ptr<e::ViewpointEvaluator> evaluator;  ///< Calibrated camera predictor.
    std::shared_ptr<e::VistaPathProvider> paths;  ///< Real visibility-graph routing.
    e::HierarchicalCoverageOptions options;  ///< Small page limits expose resumption.

    /** @brief Creates an open or mostly invalid candidate domain. */
    explicit Fixture(n::CellState fill = n::CellState::kFree, int width = 6,
                     int height = 4) {
        n::GridMap grid(width, height, .2);
        grid.Fill({fill, 0});
        grid.Set(0, 0, {n::CellState::kFree, 0});
        if (fill != n::CellState::kFree)
            grid.Set(width - 1, height - 1, {n::CellState::kFree, 0});
        e::MapReader reader;
        map = reader.Read(grid, 1, 0, 1, true);
        coverage.Reset(map.map_epoch);
        auto camera = std::make_shared<e::CameraVisibilityModel>(
            p::CameraModel{80, 60, 80, 80, 40, 30, .1, 8}, p::RigidTransform{});
        evaluator = std::make_shared<e::ViewpointEvaluator>(
            std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{camera});
        auto planner = std::make_shared<v::VistaRoutePlanner>();
        planner->Update(map);
        paths = std::make_shared<e::VistaPathProvider>(planner);
        options.sampling.yaw_samples = 2;
        options.sampling.maximum_evaluations = 3;
        options.sampling.maximum_examined = 3;
    }

    /** @brief Adds a real requirement outside the fixture camera's visible domain. */
    e::CoverageUnit AddRequirement(int x = 100) {
        e::CoverageUnit unit;
        unit.id = {x, 0, 0, 0, e::CoverageKind::kSurface};
        unit.center = {20, 0, 0};
        unit.required_quality = 1;
        coverage.Upsert(unit);
        return unit;
    }

    /** @brief Advances the production strategy through a full finite sweep. */
    e::StrategyDecision CompleteSweep(e::HierarchicalCoverageStrategy& strategy) {
        e::StrategyDecision decision;
        for (unsigned i = 0; i < 80; ++i) {
            decision = strategy.Select(start, start, map, coverage,
                                       p::Mask(p::SensorKind::kRgbd), 1 + i * .001);
            if (decision.sweep_checked && !decision.sampling_limited) return decision;
        }
        throw std::runtime_error("finite candidate sweep did not complete");
    }
};

/** @brief Unknown, occupied and unreachable slots must consume the page budget. */
void InvalidCandidateSlotsAreBounded() {
    for (auto fill : {n::CellState::kUnknown, n::CellState::kOccupied}) {
        Fixture fixture(fill, 20, 20);
        auto result = e::ViewpointSampler{}.Sample(
            fixture.start, fixture.map, fixture.coverage, *fixture.evaluator,
            e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {},
            fixture.options.sampling, true);
        Require(result.limited && !result.finished && result.next_cursor == 3,
                "invalid slots escaped the page examination budget");
    }
}

/** @brief Coarse stride skips are work even when no visibility call is made. */
void StrideSkippedSlotsAreBounded() {
    Fixture fixture;
    fixture.options.sampling.spacing = 1;
    const auto result = e::ViewpointSampler{}.Sample(
        fixture.start, fixture.map, fixture.coverage, *fixture.evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {},
        fixture.options.sampling, false);
    Require(result.limited && result.next_cursor == 3,
            "stride-skipped slots escaped the page examination budget");
}

/** @brief Measured quality improvement must not repeatedly reopen unchanged work. */
void QualityImprovementPreservesSweepProgress() {
    Fixture fixture;
    const auto unit = fixture.AddRequirement();
    e::HierarchicalCoverageStrategy strategy(fixture.paths, fixture.evaluator,
                                             fixture.options);
    bool completed = false;
    for (unsigned i = 0; i < 80; ++i) {
        fixture.coverage.Confirm(unit.id, p::SensorKind::kRgbd, .1 + i * .001,
                                 1 + i * .001);
        const auto decision = strategy.Select(
            fixture.start, fixture.start, fixture.map, fixture.coverage,
            p::Mask(p::SensorKind::kRgbd), 1 + i * .001);
        if (decision.sweep_checked && !decision.sampling_limited) {
            completed = true;
            break;
        }
    }
    Require(completed, "quality-only revisions starved a finite coverage sweep");
    Require(fixture.coverage.PendingCount() == 1,
            "candidate exhaustion must not fulfill an unobserved requirement");
}

/** @brief A valid completed empty sweep is reused instead of scanning again. */
void CompletedEmptySweepIsReused() {
    Fixture fixture;
    e::HierarchicalCoverageStrategy strategy(fixture.paths, fixture.evaluator,
                                             fixture.options);
    fixture.CompleteSweep(strategy);
    const auto decision = strategy.Select(fixture.start, fixture.start, fixture.map,
        fixture.coverage, p::Mask(p::SensorKind::kRgbd), 2);
    Require(decision.sweep_checked && !decision.sampling_limited && !decision.goal,
            "completed empty sweep was replaced by another limited empty page");
    fixture.AddRequirement();
    Require(!strategy.Select(fixture.start, fixture.start, fixture.map,
        fixture.coverage, p::Mask(p::SensorKind::kRgbd), 2.01).sweep_checked,
        "new coverage work reused an obsolete completed sweep");
}

/** @brief Cancellation during reachability preparation retains the caller cursor. */
void CanceledPreparationPreservesCursor() {
    Fixture fixture;
    const auto result = e::ViewpointSampler{}.Sample(
        fixture.start, fixture.map, fixture.coverage, *fixture.evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {},
        fixture.options.sampling, true, 5, [] { return true; });
    Require(result.canceled && !result.finished && result.next_cursor == 5,
            "canceled preparation discarded the candidate cursor");
}

/** @brief Ledger requirements advance only when the remaining work domain changes. */
void RequirementsAndMeasuredRevisionsAreIndependent() {
    Fixture fixture;
    const auto unit = fixture.AddRequirement();
    const auto requirements = fixture.coverage.requirements_revision();
    const auto evidence = fixture.coverage.revision();
    fixture.coverage.Confirm(unit.id, p::SensorKind::kRgbd, .2, 1);
    Require(fixture.coverage.revision() > evidence &&
                fixture.coverage.requirements_revision() == requirements,
            "measured quality gain changed the work-domain version");
    fixture.coverage.Upsert(unit);
    Require(fixture.coverage.requirements_revision() == requirements,
            "identical requirement rewrote the work-domain version");
    auto changed = unit;
    changed.required_sensors = p::Mask(p::SensorMode::kLidarRgbd);
    fixture.coverage.Upsert(changed);
    Require(fixture.coverage.requirements_revision() > requirements,
            "new required source did not invalidate the old work domain");
    const auto before_invalidation = fixture.coverage.requirements_revision();
    fixture.coverage.Invalidate(unit.id);
    Require(fixture.coverage.requirements_revision() > before_invalidation,
            "lost geometry evidence did not reopen the work domain");
    const auto before_removal = fixture.coverage.requirements_revision();
    fixture.coverage.Remove(unit.id);
    Require(fixture.coverage.requirements_revision() > before_removal,
            "explicit requirement removal did not version the work domain");
}

/** @brief Geometry, epoch, origin, capability and lifetime changes revoke old proofs. */
void ChangedDependenciesRevokeSweepProof() {
    for (unsigned change = 0; change < 6; ++change) {
        Fixture fixture;
        e::HierarchicalCoverageStrategy strategy(fixture.paths, fixture.evaluator,
                                                 fixture.options);
        fixture.CompleteSweep(strategy);
        auto sensors = p::Mask(p::SensorKind::kRgbd);
        double now = 2;
        if (change == 0) ++fixture.map.geometry_revision;
        else if (change == 1) {
            ++fixture.map.map_epoch;
            fixture.coverage.Reset(fixture.map.map_epoch);
        } else if (change == 2) fixture.start.x += .2;
        else if (change == 3) sensors = p::Mask(p::SensorKind::kLidar);
        else if (change == 4) now = 40;
        else sensors = 0;
        const auto result = strategy.Select(fixture.start, fixture.start, fixture.map,
            fixture.coverage, sensors, now);
        Require(!result.sweep_checked, "a changed dependency reused a completed sweep proof");
    }
    Fixture fixture;
    e::HierarchicalCoverageStrategy strategy(fixture.paths, fixture.evaluator,
                                             fixture.options);
    fixture.CompleteSweep(strategy);
    const auto canceled = strategy.Select(fixture.start, fixture.start, fixture.map,
        fixture.coverage, p::Mask(p::SensorKind::kRgbd), 2, [] { return true; });
    Require(canceled.canceled && !canceled.sweep_checked,
            "cancellation published a cached whole-sweep proof");
}

/** @brief Counters describe actual work and reject unsafe stride conversion. */
void PageCountersAndStrideValidation() {
    Fixture fixture(n::CellState::kUnknown, 20, 20);
    const auto result = e::ViewpointSampler{}.Sample(
        fixture.start, fixture.map, fixture.coverage, *fixture.evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {},
        fixture.options.sampling, true);
    Require(result.examined == 3 && result.evaluated == 2,
            "examined slots and evaluated views were conflated");
    fixture.options.sampling.spacing = std::numeric_limits<double>::max();
    bool rejected = false;
    try {
        e::ViewpointSampler{}.Sample(fixture.start, fixture.map, fixture.coverage,
            *fixture.evaluator, e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {},
            fixture.options.sampling);
    } catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected, "overflowing stride was converted to a grid index");
}
}  // namespace

/** @brief Executes each production regression and reports every failure. */
int main() {
    unsigned failures = 0;
    for (const auto& test : std::vector<std::pair<const char*, std::function<void()>>>{
             {"invalid_candidate_page_budget", InvalidCandidateSlotsAreBounded},
             {"stride_candidate_page_budget", StrideSkippedSlotsAreBounded},
             {"quality_revision_sweep_progress", QualityImprovementPreservesSweepProgress},
             {"completed_empty_sweep_reuse", CompletedEmptySweepIsReused},
             {"canceled_preparation_cursor", CanceledPreparationPreservesCursor},
             {"requirements_evidence_revisions", RequirementsAndMeasuredRevisionsAreIndependent},
             {"sweep_dependency_invalidation", ChangedDependenciesRevokeSweepProof},
             {"page_counters_stride_validation", PageCountersAndStrideValidation}}) {
        try {
            test.second();
            std::cout << "PASS " << test.first << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
        }
    }
    return failures ? 1 : 0;
}
