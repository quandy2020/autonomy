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
#include "autonomy/task/exploration/polygon_boundary.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace n = autonomy::navigation;
namespace v = autonomy::planning::planner::vista;

/** @brief Fails a regression without disabling checks in release builds. */
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

class CountingVisibility final : public e::SensorVisibilityModel
{
public:
    p::SensorKind sensor() const override {
        return p::SensorKind::kRgbd;
    }
    std::optional<e::CoverageContribution> Predict(
        const e::Pose2d&, const e::CoverageUnit& unit, const e::MapReadView&,
        const e::ObservationQuality&) const override {
        ++calls;
        return e::CoverageContribution{unit.id, sensor(), 1.0};
    }
    mutable std::size_t calls = 0;
};

class PagedVisibility final : public e::SensorVisibilityModel
{
public:
    p::SensorKind sensor() const override {
        return p::SensorKind::kRgbd;
    }
    std::optional<e::CoverageContribution> Predict(
        const e::Pose2d& viewpoint, const e::CoverageUnit& unit,
        const e::MapReadView&, const e::ObservationQuality&) const override {
        const int expected_x = viewpoint.x < 1 ? 1 : 2;
        if (unit.id.x != expected_x)
            return std::nullopt;
        return e::CoverageContribution{unit.id, sensor(), 1.0};
    }
};

/** @brief Real geometry, camera model and route provider for sampler
 * regressions. */
struct Fixture {
    e::MapReadView map;          ///< Owned immutable geometry for this test.
    e::CoverageState coverage;   ///< Measured requirement ledger.
    e::Pose2d start{.1, .1, 0};  ///< Known free initial cell center.
    std::shared_ptr<e::ViewpointEvaluator>
        evaluator;  ///< Calibrated camera predictor.
    std::shared_ptr<e::VistaPathProvider>
        paths;  ///< Real visibility-graph routing.
    e::HierarchicalCoverageOptions
        options;  ///< Small page limits expose resumption.

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
            std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{
                camera});
        auto planner = std::make_shared<v::VistaRoutePlanner>();
        planner->Update(map);
        paths = std::make_shared<e::VistaPathProvider>(planner);
        options.sampling.yaw_samples = 2;
        options.sampling.maximum_evaluations = 3;
        options.sampling.maximum_examined = 3;
    }

    /** @brief Adds a real requirement outside the fixture camera's visible
     * domain. */
    e::CoverageUnit AddRequirement(int x = 100) {
        e::CoverageUnit unit;
        unit.id = {x, 0, 0, 0, e::CoverageKind::kSurface};
        unit.center = {20, 0, 0};
        unit.required_quality = 1;
        coverage.Upsert(unit);
        return unit;
    }

    /** @brief Advances the production strategy through a full finite sweep. */
    e::StrategyDecision CompleteSweep(
        e::HierarchicalCoverageStrategy& strategy) {
        e::StrategyDecision decision;
        for (unsigned i = 0; i < 80; ++i) {
            decision =
                strategy.Select(start, start, map, coverage,
                                p::Mask(p::SensorKind::kRgbd), 1 + i * .001);
            if (decision.sweep_checked && !decision.sampling_limited)
                return decision;
        }
        throw std::runtime_error("finite candidate sweep did not complete");
    }
};

/** @brief Unknown, occupied and unreachable slots must consume the page budget.
 */
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

/** @brief Sampling never evaluates a viewpoint outside the task boundary. */
void TaskBoundaryConstrainsSamplingDomain() {
    Fixture fixture;
    fixture.options.sampling.maximum_evaluations = 100;
    fixture.options.sampling.maximum_examined = 100;
    fixture.options.sampling.allowed_region = p::Bounds2d{0, 0, .2, .2};
    const auto result = e::ViewpointSampler{}.Sample(
        fixture.start, fixture.map, fixture.coverage, *fixture.evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {},
        fixture.options.sampling, true);
    Require(
        result.finished &&
            result.evaluated == fixture.options.sampling.yaw_samples,
        "viewpoints outside the task boundary reached visibility evaluation");

    fixture.options.sampling.allowed_region = p::Bounds2d{1, 0, 0, 1};
    bool rejected = false;
    try {
        e::ViewpointSampler{}.Sample(
            fixture.start, fixture.map, fixture.coverage, *fixture.evaluator,
            e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {},
            fixture.options.sampling, true);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Require(rejected, "an inverted task boundary was accepted");
}

/** @brief Concave runtime no-go polygons exclude edges and interiors. */
void ConcaveNoGoConstrainsSamplingDomain() {
    Fixture fixture;
    auto visibility = std::make_shared<CountingVisibility>();
    fixture.evaluator = std::make_shared<e::ViewpointEvaluator>(
        std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{
            visibility});
    fixture.AddRequirement();
    fixture.options.sampling.maximum_evaluations = 1000;
    fixture.options.sampling.maximum_examined = 1000;
    ::automsgs::msgs::geometry_msgs::Polygon no_go;
    for (const auto [x, y] : {std::pair<float, float>{.1F, .1F},
                              {.7F, .1F},
                              {.7F, .5F},
                              {.5F, .5F},
                              {.5F, .3F},
                              {.1F, .3F}}) {
        auto* point = no_go.add_points();
        point->set_x(x);
        point->set_y(y);
    }
    fixture.options.sampling.no_go_areas.push_back(no_go);
    n::SafetyOptions safety;
    safety.robot_radius = .01;
    const auto result = e::ViewpointSampler{}.Sample(
        fixture.start, fixture.map, fixture.coverage, *fixture.evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), safety,
        fixture.options.sampling, true);
    Require(!result.candidates.empty(),
            "concave no-go removed every valid viewpoint");
    for (const auto& candidate : result.candidates) {
        Require(
            e::PolygonBoundaryOutsideAll(fixture.options.sampling.no_go_areas,
                                         candidate.pose.x, candidate.pose.y),
            "viewpoint on or inside a concave no-go was evaluated");
    }
}

/** @brief Concave polygon cutouts exclude candidates inside broad-phase bounds.
 */
void ConcaveTaskPolygonConstrainsSamplingDomain() {
    Fixture fixture;
    auto visibility = std::make_shared<CountingVisibility>();
    fixture.evaluator = std::make_shared<e::ViewpointEvaluator>(
        std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{
            visibility});
    fixture.AddRequirement();
    fixture.options.sampling.maximum_evaluations = 1000;
    fixture.options.sampling.maximum_examined = 1000;
    fixture.options.sampling.allowed_region = p::Bounds2d{0, 0, 1.2, .8};
    ::automsgs::msgs::geometry_msgs::Polygon polygon;
    for (const auto [x, y] : {std::pair<float, float>{0, 0},
                              {1.2F, 0},
                              {1.2F, .4F},
                              {.6F, .4F},
                              {.6F, .8F},
                              {0, .8F}}) {
        auto* point = polygon.add_points();
        point->set_x(x);
        point->set_y(y);
    }
    fixture.options.sampling.allowed_polygon = polygon;
    n::SafetyOptions safety;
    safety.robot_radius = .01;
    const auto result = e::ViewpointSampler{}.Sample(
        fixture.start, fixture.map, fixture.coverage, *fixture.evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), safety,
        fixture.options.sampling, true);
    Require(!result.candidates.empty(),
            "concave polygon removed every valid viewpoint");
    for (const auto& candidate : result.candidates) {
        Require(!(candidate.pose.x > .6 && candidate.pose.y > .4),
                "viewpoint in concave cutout escaped exact polygon filtering");
    }
}

/** @brief Measured quality improvement must not repeatedly reopen unchanged
 * work. */
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
    Require(completed,
            "quality-only revisions starved a finite coverage sweep");
    Require(fixture.coverage.PendingCount() == 1,
            "candidate exhaustion must not fulfill an unobserved requirement");
}

/** @brief A valid completed empty sweep is reused instead of scanning again. */
void CompletedEmptySweepIsReused() {
    Fixture fixture;
    e::HierarchicalCoverageStrategy strategy(fixture.paths, fixture.evaluator,
                                             fixture.options);
    fixture.CompleteSweep(strategy);
    const auto decision =
        strategy.Select(fixture.start, fixture.start, fixture.map,
                        fixture.coverage, p::Mask(p::SensorKind::kRgbd), 2);
    Require(
        decision.sweep_checked && !decision.sampling_limited && !decision.goal,
        "completed empty sweep was replaced by another limited empty page");
    fixture.AddRequirement();
    Require(!strategy
                 .Select(fixture.start, fixture.start, fixture.map,
                         fixture.coverage, p::Mask(p::SensorKind::kRgbd), 2.01)
                 .sweep_checked,
            "new coverage work reused an obsolete completed sweep");
}

/** @brief Cancellation during reachability preparation retains the caller
 * cursor. */
void CanceledPreparationPreservesCursor() {
    Fixture fixture;
    const auto result = e::ViewpointSampler{}.Sample(
        fixture.start, fixture.map, fixture.coverage, *fixture.evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {},
        fixture.options.sampling, true, 5, [] { return true; });
    Require(result.canceled && !result.finished && result.next_cursor == 5,
            "canceled preparation discarded the candidate cursor");
}

/** @brief Ledger requirements advance only when the remaining work domain
 * changes. */
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

/** @brief Geometry, epoch, origin, capability and lifetime changes revoke old
 * proofs. */
void ChangedDependenciesRevokeSweepProof() {
    for (unsigned change = 0; change < 6; ++change) {
        Fixture fixture;
        e::HierarchicalCoverageStrategy strategy(
            fixture.paths, fixture.evaluator, fixture.options);
        fixture.CompleteSweep(strategy);
        auto sensors = p::Mask(p::SensorKind::kRgbd);
        double now = 2;
        if (change == 0)
            ++fixture.map.geometry_revision;
        else if (change == 1) {
            ++fixture.map.map_epoch;
            fixture.coverage.Reset(fixture.map.map_epoch);
        } else if (change == 2)
            fixture.start.x += .2;
        else if (change == 3)
            sensors = p::Mask(p::SensorKind::kLidar);
        else if (change == 4)
            now = 40;
        else
            sensors = 0;
        const auto result =
            strategy.Select(fixture.start, fixture.start, fixture.map,
                            fixture.coverage, sensors, now);
        Require(!result.sweep_checked,
                "a changed dependency reused a completed sweep proof");
    }
    Fixture fixture;
    e::HierarchicalCoverageStrategy strategy(fixture.paths, fixture.evaluator,
                                             fixture.options);
    fixture.CompleteSweep(strategy);
    const auto canceled = strategy.Select(
        fixture.start, fixture.start, fixture.map, fixture.coverage,
        p::Mask(p::SensorKind::kRgbd), 2, [] { return true; });
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
        e::ViewpointSampler{}.Sample(
            fixture.start, fixture.map, fixture.coverage, *fixture.evaluator,
            e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {},
            fixture.options.sampling);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Require(rejected, "overflowing stride was converted to a grid index");
}

/** @brief Reachable dense views survive while bounded sampling advances pages.
 */
void LocalViewsPersistAcrossSamplingPages() {
    Fixture fixture(n::CellState::kFree, 10, 6);
    fixture.options.sampling.maximum_evaluations = 2;
    fixture.options.sampling.maximum_examined = 2;
    fixture.options.fairness_interval = 1e6;
    fixture.options.local_views = 8;
    for (int x : {1, 2}) {
        e::CoverageUnit unit;
        unit.id = {x, 0, 0, 0, e::CoverageKind::kSurface};
        unit.center = {x == 1 ? .5 : 1.5, .6, 0};
        unit.required_quality = 1;
        fixture.coverage.Upsert(unit);
    }
    fixture.evaluator = std::make_shared<e::ViewpointEvaluator>(
        std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{
            std::make_shared<PagedVisibility>()});
    e::HierarchicalCoverageStrategy strategy(fixture.paths, fixture.evaluator,
                                             fixture.options);
    bool saw_near = false;
    bool saw_both = false;
    for (unsigned page = 0; page < 100 && !saw_both; ++page) {
        const auto decision = strategy.Select(
            fixture.start, fixture.start, fixture.map, fixture.coverage,
            p::Mask(p::SensorKind::kRgbd), 1 + page * .001);
        bool near = false;
        bool far = false;
        for (const auto& view : decision.local_route.views) {
            for (const auto& contribution : view.contributions) {
                near = near || contribution.id.x == 1;
                far = far || contribution.id.x == 2;
            }
        }
        saw_near = saw_near || near;
        if (saw_near && far) {
            Require(near,
                    "a prior reachable local view was lost on a later page");
            saw_both = true;
        }
    }
    Require(saw_both,
            "bounded sampling never combined retained and newly sampled views");
}

/** @brief Sparse measured volumes gate candidates before visibility prediction.
 */
void UnknownMeasuredVolumeFailsClosedInSampler() {
    Fixture fixture;
    e::CoverageUnit unit;
    unit.id = {2, 2, 0, 0, e::CoverageKind::kSurface};
    unit.center = {.5, .5, 0};
    unit.required_quality = 1;
    fixture.coverage.Upsert(unit);
    auto sensor = std::make_shared<CountingVisibility>();
    e::ViewpointEvaluator evaluator(
        std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{sensor});
    auto sampling = fixture.options.sampling;
    sampling.maximum_examined = 1000;
    sampling.maximum_evaluations = 1000;

    const auto ungated = e::ViewpointSampler{}.Sample(
        fixture.start, fixture.map, fixture.coverage, evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {}, sampling, true);
    Require(ungated.evaluated > 0 && sensor->calls > 0,
            "the visibility test fixture evaluated no safe planar candidate");

    e::RollingViewpointFieldOptions field_options;
    field_options.dimensions = {6, 4, 4};
    field_options.rollover_step = {2, 2, 2};
    field_options.maximum_cells = 96;
    field_options.maximum_input_voxels = 96;
    e::RollingViewpointField field(field_options);
    p::ObservedVolumeView volume;
    volume.map_epoch = fixture.map.map_epoch;
    volume.geometry_revision = fixture.map.geometry_revision;
    volume.source_stamp = 1;
    volume.frame_id = fixture.map.frame_id;
    volume.dimensions = {6, 4, 4};
    volume.resolution = .2;
    volume.mapping_synchronized = true;
    volume.voxels = std::make_shared<const std::vector<p::ObservedVoxel>>();
    const auto update =
        field.Update(volume, {fixture.start.x, fixture.start.y, 0});
    Require(update.status == e::ViewpointFieldUpdateStatus::kPublished,
            "the sparse complete measured-volume snapshot was rejected");

    sensor->calls = 0;
    const auto gated = e::ViewpointSampler{}.Sample(
        fixture.start, fixture.map, fixture.coverage, evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), {}, sampling, true,
        0, {}, field.Read());
    Require(
        gated.candidates.empty() && gated.evaluated == 0 && sensor->calls == 0,
        "omitted measured voxels were interpreted as free candidates");
}

void ExcessiveLocalRouteVariantsAreRejected() {
    Fixture fixture;
    fixture.options.local_views = 32;
    fixture.options.local_selection_attempts = 9;
    bool threw = false;
    try {
        e::HierarchicalCoverageStrategy strategy(
            fixture.paths, fixture.evaluator, fixture.options);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    Require(
        threw,
        "unbounded independently optimized local route variants were accepted");
}
}  // namespace

/** @brief Executes each production regression and reports every failure. */
int main() {
    unsigned failures = 0;
    for (const auto& test :
         std::vector<std::pair<const char*, std::function<void()>>>{
             {"invalid_candidate_page_budget", InvalidCandidateSlotsAreBounded},
             {"stride_candidate_page_budget", StrideSkippedSlotsAreBounded},
             {"task_boundary_sampling_domain",
              TaskBoundaryConstrainsSamplingDomain},
             {"concave_task_polygon_sampling_domain",
              ConcaveTaskPolygonConstrainsSamplingDomain},
             {"concave_no_go_sampling_domain",
              ConcaveNoGoConstrainsSamplingDomain},
             {"quality_revision_sweep_progress",
              QualityImprovementPreservesSweepProgress},
             {"completed_empty_sweep_reuse", CompletedEmptySweepIsReused},
             {"canceled_preparation_cursor",
              CanceledPreparationPreservesCursor},
             {"requirements_evidence_revisions",
              RequirementsAndMeasuredRevisionsAreIndependent},
             {"sweep_dependency_invalidation",
              ChangedDependenciesRevokeSweepProof},
             {"page_counters_stride_validation",
              PageCountersAndStrideValidation},
             {"local_views_persist_across_pages",
              LocalViewsPersistAcrossSamplingPages},
             {"unknown_measured_volume_fails_closed",
              UnknownMeasuredVolumeFailsClosedInSampler},
             {"bounded_local_route_variants",
              ExcessiveLocalRouteVariantsAreRejected}}) {
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
