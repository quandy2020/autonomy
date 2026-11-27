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

#include "autonomy/task/exploration/coverage/coverage_fusion.hpp"
#include "autonomy/task/exploration/coverage/local_coverage_planner.hpp"
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

class SpatialKindVisibility final : public e::SensorVisibilityModel
{
public:
    p::SensorKind sensor() const override {
        return p::SensorKind::kRgbd;
    }
    std::optional<e::CoverageContribution> Predict(
        const e::Pose2d& viewpoint, const e::CoverageUnit& unit,
        const e::MapReadView&, const e::ObservationQuality&) const override {
        const bool local_view = viewpoint.x < .6;
        if (local_view != (unit.id.x == 1))
            return std::nullopt;
        return e::CoverageContribution{unit.id, sensor(),
                                       local_view ? 1.0 : .15};
    }
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
    fixture.options.sampling.allowed_region = p::Bounds2d{0, 0, 1.21, .81};
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

/** @brief Candidate body clearance follows each measured cell floor height. */
void MeasuredTerrainHeightOffsetsBodyClearance() {
    Fixture fixture;
    fixture.AddRequirement();
    auto sensor = std::make_shared<CountingVisibility>();
    e::ViewpointEvaluator evaluator(
        std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{sensor});
    auto sampling = fixture.options.sampling;
    sampling.yaw_samples = 1;
    sampling.maximum_examined = 1000;
    sampling.maximum_evaluations = 1000;
    n::SafetyOptions body;
    body.robot_radius = .01;
    body.horizontal_margin = 0;
    body.robot_height = .6;
    body.vertical_margin = 0;

    e::RollingViewpointFieldOptions field_options;
    field_options.dimensions = {6, 4, 8};
    field_options.rollover_step = {2, 2, 2};
    field_options.maximum_cells = 192;
    field_options.maximum_input_voxels = 192;
    e::RollingViewpointField field(field_options);
    std::vector<p::ObservedVoxel> voxels;
    for (int z = 0; z < 8; ++z)
        for (int y = 0; y < 4; ++y)
            for (int x = 0; x < 6; ++x)
                voxels.push_back({{x, y, z},
                                  z == 1 ? p::ObservedVoxelState::kOccupied
                                         : p::ObservedVoxelState::kFree});
    p::ObservedVolumeView volume;
    volume.map_epoch = fixture.map.map_epoch;
    volume.geometry_revision = fixture.map.geometry_revision;
    volume.source_stamp = 1;
    volume.frame_id = fixture.map.frame_id;
    volume.dimensions = {6, 4, 8};
    volume.resolution = .2;
    volume.mapping_synchronized = true;
    volume.voxels = std::make_shared<const std::vector<p::ObservedVoxel>>(
        std::move(voxels));
    Require(
        field.Update(volume, {fixture.start.x, fixture.start.y, .8}).status ==
            e::ViewpointFieldUpdateStatus::kPublished,
        "measured terrain fixture did not publish its 3-D field");

    const auto low = e::ViewpointSampler{}.Sample(
        fixture.start, fixture.map, fixture.coverage, evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), body, sampling, true,
        0, {}, field.Read());
    Require(low.candidates.empty() && low.evaluated == 0,
            "low measured floors ignored the occupied body-height plane");

    n::GridMap elevated_grid(6, 4, .2);
    elevated_grid.Fill({n::CellState::kFree, .8});
    e::MapReader reader;
    const auto elevated = reader.Read(elevated_grid, fixture.map.map_epoch, 0,
                                      fixture.map.geometry_revision, true);
    sensor->calls = 0;
    const auto high = e::ViewpointSampler{}.Sample(
        fixture.start, elevated, fixture.coverage, evaluator,
        e::RegionManager{}, p::Mask(p::SensorKind::kRgbd), body, sampling, true,
        0, {}, field.Read());
    Require(
        !high.candidates.empty() && high.evaluated > 0 && sensor->calls > 0,
        "measured elevated floors were not used as the body-clearance datum");
}

/** @brief Measured candidate elevation changes sensor visibility and gain. */
void MeasuredTerrainHeightOffsetsSensorVisibility() {
    n::GridMap low_grid(6, 4, .2);
    low_grid.Fill({n::CellState::kFree, 0});
    n::GridMap elevated_grid(6, 4, .2);
    elevated_grid.Fill({n::CellState::kFree, 1});
    e::MapReader reader;
    const auto low = reader.Read(low_grid, 1, 0, 1, true);
    const auto elevated = reader.Read(elevated_grid, 1, 0, 1, true);

    e::CoverageState coverage;
    coverage.Reset(low.map_epoch);
    e::CoverageUnit unit;
    unit.id = {4, 0, 0, 0, e::CoverageKind::kSurface};
    unit.center = {.9, .1, .5};
    unit.normal = {-1, 0, 0};
    unit.required_quality = 1;
    unit.required_sensors = p::Mask(p::SensorKind::kRgbd);
    coverage.Upsert(unit);

    p::RigidTransform base_from_camera;
    base_from_camera.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    base_from_camera.translation.z = .5;
    auto camera = std::make_shared<e::CameraVisibilityModel>(
        p::CameraModel{100, 100, 50, 50, 50, 50, .1, 8}, base_from_camera);
    e::ViewpointEvaluator evaluator(
        std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{camera});
    const e::Pose2d pose{.1, .1, 0};

    const auto low_prediction =
        evaluator.Predict(pose, coverage, low, p::Mask(p::SensorKind::kRgbd));
    const auto elevated_prediction = evaluator.Predict(
        pose, coverage, elevated, p::Mask(p::SensorKind::kRgbd));
    Require(low_prediction.size() == 1 &&
                e::CoverageFusion::Gain(coverage, low_prediction) > 0,
            "floor-zero candidate lost visible measured coverage gain");
    Require(elevated_prediction.empty() &&
                e::CoverageFusion::Gain(coverage, elevated_prediction) == 0,
            "elevated candidate predicted gain from a floor-zero sensor pose");
}

/** @brief Remote surface/frontier thresholds are independent and local-safe. */
void RemoteIncrementThresholdsAreKindSpecific() {
    const auto select = [](e::CoverageKind remote_kind,
                           double surface_threshold,
                           double frontier_threshold) {
        Fixture fixture(n::CellState::kFree, 8, 8);
        auto visibility = std::make_shared<SpatialKindVisibility>();
        fixture.evaluator = std::make_shared<e::ViewpointEvaluator>(
            std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{
                visibility});
        fixture.options.region_size = .6;
        fixture.options.local_radius = .15;
        fixture.options.use_frontier_points = true;
        fixture.options.fairness_interval = 1e31;
        fixture.options.minimum_incremental_surface_gain = surface_threshold;
        fixture.options.minimum_incremental_frontier_gain = frontier_threshold;
        fixture.options.safety.robot_radius = .01;
        fixture.options.sampling.spacing = .6;
        fixture.options.sampling.yaw_samples = 1;
        fixture.options.sampling.maximum_evaluations = 1000;
        fixture.options.sampling.maximum_examined = 1000;

        e::CoverageUnit local;
        local.id = {1, 0, 0, 0, e::CoverageKind::kSurface};
        local.center = {.1, .1, 0};
        local.required_quality = 1;
        fixture.coverage.Upsert(local);
        e::CoverageUnit remote;
        remote.id = {2, 0, 0, 0, remote_kind};
        remote.center = {.7, .7, 0};
        remote.required_quality = 1;
        fixture.coverage.Upsert(remote);
        e::HierarchicalCoverageStrategy strategy(
            fixture.paths, fixture.evaluator, fixture.options);
        const auto first =
            strategy.Select(fixture.start, fixture.start, fixture.map,
                            fixture.coverage, p::Mask(p::SensorKind::kRgbd), 1);
        Require(
            first.goal && !first.global_route.views.empty(),
            "one low-value pass retired remote work or suppressed local work");
        return strategy.Select(fixture.start, fixture.start, fixture.map,
                               fixture.coverage, p::Mask(p::SensorKind::kRgbd),
                               2);
    };

    const auto low_surface = select(e::CoverageKind::kSurface, .2, .1);
    Require(
        low_surface.global_route.views.empty() && low_surface.goal,
        "low surface gain admitted remote relocation or suppressed local work");
    const auto admitted_surface = select(e::CoverageKind::kSurface, .1, .2);
    Require(!admitted_surface.global_route.views.empty(),
            "surface gain used the frontier threshold");
    const auto admitted_frontier = select(e::CoverageKind::kFrontier, .2, .1);
    Require(!admitted_frontier.global_route.views.empty(),
            "frontier gain used the surface threshold");
    const auto low_frontier = select(e::CoverageKind::kFrontier, .1, .2);
    Require(low_frontier.global_route.views.empty() && low_frontier.goal,
            "low frontier gain admitted relocation or suppressed local work");
}

/** @brief Region hysteresis gates remote representatives but never local
 * work.
 */
void RegionStatusGatesOnlyRemoteRepresentatives() {
    Fixture fixture(n::CellState::kFree, 8, 8);
    auto visibility = std::make_shared<CountingVisibility>();
    fixture.evaluator = std::make_shared<e::ViewpointEvaluator>(
        std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{
            visibility});
    fixture.options.region_size = .6;
    fixture.options.local_radius = .15;
    fixture.options.safety.robot_radius = .01;
    fixture.options.sampling.spacing = .6;
    fixture.options.sampling.yaw_samples = 1;
    fixture.options.sampling.maximum_evaluations = 1000;
    fixture.options.sampling.maximum_examined = 1000;

    e::CoverageUnit local;
    local.id = {1, 0, 0, 0, e::CoverageKind::kSurface};
    local.center = {.1, .1, 0};
    local.required_quality = 1;
    fixture.coverage.Upsert(local);
    e::CoverageUnit remote;
    remote.id = {2, 0, 0, 0, e::CoverageKind::kSurface};
    remote.center = {.7, .7, 0};
    remote.required_quality = 1;
    fixture.coverage.Upsert(remote);

    e::HierarchicalCoverageStrategy strategy(fixture.paths, fixture.evaluator,
                                             fixture.options);
    const auto initial =
        strategy.Select(fixture.start, fixture.start, fixture.map,
                        fixture.coverage, p::Mask(p::SensorKind::kRgbd), 1);
    Require(!initial.global_route.views.empty() && initial.goal,
            "measured remote and local work were not both admitted");

    fixture.coverage.Confirm(remote.id, p::SensorKind::kRgbd, 1, 2);
    const auto first_empty =
        strategy.Select(fixture.start, fixture.start, fixture.map,
                        fixture.coverage, p::Mask(p::SensorKind::kRgbd), 2);
    Require(!first_empty.global_route.views.empty(),
            "one empty frame retired a feasible remote representative");
    Require(first_empty.goal.has_value(),
            "remote-region debounce suppressed measured local candidates");

    const auto second_empty =
        strategy.Select(fixture.start, fixture.start, fixture.map,
                        fixture.coverage, p::Mask(p::SensorKind::kRgbd), 3);
    Require(second_empty.global_route.views.empty(),
            "two empty frames retained a covered remote region");
    Require(second_empty.goal.has_value(),
            "covered remote status suppressed measured local work");
}

/** @brief Creates one pending requirement with normalized unit gain. */
e::CoverageId AddRequirement(e::CoverageState& state, int x,
                             e::CoverageKind kind) {
    e::CoverageUnit unit;
    unit.id = {x, 0, 0, 0, kind};
    unit.center = {static_cast<double>(x), 0, 0};
    unit.required_quality = 1;
    state.Upsert(unit);
    return unit.id;
}

/** @brief Creates one bounded-cost view contributing to exactly one unit. */
e::Viewpoint MakeView(std::uint64_t id, const e::CoverageId& unit,
                      double quality) {
    e::Viewpoint viewpoint;
    viewpoint.id = id;
    viewpoint.observation_time = 1;
    viewpoint.contributions.push_back({unit, p::SensorKind::kRgbd, quality});
    return viewpoint;
}

/** @brief Attempt seeds diversify non-tied candidates inside the near-best set.
 */
void LocalSelectionAttemptsDiversifyNearBestViews() {
    e::CoverageState state;
    state.Reset(1);
    const auto strongest = AddRequirement(state, 1, e::CoverageKind::kSurface);
    const auto near = AddRequirement(state, 2, e::CoverageKind::kSurface);
    const std::vector<e::Viewpoint> candidates{MakeView(10, strongest, 1),
                                               MakeView(20, near, .95)};

    e::LocalCoverageSelectionOptions options;
    options.diversification_ratio = .9;
    options.diversification_seed = 0;
    const auto first =
        e::LocalCoveragePlanner{}.Select(candidates, state, 1, {}, options);
    options.diversification_seed = 1;
    const auto second =
        e::LocalCoveragePlanner{}.Select(candidates, state, 1, {}, options);
    Require(first.size() == 1 && second.size() == 1,
            "near-best attempts failed to select a view");
    Require(first.front().id != second.front().id,
            "attempt seeds only changed tie handling, not the cover set");
}

/** @brief Surface/frontier thresholds are independent; mandatory work is
 * forced. */
void LocalIncrementThresholdsAreKindSpecific() {
    e::CoverageState state;
    state.Reset(1);
    const auto surface = AddRequirement(state, 1, e::CoverageKind::kSurface);
    const auto frontier = AddRequirement(state, 2, e::CoverageKind::kFrontier);
    const std::vector<e::Viewpoint> candidates{MakeView(10, surface, .15),
                                               MakeView(20, frontier, .15)};

    e::LocalCoverageSelectionOptions options;
    options.minimum_incremental_surface_gain = .2;
    options.minimum_incremental_frontier_gain = .1;
    const auto thresholded =
        e::LocalCoveragePlanner{}.Select(candidates, state, 2, {}, options);
    Require(thresholded.size() == 1 && thresholded.front().id == 20,
            "surface threshold incorrectly filtered an admissible frontier");

    options.minimum_incremental_surface_gain = .1;
    options.minimum_incremental_frontier_gain = .2;
    const auto reversed =
        e::LocalCoveragePlanner{}.Select(candidates, state, 2, {}, options);
    Require(reversed.size() == 1 && reversed.front().id == 10,
            "frontier threshold incorrectly filtered an admissible surface");

    options.minimum_incremental_surface_gain = .2;
    options.minimum_incremental_frontier_gain = .1;
    const auto mandatory = e::LocalCoveragePlanner{}.Select(
        candidates, state, 1, surface, options);
    Require(mandatory.size() == 1 && mandatory.front().id == 10,
            "a mandatory contributor was filtered by an incremental threshold");
}

/** @brief A prior local view is retained while it still adds useful coverage.
 */
void UsefulPriorLocalSelectionIsRetained() {
    e::CoverageState state;
    state.Reset(1);
    const auto prior_unit = AddRequirement(state, 1, e::CoverageKind::kSurface);
    const auto replacement_unit =
        AddRequirement(state, 2, e::CoverageKind::kSurface);
    const e::Viewpoint prior = MakeView(10, prior_unit, .5);
    const e::Viewpoint replacement = MakeView(20, replacement_unit, 1);

    e::LocalCoverageSelectionOptions options;
    options.minimum_incremental_surface_gain = .1;
    const auto selected = e::LocalCoveragePlanner{}.Select(
        {prior, replacement}, state, 1, {}, options, {prior});
    Require(selected.size() == 1 && selected.front().id == prior.id,
            "a still-useful prior view was replaced before route ordering");
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
             {"measured_terrain_height_clearance",
              MeasuredTerrainHeightOffsetsBodyClearance},
             {"measured_terrain_height_visibility",
              MeasuredTerrainHeightOffsetsSensorVisibility},
             {"remote_increment_thresholds_by_kind",
              RemoteIncrementThresholdsAreKindSpecific},
             {"region_status_remote_representatives",
              RegionStatusGatesOnlyRemoteRepresentatives},
             {"local_attempts_diversify_near_best",
              LocalSelectionAttemptsDiversifyNearBestViews},
             {"local_increment_thresholds_by_kind",
              LocalIncrementThresholdsAreKindSpecific},
             {"useful_prior_local_selection_retained",
              UsefulPriorLocalSelectionIsRetained},
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
