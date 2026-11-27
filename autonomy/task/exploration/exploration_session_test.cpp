/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/exploration_session.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include "autonomy/task/exploration/integration/vista_path_provider.hpp"
#include "autonomy/task/exploration/observation/camera_visibility_model.hpp"
#include "exploration_test_support.hpp"

namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace v = autonomy::planning::planner::vista;
namespace n = autonomy::navigation;
namespace {
using namespace exploration_test;

template <typename Predicate>
void WaitFor(Fixture& fixture, double now, Predicate predicate,
             const char* message) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
        fixture.session->Tick(now);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(predicate(), message);
}

class SurfaceLifecycleGoal final : public e::ExplorationStrategy
{
public:
    explicit SurfaceLifecycleGoal(bool complete_after_goal = false)
        : complete_after_goal_(complete_after_goal) {}

    void Reset() override {
        selected_ = false;
    }

    e::StrategyDecision Select(const e::Pose2d&, const e::Pose2d&,
                               const e::MapReadView& map,
                               const e::CoverageState& coverage, p::SensorMask,
                               double,
                               const std::function<bool()>& canceled) override {
        e::StrategyDecision result;
        const e::CoverageId id{10, 5, 2, 1, e::CoverageKind::kSurface};
        if (canceled && canceled())
            return result;
        if (selected_) {
            result.grid_fallback_checked = complete_after_goal_;
            result.sweep_checked = complete_after_goal_;
            return result;
        }
        if (!coverage.units().count(id))
            return result;
        selected_ = true;
        e::ObservationGoal goal;
        goal.goal_id = 42;
        goal.goal_version = 1;
        goal.map_epoch = map.map_epoch;
        goal.coverage_revision = coverage.revision();
        goal.original_pose = {1, 1, 0};
        goal.required_views = {0};
        goal.allowed_translation = goal.allowed_yaw_change = 0;
        goal.required_coverage_units = {id};
        goal.minimum_gain = .01;
        result.goal = std::move(goal);
        return result;
    }

private:
    bool complete_after_goal_ = false;
    bool selected_ = false;
};

e::MapReadView SurfaceVolumeMap(Fixture& fixture, double stamp, bool live) {
    auto map = fixture.reader.Read(fixture.grid, stamp, 0, fixture.epoch);
    map.mapping_synchronized = true;
    map.observation_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    auto volume = std::make_shared<p::ObservedVolumeView>();
    volume->map_epoch = map.map_epoch;
    volume->geometry_revision = map.geometry_revision;
    volume->source_stamp = stamp;
    volume->frame_id = map.frame_id;
    volume->origin = {0, -.1, 0};
    volume->dimensions = {20, 20, 5};
    volume->resolution = .2;
    volume->mapping_synchronized = true;
    volume->voxels = std::make_shared<const std::vector<p::ObservedVoxel>>(
        std::vector<p::ObservedVoxel>{
            {{10, 5, 2}, p::ObservedVoxelState::kOccupied},
            {{9, 5, 2},
             live ? p::ObservedVoxelState::kFree
                  : p::ObservedVoxelState::kOccupied}});
    map.observed_volume = std::move(volume);
    return map;
}

void SaveCannotHideLocalizationLoss() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false,
                    .1);
    WaitFor(
        fixture, 1.02, [&] { return fixture.refresh_request.has_value(); },
        "completion requests a real map refresh");
    p::ObservationCut cut;
    cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    fixture.refresh_reply =
        e::MappingReply{*fixture.refresh_request, true, true, cut};
    fixture.session->Tick(1.03);
    Require(fixture.save_request.has_value(),
            "verified refresh leads to a versioned save request");
    fixture.save_reply =
        e::MappingReply{*fixture.save_request, true, true, cut};
    const auto result = fixture.session->Tick(1.2);
    Require(result.state == e::ExplorationState::kFinished &&
                result.result.saved == e::MapSaveOutcome::kSaved,
            "a valid save acknowledgement can preserve the partial map");
    Require(!result.completion_confirmed && !result.result.Success(false, true),
            "stale localization during save cannot produce full exploration "
            "success");
}

/** @brief New 3-D evidence creates work without a planar geometry revision. */
void MeasuredVolumeRefreshCreatesSurfaceRequirements() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false, 10,
                    32, e::ExplorationDriver::kAutomatic, false,
                    [](e::ExplorationSessionOptions& options) {
                        options.completion_rounds = 3;
                    });
    auto map = fixture.reader.Read(fixture.grid, 1.02, 0, fixture.epoch);
    map.mapping_synchronized = true;
    map.observation_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    auto volume = std::make_shared<p::ObservedVolumeView>();
    volume->map_epoch = map.map_epoch;
    volume->geometry_revision = map.geometry_revision;
    volume->source_stamp = 1.02;
    volume->frame_id = map.frame_id;
    volume->origin = {0, 0, 0};
    volume->dimensions = {20, 20, 5};
    volume->resolution = .2;
    volume->mapping_synchronized = true;
    volume->voxels = std::make_shared<const std::vector<p::ObservedVoxel>>(
        std::vector<p::ObservedVoxel>{
            {{5, 5, 2}, p::ObservedVoxelState::kOccupied},
            {{6, 5, 2}, p::ObservedVoxelState::kFree}});
    map.observed_volume = volume;
    Require(fixture.session->PushMap(std::move(map)),
            "same-geometry measured volume update queued");
    for (unsigned i = 0; i < 20; ++i)
        fixture.session->Tick(1.02 + i * .001);
    Require(!fixture.refresh_request,
            "measured surface work was omitted from completion admission");

    auto omitted_map =
        fixture.reader.Read(fixture.grid, 1.02, 0, fixture.epoch);
    omitted_map.mapping_synchronized = true;
    omitted_map.observation_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    auto omitted_volume = std::make_shared<p::ObservedVolumeView>(*volume);
    omitted_volume->voxels =
        std::make_shared<const std::vector<p::ObservedVoxel>>(
            std::vector<p::ObservedVoxel>{
                {{5, 5, 2}, p::ObservedVoxelState::kOccupied}});
    omitted_map.observed_volume = std::move(omitted_volume);
    Require(fixture.session->PushMap(std::move(omitted_map)),
            "same-stamp sparse omission queued");
    for (unsigned i = 0; i < 20; ++i)
        fixture.session->Tick(1.04 + i * .001);
    Require(!fixture.refresh_request,
            "sparse omission incorrectly retired measured surface work");

    auto cleared_map =
        fixture.reader.Read(fixture.grid, 1.02, 0, fixture.epoch);
    cleared_map.mapping_synchronized = true;
    cleared_map.observation_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    auto cleared_volume = std::make_shared<p::ObservedVolumeView>(*volume);
    cleared_volume->voxels =
        std::make_shared<const std::vector<p::ObservedVoxel>>(
            std::vector<p::ObservedVoxel>{
                {{5, 5, 2}, p::ObservedVoxelState::kFree}});
    cleared_map.observed_volume = std::move(cleared_volume);
    Require(fixture.session->PushMap(std::move(cleared_map)),
            "same-stamp replacement volume queued");
    WaitFor(
        fixture, 1.05, [&] { return fixture.refresh_request.has_value(); },
        "explicit source-free evidence retires the stale surface");
}

/** @brief Explicit adjacent occupancy retires an obsolete surface face. */
void AdjacentOccupiedRetiresSurfaceRequirement() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false, 10,
                    32, e::ExplorationDriver::kAutomatic, false,
                    [](e::ExplorationSessionOptions& options) {
                        options.completion_rounds = 3;
                    });
    const auto make_map = [&](std::vector<p::ObservedVoxel> voxels) {
        auto map = fixture.reader.Read(fixture.grid, 1.02, 0, fixture.epoch);
        map.mapping_synchronized = true;
        map.observation_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
        auto volume = std::make_shared<p::ObservedVolumeView>();
        volume->map_epoch = map.map_epoch;
        volume->geometry_revision = map.geometry_revision;
        volume->source_stamp = 1.02;
        volume->frame_id = map.frame_id;
        volume->dimensions = {20, 20, 5};
        volume->resolution = .2;
        volume->mapping_synchronized = true;
        volume->voxels = std::make_shared<const std::vector<p::ObservedVoxel>>(
            std::move(voxels));
        map.observed_volume = std::move(volume);
        return map;
    };
    Require(fixture.session->PushMap(
                make_map({{{5, 5, 2}, p::ObservedVoxelState::kOccupied},
                          {{6, 5, 2}, p::ObservedVoxelState::kFree}})),
            "initial measured face queued");
    for (unsigned i = 0; i < 20; ++i)
        fixture.session->Tick(1.02 + i * .001);
    Require(!fixture.refresh_request,
            "surface work disappeared before explicit contradiction");
    Require(fixture.session->PushMap(
                make_map({{{5, 5, 2}, p::ObservedVoxelState::kOccupied},
                          {{6, 5, 2}, p::ObservedVoxelState::kOccupied}})),
            "adjacent-occupied replacement queued");
    WaitFor(
        fixture, 1.05, [&] { return fixture.refresh_request.has_value(); },
        "explicit adjacent occupancy retires the stale surface");
}

void NewWorkInvalidatesOldSaveAcknowledgement() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false);
    WaitFor(
        fixture, 1.02, [&] { return fixture.refresh_request.has_value(); },
        "refresh pending");
    p::ObservationCut cut;
    cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    fixture.refresh_reply =
        e::MappingReply{*fixture.refresh_request, true, true, cut};
    fixture.session->Tick(1.03);
    const auto old = *fixture.save_request;
    e::CoverageUnit unit;
    unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
    unit.center = {2, 1, .5};
    unit.normal = {-1, 0, 0};
    unit.required_quality = .5;
    Require(fixture.session->PushCoverageUnit(unit),
            "new required coverage arrives during saving");
    fixture.save_reply = e::MappingReply{old, true, true, cut};
    const auto result = fixture.session->Tick(1.04);
    Require(!result.completion_confirmed &&
                result.result.saved != e::MapSaveOutcome::kSaved &&
                !result.result.Success(false, true),
            "late old save ACK cannot hide newer pending work");
}

void ExceptionsStayInsideSessionLifecycle() {
    {
        Fixture fixture({}, .05, std::make_shared<ThrowingStrategy>(), false,
                        false);
        WaitFor(
            fixture, 1.02,
            [&] { return !fixture.session->Snapshot().reason.empty(); },
            "non-standard strategy exceptions are delivered as task state");
        Require(fixture.session->Snapshot().reason.find("non-standard") !=
                    std::string::npos,
                "the worker reports its non-standard exception");
    }
    {
        Fixture fixture({0});
        fixture.throw_poll = fixture.throw_cancel = true;
        const auto state = fixture.session->Tick(1.03);
        Require(state.state == e::ExplorationState::kFinished &&
                    !state.result.Success(false, false) &&
                    fixture.cancellation_count == 1,
                "owner callback failure attempts cancellation and cannot "
                "report success");
    }
    {
        Fixture fixture({0});
        fixture.throw_cancel = true;
        fixture.session.reset();
        Require(fixture.cancellation_count == 1,
                "destructor contains a non-standard cancellation exception");
    }
}

void RestartAndCancellationUnderBacklog() {
    Fixture fixture({0}, .05, {}, false, true, 10, 1);
    const auto old = fixture.submissions.front().second;
    for (unsigned i = 0; i < 40; ++i)
        fixture.Pose(1.03 + i * .001, 0);
    Require(fixture.session->PostCommand(e::SessionCommand::kCancel),
            "cancel accepted under input backlog");
    Require(
        fixture.session->Tick(1.08).state == e::ExplorationState::kFinished &&
            fixture.cancellation_count == 1,
        "cancel is processed before ordinary input backlog");
    fixture.session->Start({1, 1, 0}, fixture.epoch, 2);
    Require(
        fixture.session->PushMap(fixture.reader.Read(fixture.grid, 2, 0, 1)),
        "restart map queued");
    e::CoverageUnit unit;
    unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
    unit.center = {2, 1, .5};
    unit.normal = {-1, 0, 0};
    unit.required_quality = .5;
    fixture.session->PushCoverageUnit(unit);
    fixture.Pose(2, 0);
    fixture.Observe(2, false);
    WaitFor(
        fixture, 2.01, [&] { return fixture.submissions.size() == 2; },
        "same-epoch restart resets strategy and selects again");
    fixture.reply = e::NavigationReply{old, true};
    const auto state = fixture.session->Tick(2.02);
    Require(state.session_id != old.session_id &&
                state.state != e::ExplorationState::kObserving &&
                state.active_request.has_value(),
            "old-session navigation result cannot satisfy the new request");
}

void ActualViewsAndMapUpdates() {
    Fixture fixture({0, .3});
    fixture.Arrive(1.05);
    Require(fixture.session->Snapshot().last_observed_goal_id == 0,
            "arrival alone does not confirm observations");
    fixture.grid.Set(17, 17, {n::CellState::kOccupied, 0});
    Require(
        fixture.session->PushMap(fixture.reader.Read(fixture.grid, 1.06, 0, 1)),
        "new unrelated geometry queued");
    Require(
        fixture.session->Tick(1.06).state == e::ExplorationState::kObserving,
        "valid map update keeps active observation phase");
    fixture.Pose(1.07, 0);
    fixture.Observe(1.07);
    Require(fixture.session->Tick(1.07).last_observed_goal_id == 0,
            "first required view is not the entire goal");
    fixture.session->Tick(1.13);
    Require(fixture.submissions.size() == 2,
            "same owner submits next required yaw");
    fixture.Arrive(1.14);
    const auto distance = fixture.session->Snapshot().traveled_distance;
    Require(fixture.session->PushPose({3, 3, 0}, 1.15, fixture.epoch - 1,
                                      p::Mask(p::SensorKind::kRgbd)),
            "late previous-epoch pose queued");
    const auto after_old_pose = fixture.session->Tick(1.15);
    Require(after_old_pose.state == e::ExplorationState::kObserving &&
                after_old_pose.traveled_distance == distance,
            "old epoch cannot replace pose or motion evidence");
    fixture.Pose(1.16, .3);
    fixture.Observe(1.16);
    fixture.session->Tick(1.16);
    const auto done = fixture.session->Tick(1.22);
    Require(done.last_observed_goal_id == 42 && done.observed_view_count == 2,
            "goal confirms only after actual rays at both required views and "
            "dwell");
    Require(done.acquisition_keyposes && done.acquisition_keyposes->empty(),
            "legacy observations never fabricate packet-backed acquisition "
            "keyposes");
}

void PauseDoesNotSatisfyDwell() {
    Fixture fixture({0}, .2);
    fixture.Arrive(1.05);
    fixture.Pose(1.07, 0);
    fixture.Observe(1.07);
    fixture.session->Tick(1.07);
    Require(fixture.session->PostCommand(e::SessionCommand::kPause),
            "pause queued");
    Require(fixture.session->Tick(1.08).state == e::ExplorationState::kPaused,
            "pause preserves phase");
    Require(fixture.session->PostCommand(e::SessionCommand::kResume),
            "resume queued");
    fixture.Pose(2, 0);
    fixture.Observe(2);
    auto resumed = fixture.session->Tick(2);
    Require(resumed.state == e::ExplorationState::kObserving &&
                resumed.last_observed_goal_id == 0,
            "paused wall time does not satisfy observation dwell");
    fixture.session->Tick(2.1);
    Require(fixture.session->Snapshot().last_observed_goal_id == 0,
            "unfinished view must collect its remaining dwell after resume");
    fixture.Pose(2.25, 0);
    fixture.Observe(2.25);
    Require(fixture.session->Tick(2.25).last_observed_goal_id == 42,
            "fresh post-resume observation and dwell can confirm goal");
}
/** @brief Fresh sensing and a full empty candidate sweep cannot certify an
 * unobserved or blocked robot domain. */
void EmptyDomainCannotComplete() {
    for (auto state : {n::CellState::kUnknown, n::CellState::kOccupied}) {
        Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true,
                        false);
        fixture.grid.Fill({state, 0, 1});
        auto map = fixture.reader.Read(fixture.grid, 1.02, 0, fixture.sequence,
                                       false, p::UnknownCellPolicy::kReplace);
        map.mapping_synchronized = true;
        map.observation_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
        Require(fixture.session->PushMap(map),
                "actual empty/blocked traversable-domain change queued");
        for (unsigned round = 0; round < 30; ++round) {
            const double now = 1.03 + round * .01;
            fixture.Pose(now, 0);
            fixture.Observe(now, false);
            map.source_stamp = now;
            map.observation_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
            Require(fixture.session->PushMap(map),
                    "actual current finite map/source prefix queued");
            fixture.session->Tick(now);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const auto result = fixture.session->Snapshot();
        Require(!result.completion_confirmed &&
                    result.result.exploration !=
                        e::ExplorationOutcome::kCompleted &&
                    !fixture.refresh_request && !fixture.save_request,
                "fresh observations and no candidates cannot fabricate "
                "completed coverage or a final save");
    }
}

void IntermediateLookaheadContinuesWithoutObserving() {
    auto strategy =
        std::make_shared<OneGoal>(std::vector<double>{0}, e::Pose2d{1.8, 1, 0});
    Fixture fixture({0}, .05, strategy, false, true, 10, 32,
                    e::ExplorationDriver::kAutomatic, false,
                    [](auto& options) { options.lookahead_distance_m = .2; });
    const auto first_command = fixture.submissions.front();
    Require(first_command.first.x < 1.8,
            "short lookahead submits an intermediate route waypoint");
    const auto immutable_route = fixture.session->Snapshot().goal->route;
    fixture.PushPose(first_command.first, 1.05);
    fixture.reply = e::NavigationReply{first_command.second, true};
    auto state = fixture.session->Tick(1.05);
    Require(state.state != e::ExplorationState::kObserving &&
                state.last_observed_goal_id == 0 &&
                state.observed_view_count == 0,
            "intermediate arrival never starts or confirms observation");
    WaitFor(
        fixture, 1.06, [&] { return fixture.submissions.size() > 1; },
        "intermediate arrival requests a continuation leg");
    const auto continued_snapshot = fixture.session->Snapshot();
    Require(continued_snapshot.goal.has_value(),
            "continuation retains the admitted observation goal");
    const auto& retained = continued_snapshot.goal->route;
    Require(
        retained.length == immutable_route.length &&
            retained.map_epoch == immutable_route.map_epoch &&
            retained.geometry_revision == immutable_route.geometry_revision &&
            retained.path.poses.size() == immutable_route.path.poses.size(),
        "continuation cannot replace immutable first-route evidence");
    Require(fixture.submissions.back().first.x > first_command.first.x,
            "continuation advances toward the immutable exact final pose");
}

void FinalObservationRequiresMeasuredPoseAndYaw() {
    auto strategy = std::make_shared<OneGoal>(std::vector<double>{.6},
                                              e::Pose2d{1.8, 1, .6});
    Fixture fixture({.6}, .05, strategy);
    const auto command = fixture.submissions.front();
    auto wrong_yaw = command.first;
    wrong_yaw.yaw = -.6;
    fixture.PushPose(wrong_yaw, 1.05);
    fixture.reply = e::NavigationReply{command.second, true};
    const auto state = fixture.session->Tick(1.05);
    Require(
        state.state != e::ExplorationState::kObserving &&
            state.last_observed_goal_id == 0 && state.observed_view_count == 0,
        "successful navigation without measured final yaw started observation");
}

void GeometryRevisionCancelsActiveLookahead() {
    auto strategy =
        std::make_shared<OneGoal>(std::vector<double>{0}, e::Pose2d{1.8, 1, 0});
    Fixture fixture({0}, .05, strategy, false, true, 10, 32,
                    e::ExplorationDriver::kAutomatic, false,
                    [](auto& options) { options.lookahead_distance_m = .2; });
    const auto stale = fixture.submissions.front();
    fixture.grid.Set(18, 18, {n::CellState::kOccupied, 0});
    Require(fixture.session->PushMap(
                fixture.reader.Read(fixture.grid, 1.04, 0, fixture.sequence)),
            "new geometry revision queued during active lookahead");
    auto state = fixture.session->Tick(1.04);
    Require(fixture.cancellation_count == 1 &&
                state.state != e::ExplorationState::kObserving,
            "geometry revision cancels the active execution leg");
    fixture.PushPose(stale.first, 1.05);
    fixture.reply = e::NavigationReply{stale.second, true};
    state = fixture.session->Tick(1.05);
    Require(state.state != e::ExplorationState::kObserving &&
                state.last_observed_goal_id == 0,
            "stale arrival cannot satisfy a geometry-invalidated leg");
}

void FullFirstRouteControlsAdmissionBudget() {
    auto strategy =
        std::make_shared<OneGoal>(std::vector<double>{0}, e::Pose2d{1.8, 1, 0});
    Fixture fixture(
        {0}, .05, strategy, false, false, 10, 32,
        e::ExplorationDriver::kAutomatic, false,
        [](auto& options) {
            options.lookahead_distance_m = .1;
            options.distance_budget = .3;
        },
        false);
    e::CoverageUnit unit;
    unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
    unit.center = {2, 1, .5};
    unit.normal = {-1, 0, 0};
    unit.required_quality = .5;
    Require(fixture.session->PushCoverageUnit(unit),
            "budget regression coverage queued");
    fixture.Pose(1.02, 0);
    fixture.Observe(1.02, false);
    WaitFor(
        fixture, 1.03,
        [&] {
            return fixture.session->Snapshot().result.exploration ==
                   e::ExplorationOutcome::kPartialBudget;
        },
        "full first route exceeds the remaining distance budget");
    Require(fixture.submissions.empty(),
            "short lookahead arc cannot admit a longer unaffordable route");
}

void DormantActiveSurfaceIsReleasedOnCancellation() {
    auto strategy = std::make_shared<SurfaceLifecycleGoal>();
    Fixture fixture({}, .05, strategy, false, false);
    Require(fixture.session->PushMap(SurfaceVolumeMap(fixture, 1.02, true)),
            "measured surface requirement queued");
    WaitFor(
        fixture, 1.02, [&] { return !fixture.submissions.empty(); },
        "measured surface becomes an active observation goal");
    Require(fixture.session->Snapshot().goal.has_value() &&
                fixture.session->Snapshot().pending_units == 1,
            "active surface goal owns one pending requirement");

    Require(fixture.session->PushMap(SurfaceVolumeMap(fixture, 1.03, false)),
            "surface disappearance queued while goal is active");
    const auto dormant = fixture.session->Tick(1.03);
    Require(dormant.goal.has_value() && dormant.pending_units == 1,
            "disappearing active-goal surface remains dormant only for its "
            "owner");

    Require(fixture.session->PostCommand(e::SessionCommand::kCancel),
            "session cancellation queued");
    const auto canceled = fixture.session->Tick(1.04);
    Require(!canceled.goal && canceled.pending_units == 0 &&
                canceled.state == e::ExplorationState::kFinished,
            "ReleaseGoal immediately removes dormant surface requirements");
}

void CompletedSurfaceReappearanceReopensMeasuredWork() {
    auto strategy = std::make_shared<SurfaceLifecycleGoal>(true);
    Fixture fixture({}, .05, strategy, false, false, 10, 32,
                    e::ExplorationDriver::kAutomatic, false,
                    [](e::ExplorationSessionOptions& options) {
                        options.completion_rounds = 100;
                    });
    Require(fixture.session->PushMap(SurfaceVolumeMap(fixture, 1.02, true)),
            "measured surface requirement queued");
    WaitFor(
        fixture, 1.02, [&] { return !fixture.submissions.empty(); },
        "measured surface becomes an observation goal");
    fixture.Arrive(1.05);
    fixture.Pose(1.07, 0);
    fixture.Observe(1.07);
    fixture.session->Tick(1.07);
    WaitFor(
        fixture, 1.13,
        [&] {
            const auto snapshot = fixture.session->Snapshot();
            return snapshot.last_observed_goal_id == 42 &&
                   !snapshot.goal.has_value();
        },
        "measured surface goal completes and releases ownership");
    Require(fixture.session->Snapshot().pending_units == 0,
            "surface measurement completes its requirement");

    Require(fixture.session->PushMap(SurfaceVolumeMap(fixture, 1.14, false)),
            "completed surface disappearance queued");
    Require(fixture.session->Tick(1.14).pending_units == 0,
            "disappeared completed surface leaves no pending work");
    Require(fixture.session->PushMap(SurfaceVolumeMap(fixture, 1.15, true)),
            "same measured surface reappearance queued");
    const auto reopened = fixture.session->Tick(1.15);
    Require(reopened.last_observed_goal_id == 42 &&
                reopened.pending_units == 1 && !reopened.completion_confirmed,
            "reappearing surface invalidates stale completion evidence and "
            "reopens measured work");
}

void ReturnArrivalIgnoresYawInsidePositionTolerance() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), false, false,
                    10, 32, e::ExplorationDriver::kAutomatic, true,
                    [](e::ExplorationSessionOptions& options) {
                        options.completion_rounds = 2;
                        options.position_tolerance = .1;
                        options.return_position_tolerance = .5;
                        options.yaw_tolerance = .05;
                    });
    fixture.PushPose({2, 1, 0}, 1.011);
    WaitFor(
        fixture, 1.02, [&] { return !fixture.submissions.empty(); },
        "completed exploration submits a Vista-owned return command");

    fixture.PushPose({1.49, 1, 3.0}, 1.03);
    fixture.reply = e::NavigationReply{fixture.submissions.back().second, true};
    const auto result = fixture.session->Tick(1.03);
    Require(result.state == e::ExplorationState::kFinished &&
                result.result.returned == e::ReturnOutcome::kReturned &&
                result.result.Success(true, false),
            "measured return position inside tolerance must succeed despite "
            "yaw mismatch");
}

void ReturnArrivalOutsidePositionToleranceFailsDespiteNavigationSuccess() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), false, false,
                    10, 32, e::ExplorationDriver::kAutomatic, true,
                    [](e::ExplorationSessionOptions& options) {
                        options.completion_rounds = 2;
                        options.position_tolerance = .1;
                        options.return_position_tolerance = .5;
                        options.yaw_tolerance = .05;
                    });
    fixture.PushPose({2, 1, 0}, 1.011);
    WaitFor(
        fixture, 1.02, [&] { return !fixture.submissions.empty(); },
        "completed exploration submits a Vista-owned return command");

    fixture.PushPose({1.51, 1, 0}, 1.03);
    fixture.reply = e::NavigationReply{fixture.submissions.back().second, true};
    const auto result = fixture.session->Tick(1.03);
    Require(result.state == e::ExplorationState::kFinished &&
                result.result.returned == e::ReturnOutcome::kReturnFailed &&
                !result.result.Success(true, false),
            "successful navigation outside measured return tolerance must not "
            "complete the required return");
}

}  // namespace

int main() {
    try {
        IntermediateLookaheadContinuesWithoutObserving();
        std::cout << "PASS intermediate_lookahead_continuation\n";
        FinalObservationRequiresMeasuredPoseAndYaw();
        std::cout << "PASS final_measured_pose_yaw\n";
        GeometryRevisionCancelsActiveLookahead();
        std::cout << "PASS lookahead_geometry_invalidation\n";
        FullFirstRouteControlsAdmissionBudget();
        std::cout << "PASS full_first_route_budget\n";
        DormantActiveSurfaceIsReleasedOnCancellation();
        std::cout << "PASS dormant_surface_release_on_cancel\n";
        CompletedSurfaceReappearanceReopensMeasuredWork();
        std::cout << "PASS completed_surface_reappearance\n";
        ReturnArrivalIgnoresYawInsidePositionTolerance();
        std::cout << "PASS return_position_ignores_yaw\n";
        ReturnArrivalOutsidePositionToleranceFailsDespiteNavigationSuccess();
        std::cout << "PASS return_outside_position_tolerance\n";
        ActualViewsAndMapUpdates();
        std::cout << "PASS actual_views_map_epoch\n";
        PauseDoesNotSatisfyDwell();
        std::cout << "PASS paused_observation_dwell\n";
        SaveCannotHideLocalizationLoss();
        std::cout << "PASS save_localization_loss\n";
        MeasuredVolumeRefreshCreatesSurfaceRequirements();
        std::cout << "PASS measured_volume_surface_refresh\n";
        AdjacentOccupiedRetiresSurfaceRequirement();
        std::cout << "PASS adjacent_occupied_surface_retirement\n";
        NewWorkInvalidatesOldSaveAcknowledgement();
        std::cout << "PASS new_work_old_save_ack\n";
        ExceptionsStayInsideSessionLifecycle();
        std::cout << "PASS callback_exception_boundaries\n";
        RestartAndCancellationUnderBacklog();
        std::cout << "PASS restart_cancel_backlog\n";
        EmptyDomainCannotComplete();
        std::cout << "PASS empty_domain_cannot_complete\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL session: " << error.what() << '\n';
        return 1;
    }
}
