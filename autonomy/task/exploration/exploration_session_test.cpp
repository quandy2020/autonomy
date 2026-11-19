/* Copyright 2026 The Openbot Authors. */
#include "exploration_test_support.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include "autonomy/task/exploration/exploration_session.hpp"
#include "autonomy/task/exploration/integration/vista_path_provider.hpp"
#include "autonomy/task/exploration/observation/camera_visibility_model.hpp"

namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace v = autonomy::planning::planner::vista;
namespace n = opt::navigation;
namespace {
using namespace exploration_test;

template <typename Predicate>
void WaitFor(Fixture& fixture, double now, Predicate predicate, const char* message) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
        fixture.session->Tick(now);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(predicate(), message);
}

void SaveCannotHideLocalizationLoss() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false, .1);
    WaitFor(fixture, 1.02, [&]{ return fixture.refresh_request.has_value(); },
            "completion requests a real map refresh");
    p::ObservationCut cut; cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    fixture.refresh_reply = e::MappingReply{*fixture.refresh_request, true, true, cut};
    fixture.session->Tick(1.03);
    Require(fixture.save_request.has_value(), "verified refresh leads to a versioned save request");
    fixture.save_reply = e::MappingReply{*fixture.save_request, true, true, cut};
    const auto result = fixture.session->Tick(1.2);
    Require(result.state == e::ExplorationState::kFinished && result.result.saved == e::MapSaveOutcome::kSaved,
            "a valid save acknowledgement can preserve the partial map");
    Require(!result.completion_confirmed && !result.result.Success(false, true),
            "stale localization during save cannot produce full exploration success");
}

void NewWorkInvalidatesOldSaveAcknowledgement() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false);
    WaitFor(fixture, 1.02, [&]{ return fixture.refresh_request.has_value(); }, "refresh pending");
    p::ObservationCut cut; cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    fixture.refresh_reply = e::MappingReply{*fixture.refresh_request, true, true, cut};
    fixture.session->Tick(1.03);
    const auto old = *fixture.save_request;
    e::CoverageUnit unit;
    unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
    unit.center = {2, 1, .5}; unit.normal = {-1, 0, 0}; unit.required_quality = .5;
    Require(fixture.session->PushCoverageUnit(unit), "new required coverage arrives during saving");
    fixture.save_reply = e::MappingReply{old, true, true, cut};
    const auto result = fixture.session->Tick(1.04);
    Require(!result.completion_confirmed && result.result.saved != e::MapSaveOutcome::kSaved &&
            !result.result.Success(false, true), "late old save ACK cannot hide newer pending work");
}

void ExceptionsStayInsideSessionLifecycle() {
    {
        Fixture fixture({}, .05, std::make_shared<ThrowingStrategy>(), false, false);
        WaitFor(fixture, 1.02, [&]{ return !fixture.session->Snapshot().reason.empty(); },
                "non-standard strategy exceptions are delivered as task state");
        Require(fixture.session->Snapshot().reason.find("non-standard") != std::string::npos,
                "the worker reports its non-standard exception");
    }
    {
        Fixture fixture({0});
        fixture.throw_poll = fixture.throw_cancel = true;
        const auto state = fixture.session->Tick(1.03);
        Require(state.state == e::ExplorationState::kFinished && !state.result.Success(false, false) &&
                fixture.cancellation_count == 1, "owner callback failure attempts cancellation and cannot report success");
    }
    {
        Fixture fixture({0});
        fixture.throw_cancel = true;
        fixture.session.reset();
        Require(fixture.cancellation_count == 1, "destructor contains a non-standard cancellation exception");
    }
}

void RestartAndCancellationUnderBacklog() {
    Fixture fixture({0}, .05, {}, false, true, 10, 1);
    const auto old = fixture.submissions.front().second;
    for (unsigned i = 0; i < 40; ++i)
        fixture.Pose(1.03 + i * .001, 0);
    Require(fixture.session->PostCommand(e::SessionCommand::kCancel), "cancel accepted under input backlog");
    Require(fixture.session->Tick(1.08).state == e::ExplorationState::kFinished &&
            fixture.cancellation_count == 1, "cancel is processed before ordinary input backlog");
    fixture.session->Start({1, 1, 0}, fixture.epoch, 2);
    Require(fixture.session->PushMap(fixture.reader.Read(fixture.grid, 2, 0, 1)), "restart map queued");
    e::CoverageUnit unit;
    unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
    unit.center = {2, 1, .5}; unit.normal = {-1, 0, 0}; unit.required_quality = .5;
    fixture.session->PushCoverageUnit(unit);
    fixture.Pose(2, 0); fixture.Observe(2, false);
    WaitFor(fixture, 2.01, [&]{ return fixture.submissions.size() == 2; },
            "same-epoch restart resets strategy and selects again");
    fixture.reply = e::NavigationReply{old, true};
    const auto state = fixture.session->Tick(2.02);
    Require(state.session_id != old.session_id && state.state != e::ExplorationState::kObserving &&
            state.active_request.has_value(), "old-session navigation result cannot satisfy the new request");
}

void ActualViewsAndMapUpdates() {
    Fixture fixture({0, .3});
    fixture.Arrive(1.05);
    Require(fixture.session->Snapshot().last_observed_goal_id == 0,
            "arrival alone does not confirm observations");
    fixture.grid.Set(17, 17, {n::CellState::kOccupied, 0});
    Require(fixture.session->PushMap(fixture.reader.Read(fixture.grid, 1.06, 0, 1)),
            "new unrelated geometry queued");
    Require(fixture.session->Tick(1.06).state == e::ExplorationState::kObserving,
            "valid map update keeps active observation phase");
    fixture.Pose(1.07, 0);
    fixture.Observe(1.07);
    Require(fixture.session->Tick(1.07).last_observed_goal_id == 0,
            "first required view is not the entire goal");
    fixture.session->Tick(1.13);
    Require(fixture.submissions.size() == 2, "same owner submits next required yaw");
    fixture.Arrive(1.14);
    const auto distance = fixture.session->Snapshot().traveled_distance;
    Require(fixture.session->PushPose({3, 3, 0}, 1.15, fixture.epoch - 1,
        p::Mask(p::SensorKind::kRgbd)), "late previous-epoch pose queued");
    const auto after_old_pose = fixture.session->Tick(1.15);
    Require(after_old_pose.state == e::ExplorationState::kObserving &&
        after_old_pose.traveled_distance == distance, "old epoch cannot replace pose or motion evidence");
    fixture.Pose(1.16, .3);
    fixture.Observe(1.16);
    fixture.session->Tick(1.16);
    const auto done = fixture.session->Tick(1.22);
    Require(done.last_observed_goal_id == 42 && done.observed_view_count == 2,
            "goal confirms only after actual rays at both required views and dwell");
}

void PauseDoesNotSatisfyDwell() {
    Fixture fixture({0}, .2);
    fixture.Arrive(1.05);
    fixture.Pose(1.07, 0);
    fixture.Observe(1.07);
    fixture.session->Tick(1.07);
    Require(fixture.session->PostCommand(e::SessionCommand::kPause), "pause queued");
    Require(fixture.session->Tick(1.08).state == e::ExplorationState::kPaused,
            "pause preserves phase");
    Require(fixture.session->PostCommand(e::SessionCommand::kResume), "resume queued");
    fixture.Pose(2, 0);
    fixture.Observe(2);
    auto resumed = fixture.session->Tick(2);
    Require(resumed.state == e::ExplorationState::kObserving && resumed.last_observed_goal_id == 0,
            "paused wall time does not satisfy observation dwell");
    fixture.session->Tick(2.1);
    Require(fixture.session->Snapshot().last_observed_goal_id == 0,
            "unfinished view must collect its remaining dwell after resume");
    fixture.Pose(2.25, 0);
    fixture.Observe(2.25);
    Require(fixture.session->Tick(2.25).last_observed_goal_id == 42,
            "fresh post-resume observation and dwell can confirm goal");
}
/** @brief Fresh sensing and a full empty candidate sweep cannot certify an unobserved or blocked robot domain. */
void EmptyDomainCannotComplete() {
    for (auto state : {n::CellState::kUnknown,n::CellState::kOccupied}) {
        Fixture fixture({},.05,std::make_shared<EmptyComplete>(),true,false);
        fixture.grid.Fill({state,0,1});
        auto map = fixture.reader.Read(fixture.grid,1.02,0,fixture.sequence,false,p::UnknownCellPolicy::kReplace);
        map.mapping_synchronized = true; map.observation_cut.Include(p::SensorKind::kRgbd,fixture.sequence);
        Require(fixture.session->PushMap(map),"actual empty/blocked traversable-domain change queued");
        for (unsigned round = 0; round < 30; ++round) {
            const double now = 1.03+round*.01;
            fixture.Pose(now,0); fixture.Observe(now,false);
            map.source_stamp = now; map.observation_cut.Include(p::SensorKind::kRgbd,fixture.sequence);
            Require(fixture.session->PushMap(map),"actual current finite map/source prefix queued");
            fixture.session->Tick(now);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const auto result = fixture.session->Snapshot();
        Require(!result.completion_confirmed && result.result.exploration != e::ExplorationOutcome::kCompleted &&
            !fixture.refresh_request && !fixture.save_request,
            "fresh observations and no candidates cannot fabricate completed coverage or a final save");
    }
}
}  // namespace

int main() {
    try {
        ActualViewsAndMapUpdates();
        std::cout << "PASS actual_views_map_epoch\n";
        PauseDoesNotSatisfyDwell();
        std::cout << "PASS paused_observation_dwell\n";
        SaveCannotHideLocalizationLoss();
        std::cout << "PASS save_localization_loss\n";
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
