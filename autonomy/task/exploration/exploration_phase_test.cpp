/* Copyright 2026 The Openbot Authors. */
#include <iostream>
#include "exploration_test_support.hpp"

namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
using namespace exploration_test;
namespace {
e::PhaseTicket Request(Fixture& fixture, e::ExplorationPhase phase,
                       double now) {
    auto ticket = fixture.session->RequestPhase(phase, now);
    Require(ticket.has_value(), "owner obtains a unique phase ticket");
    return *ticket;
}
e::PhaseResult Await(Fixture& fixture, const e::PhaseTicket& ticket,
                     double now) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (fixture.session->PhaseState(ticket).status ==
               e::PhaseStatus::kPending &&
           std::chrono::steady_clock::now() < deadline) {
        fixture.session->Tick(now);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    auto result = fixture.session->PhaseState(ticket);
    Require(result.status != e::PhaseStatus::kPending,
            "phase completes within test deadline");
    return result;
}
void Prepare(Fixture& fixture) {
    auto selected = Request(fixture, e::ExplorationPhase::kSelect, 1.02);
    Require(Await(fixture, selected, 1.02).status == e::PhaseStatus::kSucceeded,
            "selection succeeds independently");
    Require(fixture.session->Snapshot().selection.has_value() &&
                !fixture.session->Snapshot().goal &&
                fixture.submissions.empty(),
            "selection does not adapt or submit navigation");
    auto adapted = Request(fixture, e::ExplorationPhase::kAdapt, 1.03);
    Require(Await(fixture, adapted, 1.03).status == e::PhaseStatus::kSucceeded,
            "Opt adaptation succeeds independently");
    Require(fixture.session->Snapshot().goal.has_value() &&
                fixture.submissions.empty(),
            "adaptation preserves intent without executing");
}
void SeparateActionsAndRealObservation() {
    Fixture fixture({0}, .05, {}, false, true, 10, 32,
                    e::ExplorationDriver::kBehaviorTree);
    Require(fixture.submissions.empty() &&
                !fixture.session->Snapshot().active_request,
            "BT driver never starts automatic selection or navigation");
    auto invalid = Request(fixture, e::ExplorationPhase::kObserve, 1.01);
    Require(fixture.session->PhaseState(invalid).status ==
                e::PhaseStatus::kInvalidated,
            "observation requires a validated adaptation");
    Prepare(fixture);
    auto observing = Request(fixture, e::ExplorationPhase::kObserve, 1.04);
    Require(
        fixture.submissions.size() == 1 &&
            !fixture.session->RequestPhase(e::ExplorationPhase::kObserve, 1.04),
        "one active phase cannot be attached to or resubmitted by another "
        "node");
    fixture.Arrive(1.05);
    Require(fixture.session->PhaseState(observing).status ==
                e::PhaseStatus::kPending,
            "arrival does not complete observation");
    fixture.Observe(1.1);
    fixture.session->Tick(1.1);
    fixture.Observe(1.16);
    fixture.session->Tick(1.16);
    Require(fixture.session->PhaseState(observing).status ==
                    e::PhaseStatus::kSucceeded &&
                fixture.session->Snapshot().last_observed_goal_id == 42,
            "actual independent observations and dwell complete the phase");
    fixture.session->Tick(1.17);
    Require(
        fixture.submissions.size() == 1 &&
            !fixture.session->Snapshot().active_request,
        "observation completion does not automatically start the next goal");
}
void ExplicitReturnSaveAndOldCancellation() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false, 10,
                    32, e::ExplorationDriver::kBehaviorTree, true);
    auto selected = Request(fixture, e::ExplorationPhase::kSelect, 1.02);
    Require(
        Await(fixture, selected, 1.02).status == e::PhaseStatus::kRetryable &&
            fixture.session->Snapshot().completion_confirmed,
        "no candidate is separate from evidence-confirmed completion");
    Require(!fixture.refresh_request && fixture.submissions.empty() &&
                fixture.session->Snapshot().result.returned ==
                    e::ReturnOutcome::kNotRequested,
            "finish candidate cannot automatically return or save");
    auto returned = Request(fixture, e::ExplorationPhase::kReturn, 1.03);
    Require(fixture.session->PhaseState(returned).status ==
                    e::PhaseStatus::kSucceeded &&
                !fixture.refresh_request,
            "return confirms home without automatically saving");
    auto saving = Request(fixture, e::ExplorationPhase::kSave, 1.04);
    Require(fixture.refresh_request.has_value(),
            "save phase explicitly requests refresh");
    Require(fixture.session->CancelPhase(selected),
            "old select halt can be queued");
    fixture.session->Tick(1.04);
    Require(fixture.session->PhaseState(saving).status ==
                    e::PhaseStatus::kPending &&
                fixture.session->Snapshot().active_phase->SameRequest(saving),
            "halt of an old completed phase cannot cancel another phase");
    p::ObservationCut cut;
    cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    fixture.refresh_reply =
        e::MappingReply{*fixture.refresh_request, true, true, cut};
    fixture.session->Tick(1.05);
    Require(fixture.save_request.has_value(),
            "confirmed refresh precedes actual save");
    fixture.save_reply =
        e::MappingReply{*fixture.save_request, true, true, cut};
    fixture.session->Tick(1.06);
    Require(fixture.session->PhaseState(saving).status ==
                    e::PhaseStatus::kSucceeded &&
                fixture.session->Snapshot().result.Success(true, true),
            "return and save outcomes certify the completed session");
}
void SelectedIntentInvalidatedByMapChange() {
    Fixture fixture({0}, .05, {}, false, true, 10, 32,
                    e::ExplorationDriver::kBehaviorTree);
    auto selected = Request(fixture, e::ExplorationPhase::kSelect, 1.02);
    Require(Await(fixture, selected, 1.02).status == e::PhaseStatus::kSucceeded,
            "intent selected");
    fixture.grid.Set(17, 17, {autonomy::navigation::CellState::kOccupied, 0});
    fixture.session->PushMap(fixture.reader.Read(fixture.grid, 1.03, 0, 1));
    fixture.session->Tick(1.03);
    auto adapted = Request(fixture, e::ExplorationPhase::kAdapt, 1.04);
    const auto resolved = Await(fixture, adapted, 1.04);
    Require(resolved.status == e::PhaseStatus::kSucceeded &&
                fixture.session->Snapshot().goal.has_value() &&
                fixture.submissions.empty(),
            "selected intent is revalidated against the latest same-epoch "
            "geometry");
}
void PreparedPoseCanSurviveUnrelatedMapChange() {
    Fixture fixture({0}, .05, {}, false, true, 10, 32,
                    e::ExplorationDriver::kBehaviorTree);
    Prepare(fixture);
    fixture.grid.Set(17, 17, {autonomy::navigation::CellState::kOccupied, 0});
    fixture.session->PushMap(fixture.reader.Read(fixture.grid, 1.035, 0, 1));
    fixture.session->Tick(1.035);
    auto observed = Request(fixture, e::ExplorationPhase::kObserve, 1.04);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (fixture.submissions.empty() &&
           fixture.session->PhaseState(observed).status ==
               e::PhaseStatus::kPending &&
           std::chrono::steady_clock::now() < deadline) {
        fixture.session->Tick(1.04);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture.submissions.size() == 1 &&
                fixture.session->PhaseState(observed).status ==
                    e::PhaseStatus::kPending,
            "unrelated geometry revision uses explicit target revalidation");
    fixture.session->CancelPhase(observed);
    fixture.session->Tick(1.05);
    Require(fixture.session->PhaseState(observed).status ==
                    e::PhaseStatus::kCanceled &&
                fixture.cancellation_count == 1,
            "only the current phase cancels its motion");
}
void ObservationLossPreservesSafeActiveGoal() {
    Fixture fixture({0}, .05, {}, false, true, 10, 32,
                    e::ExplorationDriver::kBehaviorTree);
    Prepare(fixture);
    const auto observed = Request(fixture, e::ExplorationPhase::kObserve, 1.04);
    Require(fixture.submissions.size() == 1,
            "observation phase submitted one navigation goal");
    fixture.SkipObservation();
    fixture.Observe(1.06, false);
    fixture.session->Tick(1.06);
    Require(fixture.session->Snapshot().observation_losses.pending_sources &&
                fixture.session->PhaseState(observed).status ==
                    e::PhaseStatus::kPending &&
                fixture.submissions.size() == 1 &&
                fixture.cancellation_count == 0,
            "recoverable observation loss retains safe active motion");
}
void PausePreservesPhaseButNotDwell() {
    Fixture fixture({0}, .1, {}, false, true, 10, 32,
                    e::ExplorationDriver::kBehaviorTree);
    Prepare(fixture);
    auto observed = Request(fixture, e::ExplorationPhase::kObserve, 1.04);
    fixture.Arrive(1.05);
    fixture.session->PostCommand(e::SessionCommand::kPause);
    fixture.session->Tick(1.06);
    fixture.Observe(1.1);
    fixture.Pose(1.2, 0);
    fixture.session->PostCommand(e::SessionCommand::kResume);
    fixture.session->Tick(1.2);
    Require(fixture.session->PhaseState(observed).status ==
                    e::PhaseStatus::kPending &&
                fixture.submissions.size() == 2,
            "resume keeps ticket and creates a new versioned motion request");
    fixture.Arrive(1.21);
    fixture.Observe(1.25);
    fixture.session->Tick(1.25);
    Require(fixture.session->PhaseState(observed).status ==
                e::PhaseStatus::kPending,
            "paused observations and time cannot complete resumed dwell");
    fixture.Observe(1.37);
    fixture.session->Tick(1.37);
    Require(fixture.session->PhaseState(observed).status ==
                e::PhaseStatus::kSucceeded,
            "fresh resumed observations complete the original phase");
}
void AutomaticDriverRejectsPhaseRequests() {
    Fixture fixture({0});
    Require(
        !fixture.session->RequestPhase(e::ExplorationPhase::kSelect, 1.02) &&
            fixture.submissions.size() == 1,
        "automatic and BT drivers cannot run together");
}
void StartupObservationGatesNormalSelection() {
    Fixture fixture({0}, .05, {}, false, true, 10, 32,
                    e::ExplorationDriver::kBehaviorTree, false,
                    [](e::ExplorationSessionOptions& options) {
                        options.startup_observation.enabled = true;
                        options.startup_observation.target_yaw = 3.14159;
                        options.startup_observation.time_allowance = 2;
                        options.startup_observation.deadline_margin = 1;
                    });
    auto startup =
        Request(fixture, e::ExplorationPhase::kStartupObservation, 1.02);
    Require(fixture.session->PhaseState(startup).status ==
                    e::PhaseStatus::kPending &&
                fixture.startup_request.has_value() &&
                fixture.submissions.empty(),
            "startup phase starts native rotation without planning coverage "
            "motion");
    Require(!fixture.session->RequestPhase(e::ExplorationPhase::kSelect, 1.03),
            "one active phase prevents concurrent selection");
    fixture.startup_reply = e::StartupObservationReply{
        *fixture.startup_request, true, "measured startup sweep confirmed"};
    fixture.session->Tick(1.03);
    Require(fixture.session->PhaseState(startup).status ==
                    e::PhaseStatus::kSucceeded &&
                fixture.session->Snapshot().startup_observation_confirmed &&
                fixture.submissions.empty(),
            "native rotation evidence, not timeout, releases normal selection");
    const auto selected = Request(fixture, e::ExplorationPhase::kSelect, 1.04);
    Require(Await(fixture, selected, 1.04).status == e::PhaseStatus::kSucceeded,
            "coverage selection proceeds only after startup observation");
}
void StartupObservationFailureAndCancelFailClosed() {
    auto configure = [](e::ExplorationSessionOptions& options) {
        options.startup_observation.enabled = true;
        options.startup_observation.time_allowance = 2;
        options.startup_observation.deadline_margin = 1;
    };
    Fixture failed({0}, .05, {}, false, true, 10, 32,
                   e::ExplorationDriver::kBehaviorTree, false, configure);
    auto failed_ticket =
        Request(failed, e::ExplorationPhase::kStartupObservation, 1.02);
    failed.startup_reply = e::StartupObservationReply{
        *failed.startup_request, false,
        "measured translation exceeded startup drift"};
    failed.session->Tick(1.03);
    Require(failed.session->PhaseState(failed_ticket).status ==
                    e::PhaseStatus::kFailed &&
                !failed.session->Snapshot().startup_observation_confirmed &&
                failed.submissions.empty(),
            "invalid native startup motion cannot unlock exploration");
    const auto denied = Request(failed, e::ExplorationPhase::kSelect, 1.04);
    Require(failed.session->PhaseState(denied).status ==
                    e::PhaseStatus::kRetryable &&
                failed.submissions.empty(),
            "failed startup observation remains fail-closed");

    Fixture canceled({0}, .05, {}, false, true, 10, 32,
                     e::ExplorationDriver::kBehaviorTree, false, configure);
    auto canceled_ticket =
        Request(canceled, e::ExplorationPhase::kStartupObservation, 1.02);
    Require(canceled.session->CancelPhase(canceled_ticket),
            "startup phase cancellation queued");
    canceled.session->Tick(1.03);
    Require(canceled.session->PhaseState(canceled_ticket).status ==
                    e::PhaseStatus::kCanceled &&
                canceled.startup_cancellations == 1 &&
                canceled.startup_request.has_value(),
            "cancellation targets the accepted native startup action");
}
}  // namespace
int main() {
    try {
        SeparateActionsAndRealObservation();
        std::cout << "PASS separate_actions_actual_observation\n";
        ExplicitReturnSaveAndOldCancellation();
        std::cout << "PASS explicit_return_save_old_halt\n";
        SelectedIntentInvalidatedByMapChange();
        std::cout << "PASS selected_map_invalidation\n";
        PreparedPoseCanSurviveUnrelatedMapChange();
        std::cout << "PASS prepared_pose_revalidation\n";
        ObservationLossPreservesSafeActiveGoal();
        std::cout << "PASS observation_loss_retains_goal\n";
        PausePreservesPhaseButNotDwell();
        std::cout << "PASS phase_pause_dwell\n";
        AutomaticDriverRejectsPhaseRequests();
        std::cout << "PASS mutually_exclusive_drivers\n";
        StartupObservationGatesNormalSelection();
        std::cout << "PASS startup_observation_gate\n";
        StartupObservationFailureAndCancelFailClosed();
        std::cout << "PASS startup_observation_failure_cancel\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL phase: " << error.what() << '\n';
        return 1;
    }
}
