/* Copyright 2026 The Openbot Authors. */
/** @file exploration_loss_recovery_test.cpp
 * @brief Actual queue, coverage and Session recovery regressions.
 * External mapping callbacks are controlled service substitutes. These tests
 * validate consumer contracts, not real mapper fusion or robot execution.
 */
#include "exploration_test_support.hpp"

#include <functional>
#include <iostream>
#include <utility>

namespace {
using namespace exploration_test;

/** @brief Constructs a measured ray retaining its source, sequence and epoch. */
p::SensorObservation Observation(std::uint64_t sequence, double stamp = 1,
                                 p::SensorKind sensor = p::SensorKind::kRgbd,
                                 std::uint64_t epoch = 1) {
    p::SensorObservation result;
    result.sensor = sensor;
    result.sequence = sequence;
    result.map_epoch = epoch;
    result.stamp = stamp;
    result.valid_until = stamp + 10;
    result.valid_fraction = 1;
    result.localization_valid = true;
    result.rays.push_back({{1, 1, .5}, {1, 0, 0}, 1, true, 1, stamp});
    return result;
}

/** @brief Creates a source-specific quality requirement for the physical target. */
e::CoverageUnit Target(p::SensorMask sources = 0) {
    e::CoverageUnit unit;
    unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
    unit.center = {2, 1, .5};
    unit.normal = {-1, 0, 0};
    unit.required_quality = .5;
    unit.required_sensors = sources;
    return unit;
}

/** @brief Original loss remains auditable after owner-verified acknowledgement. */
void QueueRecoveryPreservesHistoricalLoss() {
    p::ObservationBuffer queue;
    queue.Reset(1);
    Require(queue.Push(Observation(1), 1), "source baseline admitted");
    Require(queue.Push(Observation(3), 1), "explicit gap admitted");
    queue.Drain(1, 1);
    const auto loss = queue.LossSnapshot();
    Require(loss.pending_sources == p::Mask(p::SensorKind::kRgbd) &&
            loss.first_loss.sequence[1] == 2 && loss.last_loss.sequence[1] == 2,
            "gap interval and affected source retained");
    Require(!queue.AcknowledgeRecovery(loss, queue.processed_cut()),
            "a frame admitted before reopening cannot establish replacement");
    Require(queue.Push(Observation(4), 1), "post-boundary measured input admitted");
    Require(queue.LossSnapshot().generation == loss.generation,
            "ordinary admission changed the loss checkpoint");
    Require(!queue.AcknowledgeRecovery(loss, queue.accepted_cut()),
            "queued replacement is not processed evidence");
    queue.Drain(1, 1);
    const auto cut = queue.processed_cut();
    Require(queue.AcknowledgeRecovery(loss, cut), "verified replacement acknowledged");
    Require(queue.HasLossThrough(cut) && !queue.HasUnrecoveredLossThrough(cut) &&
            queue.gap_cut().sequence[1] == 2 && !queue.LossSnapshot().pending_sources &&
            queue.LossSnapshot().historical_gaps.sequence[1] == 2,
            "acknowledgement erased audit history or retained resolved pending loss");
    Require(!queue.AcknowledgeRecovery(loss, cut), "checkpoint was acknowledged twice");
}

/** @brief New loss atomically invalidates a previously captured checkpoint. */
void ConcurrentLossRejectsOldAcknowledgement() {
    p::ObservationBuffer queue;
    queue.Reset(1);
    queue.Push(Observation(1), 1);
    queue.Push(Observation(3), 1);
    queue.Drain(1, 1);
    const auto old = queue.LossSnapshot();
    queue.Push(Observation(4), 1);
    queue.Drain(1, 1);
    queue.Reject(p::SensorKind::kRgbd, 4);
    const auto latest = queue.LossSnapshot();
    Require(latest.generation > old.generation &&
            !queue.AcknowledgeRecovery(old, queue.processed_cut()),
            "old checkpoint erased a concurrent input rejection");
    queue.Reject(p::SensorKind::kRgbd, 4);
    Require(queue.LossSnapshot().generation == latest.generation,
            "duplicate interval continuously restarted recovery");
    queue.Push(Observation(5), 1);
    queue.Drain(1, 1);
    Require(queue.AcknowledgeRecovery(latest, queue.processed_cut()),
            "latest complete checkpoint could not acknowledge replacements");
}

/** @brief Epoch and same-epoch Reset cannot reuse an older checkpoint. */
void ResetRejectsCapturedRecovery() {
    p::ObservationBuffer queue;
    queue.Reset(1);
    queue.Push(Observation(1), 1);
    queue.Push(Observation(3), 1);
    queue.Drain(1, 1);
    const auto old = queue.LossSnapshot();
    queue.Reset(1);
    queue.Push(Observation(1), 1);
    queue.Push(Observation(3), 1);
    queue.Push(Observation(4), 1);
    queue.Drain(1, 1);
    Require(!queue.AcknowledgeRecovery(old, queue.processed_cut()),
            "same-epoch Reset reused the old loss generation");
    const auto same_epoch = queue.LossSnapshot();
    queue.Reset(2);
    queue.Push(Observation(1, 1, p::SensorKind::kRgbd, 2), 1);
    queue.Push(Observation(3, 1, p::SensorKind::kRgbd, 2), 1);
    queue.Push(Observation(4, 1, p::SensorKind::kRgbd, 2), 1);
    queue.Drain(1, 2);
    Require(!queue.AcknowledgeRecovery(same_epoch, queue.processed_cut()),
            "new alignment acknowledged an old epoch");
}

/** @brief Reopening RGB-D retains LiDAR and independent source requirements. */
void SourceInvalidationPreservesCoveragePolicy() {
    e::CoverageState any, each;
    any.Reset(1); each.Reset(1);
    any.Upsert(Target());
    each.Upsert(Target(p::Mask(p::SensorMode::kLidarRgbd)));
    for (auto* state : {&any, &each}) {
        state->Confirm(Target().id, p::SensorKind::kLidar, .8, 1);
        state->Confirm(Target().id, p::SensorKind::kRgbd, .8, 1);
        const auto revision = state->requirements_revision();
        state->InvalidateSources(p::Mask(p::SensorKind::kRgbd));
        const auto& unit = state->units().at(Target().id);
        Require(unit.observed_quality[0] == .8 && unit.observed_quality[1] == 0 &&
                unit.observed_stamp[0] == 1 && unit.observed_stamp[1] == 0 &&
                state->requirements_revision() > revision,
                "source invalidation erased unaffected evidence or retained an old sweep");
    }
    Require(any.PendingCount() == 0 && each.PendingCount() == 1 &&
            each.units().at(Target().id).required_sensors == p::Mask(p::SensorMode::kLidarRgbd),
            "recovery changed ANY_SENSOR/PER_SENSOR policy");
    e::CoverageState empty;
    empty.Reset(1);
    const auto revision = empty.requirements_revision();
    empty.InvalidateSources(p::Mask(p::SensorKind::kRgbd));
    Require(empty.requirements_revision() > revision,
            "empty-ledger loss reused an old full-scope sweep");
}

/** @brief Creates an actual Session with a bounded room and controlled mapper. */
std::unique_ptr<Fixture> RecoveryFixture(
    e::ObservationRecoveryPolicy policy = e::ObservationRecoveryPolicy::kReobserveScope,
    e::ExplorationDriver driver = e::ExplorationDriver::kAutomatic,
    bool explicit_scope = true,
    std::function<void(e::ExplorationSessionOptions&)> configure = {}) {
    auto fixture = std::make_unique<Fixture>(std::vector<double>{}, .05,
        std::make_shared<EmptyComplete>(), true, false, 10, 32, driver, false,
        [=](auto& options) {
            options.observation_recovery_policy = policy;
            if (!explicit_scope) options.allowed_region.reset();
            if (configure) configure(options);
        });
    Require(fixture->session->PushCoverageUnit(Target()), "target requirement admitted");
    ++fixture->sequence;
    fixture->Observe(1.02, true);
    fixture->session->Tick(1.02);
    const auto snapshot = fixture->session->Snapshot();
    Require(snapshot.pending_units > 0 && snapshot.observation_losses.pending_sources,
            "pre-reopening gap frame was accepted as replacement coverage");
    fixture->refresh_request.reset();
    return fixture;
}

/** @brief Waits boundedly for a scope refresh using owner ticks and worker progress. */
e::RequestVersion AwaitRecovery(Fixture& fixture, double now = 1.04) {
    for (unsigned i = 0; i < 300; ++i) {
        if (fixture.refresh_request && fixture.refresh_request->recovery_loss)
            return *fixture.refresh_request;
        if (fixture.session->driver() == e::ExplorationDriver::kBehaviorTree)
            fixture.session->RequestPhase(e::ExplorationPhase::kSelect, now);
        fixture.session->Tick(now);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error("new observations and sweep did not request scope recovery");
}

/** @brief Delivers an explicit mapper receipt; it never updates actual map evidence. */
e::MappingReply Receipt(const e::RequestVersion& request, bool scope = true,
                        bool original_replayed = false) {
    return {request, true, true, request.observation_cut, scope, original_replayed};
}

/** @brief Updates the actual MapReader consumer; Session's worker must update Vista. */
void MapEvidence(Fixture& fixture, const e::RequestVersion& request, double now) {
    auto view = fixture.reader.Read(fixture.grid, now, 0, request.observation_cut.sequence[1]);
    view.mapping_synchronized = true;
    view.observation_cut = request.observation_cut;
    view.observation_baseline = request.observation_baseline;
    view.observation_recovery_generation = request.observation_recovery_generation;
    Require(fixture.session->PushMap(view), "actual mapper consumer view queued");
}

/** @brief New measured replacement coverage, not the gap frame, permits refresh. */
void SessionRequiresPostBoundaryMeasurements() {
    auto fixture = RecoveryFixture();
    for (unsigned i = 0; i < 20; ++i) {
        fixture->session->Tick(1.025);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(!fixture->refresh_request && fixture->session->Snapshot().pending_units == 1,
            "loss recovery ran without new measured target coverage");
    fixture->Observe(1.03, true);
    const auto request = AwaitRecovery(*fixture);
    Require(request.recovery_loss && request.recovery_region &&
            request.observation_cut.sequence[1] > request.observation_baseline.sequence[1] &&
            request.observation_recovery_generation == request.recovery_loss->generation &&
            fixture->submissions.empty(),
            "scope recovery omitted its boundary or dispatched duplicate navigation");
}

/** @brief A loss discovered in Drain reopens evidence from the entire old boundary. */
void DrainRejectionReopensPreBoundaryCoverage() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false);
    Require(fixture.session->PushCoverageUnit(Target()), "target admitted");
    fixture.Observe(1.02, true);
    fixture.session->Tick(1.02);
    Require(!fixture.session->Snapshot().pending_units, "measured coverage established");
    auto expired = Observation(++fixture.sequence, 1.03, p::SensorKind::kRgbd, fixture.epoch);
    expired.valid_until = 1.035;
    Require(fixture.session->PushObservation(expired, 1.03), "soon-expiring input admitted");
    fixture.Observe(1.04, true);
    const auto state = fixture.session->Tick(1.04);
    Require(state.pending_units == 1 && state.observation_losses.pending_sources &&
            !state.completion_confirmed,
            "Drain loss retained measurements admitted before reopening");
    fixture.Observe(1.05, true);
    const auto request = AwaitRecovery(fixture, 1.06);
    Require(request.observation_cut.sequence[1] == fixture.sequence,
            "recovery did not use the post-Drain replacement");
}

/** @brief Receipt may precede consumers but cannot replace map/graph evidence. */
void SessionWaitsForActualConsumersAndThenSaves() {
    auto fixture = RecoveryFixture();
    fixture->Observe(1.03, true);
    const auto request = AwaitRecovery(*fixture);
    fixture->refresh_reply = Receipt(request);
    fixture->session->Tick(1.05);
    Require(fixture->session->Snapshot().observation_losses.pending_sources &&
            !fixture->save_request && !fixture->session->Snapshot().completion_confirmed,
            "mapper scope receipt replaced actual map/graph consumer evidence");
    MapEvidence(*fixture, request, 1.06);
    for (unsigned i = 0; i < 300 &&
         fixture->session->Snapshot().observation_losses.pending_sources; ++i) {
        fixture->session->Tick(1.07);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto recovered = fixture->session->Snapshot();
    Require(!recovered.observation_losses.pending_sources &&
            recovered.observation_recovery_generation == request.observation_recovery_generation &&
            recovered.observation_baseline == request.observation_baseline &&
            recovered.observation_losses.historical_gaps.sequence[1] == 2,
            "matched receipt could not wait for independently updated consumers");
    for (unsigned i = 0; i < 300 &&
         (!fixture->refresh_request || fixture->refresh_request->recovery_loss); ++i) {
        fixture->session->Tick(1.08);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture->refresh_request && !fixture->refresh_request->recovery_loss,
            "recovered scope did not undergo new normal completion evaluation");
    fixture->refresh_reply = Receipt(*fixture->refresh_request, false);
    fixture->session->Tick(1.09);
    Require(fixture->save_request &&
            fixture->save_request->observation_baseline == request.observation_baseline &&
            fixture->save_request->observation_recovery_generation == request.observation_recovery_generation,
            "final save lost the verified replacement-baseline identity");
    fixture->save_reply = Receipt(*fixture->save_request, false);
    const auto final = fixture->session->Tick(1.10);
    Require(final.result.Success(false, true) && final.completion_confirmed,
            "verified replacement scope did not complete and save");
}

/** @brief Stale baseline and missing scope attestation cannot clear pending loss. */
void WrongRecoveryReceiptsAreRejected() {
    for (unsigned variant = 0; variant < 8; ++variant) {
        auto fixture = RecoveryFixture();
        fixture->Observe(1.03, true);
        const auto request = AwaitRecovery(*fixture);
        MapEvidence(*fixture, request, 1.05);
        auto reply = Receipt(request);
        if (variant == 0) reply.version.observation_recovery_generation = 0;
        if (variant == 1) reply.version.observation_baseline = {};
        if (variant == 2) reply.recovery_scope_confirmed = false;
        if (variant == 3) ++reply.version.request_id;
        if (variant == 4) ++reply.version.geometry_revision;
        if (variant == 5) ++reply.version.requirements_revision;
        if (variant == 6) reply.version.observation_cut = {};
        if (variant == 7) reply.version.recovery_region->max_x -= 1;
        fixture->refresh_reply = reply;
        fixture->session->Tick(1.06);
        Require(fixture->session->Snapshot().observation_losses.pending_sources &&
                !fixture->session->Snapshot().completion_confirmed && !fixture->save_request,
                "invalid recovery receipt acknowledged pending data loss");
    }
}

/** @brief Consumer baseline must match independently of a valid mapper receipt. */
void WrongConsumerBaselineIsRejected() {
    auto fixture = RecoveryFixture();
    fixture->Observe(1.03, true);
    const auto request = AwaitRecovery(*fixture);
    auto stale = request;
    stale.observation_baseline = {};
    stale.observation_recovery_generation = 0;
    MapEvidence(*fixture, stale, 1.05);
    fixture->refresh_reply = Receipt(request);
    for (unsigned i = 0; i < 20; ++i) {
        fixture->session->Tick(1.06);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture->session->Snapshot().observation_losses.pending_sources && !fixture->save_request,
            "a new consumer watermark masked the wrong replacement baseline");
}

/** @brief Raw-input retention cannot be silently satisfied by reobservation. */
void StrictReplayPolicyRejectsReobservation() {
    auto fixture = RecoveryFixture(e::ObservationRecoveryPolicy::kRequireOriginalReplay);
    fixture->Observe(1.03, true);
    const auto request = AwaitRecovery(*fixture);
    MapEvidence(*fixture, request, 1.05);
    fixture->refresh_reply = Receipt(request, true, false);
    fixture->session->Tick(1.06);
    Require(fixture->session->Snapshot().observation_losses.pending_sources &&
            fixture->mapping_cancellation_count > 0 && !fixture->save_request,
            "retention-required task accepted a reobservation-only receipt");
}

/** @brief New loss, work, localization failure, pause and cancel revoke old receipts. */
void NewInvalidationsRevokeRecovery() {
    for (unsigned variant = 0; variant < 5; ++variant) {
        auto fixture = RecoveryFixture();
        fixture->Observe(1.03, true);
        const auto request = AwaitRecovery(*fixture);
        MapEvidence(*fixture, request, 1.05);
        fixture->refresh_reply = Receipt(request);
        if (variant == 0) { ++fixture->sequence; fixture->Observe(1.06, true); }
        if (variant == 1) {
            auto unit = Target(); ++unit.id.x;
            Require(fixture->session->PushCoverageUnit(unit), "new required work admitted");
        }
        if (variant == 2)
            Require(fixture->session->PushPose({1, 1, 0}, 1.06, fixture->epoch, 0),
                    "localization failure admitted");
        if (variant == 3) Require(fixture->session->PostCommand(e::SessionCommand::kPause), "pause admitted");
        if (variant == 4) Require(fixture->session->PostCommand(e::SessionCommand::kCancel), "cancel admitted");
        const auto state = fixture->session->Tick(1.06);
        const std::string detail = "recovery invalidation variant " + std::to_string(variant) +
            " pending=" + std::to_string(state.observation_losses.pending_sources) +
            " complete=" + std::to_string(state.completion_confirmed) +
            " baseline=" + std::to_string(state.observation_recovery_generation) +
            " cancels=" + std::to_string(fixture->mapping_cancellation_count);
        Require(state.observation_losses.pending_sources && !state.completion_confirmed &&
                state.observation_recovery_generation == 0 && fixture->mapping_cancellation_count > 0,
                detail.c_str());
    }
}

/** @brief Unknown task extent cannot be replaced by existing-unit reobservation. */
void RecoveryRequiresExplicitScope() {
    auto fixture = RecoveryFixture(e::ObservationRecoveryPolicy::kReobserveScope,
                                   e::ExplorationDriver::kAutomatic, false);
    fixture->Observe(1.03, true);
    for (unsigned i = 0; i < 30; ++i) {
        fixture->session->Tick(1.04);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(!fixture->refresh_request && fixture->session->Snapshot().observation_losses.pending_sources &&
            !fixture->session->Snapshot().completion_confirmed,
            "unknown full task extent was acknowledged from existing-unit coverage");
}

/** @brief Valid map evidence and a receipt still wait for the actual graph consumer. */
void RecoveryWaitsForGraphConsumer() {
    auto fixture = RecoveryFixture();
    fixture->Observe(1.03, true);
    const auto request = AwaitRecovery(*fixture);
    fixture->hold_graph_updates.store(true);
    MapEvidence(*fixture, request, 1.05);
    fixture->refresh_reply = Receipt(request);
    for (unsigned i = 0; i < 20; ++i) {
        fixture->session->Tick(1.06);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture->session->Snapshot().observation_losses.pending_sources &&
            !fixture->save_request && fixture->submissions.empty(),
            "map and receipt bypassed a graph that had not consumed the replacement baseline");
    fixture->hold_graph_updates.store(false);
    for (unsigned i = 0; i < 300 &&
         fixture->session->Snapshot().observation_losses.pending_sources; ++i) {
        fixture->session->Tick(1.07);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(!fixture->session->Snapshot().observation_losses.pending_sources,
            "retained mapper receipt could not complete after graph catch-up");
}

/** @brief A BT phase cannot replace a scope recovery's in-flight operation. */
void BehaviorTreeDoesNotSupersedeRecovery() {
    auto fixture = RecoveryFixture(e::ObservationRecoveryPolicy::kReobserveScope,
                                   e::ExplorationDriver::kBehaviorTree);
    fixture->Observe(1.03, true);
    const auto request = AwaitRecovery(*fixture);
    const auto select = fixture->session->RequestPhase(e::ExplorationPhase::kSelect, 1.05);
    Require(select && fixture->session->PhaseState(*select).status == e::PhaseStatus::kRetryable,
            "BT queued new selection over an in-flight scope recovery");
    const auto active = fixture->session->Snapshot().active_request;
    Require(active && active->SameRequest(request) && fixture->submissions.empty(),
            "BT phase replaced the mapper request or issued motion");
}

/** @brief Destruction cancels the pending scope request while its dependencies still exist. */
void DestructionCancelsRecovery() {
    auto fixture = RecoveryFixture();
    fixture->Observe(1.03, true);
    AwaitRecovery(*fixture);
    const auto canceled = fixture->mapping_cancellation_count;
    fixture->session.reset();
    Require(fixture->mapping_cancellation_count == canceled + 1,
            "Session destruction leaked the in-flight recovery operation");
}

/** @brief Invalid policies and impossible scope geometry are rejected before worker startup. */
void InvalidRecoveryConfigurationIsRejected() {
    for (unsigned variant = 0; variant < 4; ++variant) {
        bool rejected = false;
        try {
            Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false,
                10, 32, e::ExplorationDriver::kAutomatic, false, [=](auto& options) {
                    if (variant == 0)
                        options.observation_recovery_policy = static_cast<e::ObservationRecoveryPolicy>(99);
                    if (variant == 1)
                        options.allowed_region = e::Bounds2d{0, 0, 0, 1};
                    if (variant == 2)
                        options.allowed_region = e::Bounds2d{2, 0, 1, 1};
                    if (variant == 3)
                        options.allowed_region = e::Bounds2d{0, 0,
                            std::numeric_limits<double>::infinity(), 1};
                });
        } catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected, "invalid recovery policy or full task scope was admitted");
    }
}

/** @brief Pause makes queued old-boundary measurements ineligible after resume. */
void PauseResumeCapturesANewRecoveryBoundary() {
    auto fixture = RecoveryFixture();
    fixture->Observe(1.03, true);
    const auto old_request = AwaitRecovery(*fixture);
    Require(fixture->session->PostCommand(e::SessionCommand::kPause), "pause admitted");
    Require(fixture->session->Tick(1.05).state == e::ExplorationState::kPaused,
            "recovery did not enter the paused state");
    Require(fixture->mapping_cancellations.size() == 1 &&
            fixture->mapping_cancellations.back().SameRequest(old_request),
            "pause did not cancel exactly the owned recovery request");
    fixture->Observe(1.06, true);
    fixture->session->Tick(1.06);
    fixture->refresh_reply = Receipt(old_request);
    fixture->refresh_request.reset();
    Require(fixture->session->PostCommand(e::SessionCommand::kResume), "resume admitted");
    const auto resumed = fixture->session->Tick(1.07);
    Require(resumed.pending_units == 1 && resumed.observation_losses.pending_sources &&
            !resumed.observation_recovery_generation,
            "a paused queued measurement or old receipt established the resumed baseline");
    fixture->Observe(1.08, true);
    const auto new_request = AwaitRecovery(*fixture, 1.09);
    Require(!new_request.SameRequest(old_request) &&
            new_request.observation_baseline.sequence[1] > old_request.observation_baseline.sequence[1] &&
            new_request.observation_cut.sequence[1] > new_request.observation_baseline.sequence[1],
            "resume reused the old request or admission boundary");
    fixture->refresh_reply = Receipt(old_request);
    fixture->session->Tick(1.10);
    Require(fixture->session->Snapshot().observation_losses.pending_sources,
            "late pre-pause receipt acknowledged the resumed scope");
    MapEvidence(*fixture, new_request, 1.11);
    fixture->refresh_reply = Receipt(new_request);
    for (unsigned i = 0; i < 300 &&
         fixture->session->Snapshot().observation_losses.pending_sources; ++i) {
        fixture->session->Tick(1.12);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(!fixture->session->Snapshot().observation_losses.pending_sources,
            "new post-resume scope could not recover after consumers caught up");
}

/** @brief Expired requests retain loss, respect backoff and stop at the attempt cap. */
void RecoveryTimeoutHonorsBackoffAndAttemptBudget() {
    auto fixture = RecoveryFixture(e::ObservationRecoveryPolicy::kReobserveScope,
        e::ExplorationDriver::kAutomatic, true, [](auto& options) {
            options.map_operation_timeout = .1;
            options.blocked_retry_delay = .05;
            options.maximum_recoveries = 2;
        });
    fixture->Observe(1.03, true);
    const auto first = AwaitRecovery(*fixture);
    const double expired = first.valid_until + .01;
    fixture->refresh_reply = Receipt(first);
    fixture->session->Tick(expired);
    Require(fixture->mapping_cancellations.size() == 1 &&
            fixture->mapping_cancellations.back().SameRequest(first) &&
            fixture->session->Snapshot().observation_losses.pending_sources,
            "deadline accepted a late receipt or lost cancellation responsibility");
    fixture->refresh_request.reset();
    for (unsigned i = 0; i < 20; ++i) {
        fixture->session->Tick(expired + .02);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture->refresh_submissions.size() == 1,
            "mapper retry ignored the owner-clock backoff");
    const auto second = AwaitRecovery(*fixture, expired + .06);
    Require(!first.SameRequest(second) && fixture->refresh_submissions.size() == 2,
            "retry reused an expired identity or submitted duplicate refreshes");
    fixture->session->Tick(second.valid_until + .01);
    fixture->refresh_request.reset();
    for (unsigned i = 0; i < 30; ++i) {
        fixture->session->Tick(second.valid_until + .1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture->refresh_submissions.size() == 2 &&
            fixture->mapping_cancellations.size() == 2 &&
            fixture->session->Snapshot().observation_losses.pending_sources &&
            !fixture->session->Snapshot().completion_confirmed,
            "attempt exhaustion dispatched more requests or certified incomplete observations");
}

/** @brief Geometry and alignment updates retire old receipts before scope confirmation. */
void MapChangesRetireRecoveryRequests() {
    for (unsigned variant = 0; variant < 2; ++variant) {
        auto fixture = RecoveryFixture();
        fixture->Observe(1.03, true);
        const auto old_request = AwaitRecovery(*fixture);
        if (variant == 0) fixture->grid.Set(0, 0, {n::CellState::kOccupied, 0});
        auto changed = fixture->reader.Read(fixture->grid, 1.05,
                                            variant == 0 ? 0 : 1, fixture->sequence);
        Require(variant == 0 ? changed.geometry_revision > old_request.geometry_revision :
                              changed.map_epoch > old_request.map_epoch,
                "actual MapReader did not advance the changed dependency");
        fixture->epoch = changed.map_epoch;
        Require(fixture->session->PushMap(changed), "changed map queued");
        Require(fixture->session->PushCoverageUnit(Target()), "current-alignment requirement queued");
        fixture->Pose(1.06, 0);
        fixture->refresh_reply = Receipt(old_request);
        const auto state = fixture->session->Tick(1.06);
        Require(fixture->mapping_cancellations.size() == 1 &&
                fixture->mapping_cancellations.back().SameRequest(old_request) &&
                state.pending_units > 0 && !state.completion_confirmed &&
                !state.observation_recovery_generation && !fixture->save_request,
                "changed map consumed an old recovery receipt or erased unfinished work");
    }
}

/** @brief The strict-policy consumer accepts an attested replay only with actual consumers.
 * The replay flag is a service substitute, not proof of a production replay producer.
 */
void StrictPolicyRequiresReplayAndConsumerEvidence() {
    auto fixture = RecoveryFixture(e::ObservationRecoveryPolicy::kRequireOriginalReplay);
    fixture->Observe(1.03, true);
    const auto request = AwaitRecovery(*fixture);
    fixture->refresh_reply = Receipt(request, true, true);
    fixture->session->Tick(1.05);
    Require(fixture->session->Snapshot().observation_losses.pending_sources,
            "replay attestation bypassed actual map and graph consumers");
    MapEvidence(*fixture, request, 1.06);
    for (unsigned i = 0; i < 300 &&
         fixture->session->Snapshot().observation_losses.pending_sources; ++i) {
        fixture->session->Tick(1.07);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto state = fixture->session->Snapshot();
    Require(!state.observation_losses.pending_sources &&
            state.observation_recovery_generation == request.observation_recovery_generation &&
            state.observation_losses.historical_gaps.sequence[1] == 2,
            "strict-policy consumer could not accept matched replay and consumer evidence");
}

/** @brief All health thresholds reject NaN and infinity before accepting observations. */
void HealthConfigurationRejectsNonFiniteValues() {
    double p::SensorHealthOptions::* const fields[] = {
        &p::SensorHealthOptions::maximum_age,
        &p::SensorHealthOptions::minimum_valid_fraction,
        &p::SensorHealthOptions::minimum_completeness,
        &p::SensorHealthOptions::dropout_hold,
        &p::SensorHealthOptions::recovery_hold
    };
    for (const auto field : fields) {
        for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
            p::SensorHealthOptions options;
            options.*field = invalid;
            bool rejected = false;
            try { p::SensorHealthMonitor monitor(p::SensorMode::kLidarRgbd, options); }
            catch (const std::invalid_argument&) { rejected = true; }
            Require(rejected, "non-finite sensor health threshold was admitted");
        }
    }
}
}  // namespace

/** @brief Runs consumer regressions without claiming actual external-service fusion. */
int main() {
    const std::pair<const char*, std::function<void()>> tests[] = {
        {"queue_historical_loss", QueueRecoveryPreservesHistoricalLoss},
        {"queue_concurrent_loss", ConcurrentLossRejectsOldAcknowledgement},
        {"queue_reset_epoch", ResetRejectsCapturedRecovery},
        {"coverage_source_policy", SourceInvalidationPreservesCoveragePolicy},
        {"session_post_boundary_measurement", SessionRequiresPostBoundaryMeasurements},
        {"session_drain_boundary", DrainRejectionReopensPreBoundaryCoverage},
        {"session_consumers_completion_save", SessionWaitsForActualConsumersAndThenSaves},
        {"session_invalid_receipts", WrongRecoveryReceiptsAreRejected},
        {"session_wrong_consumer_baseline", WrongConsumerBaselineIsRejected},
        {"session_strict_replay_policy", StrictReplayPolicyRejectsReobservation},
        {"session_recovery_invalidation", NewInvalidationsRevokeRecovery},
        {"session_explicit_scope", RecoveryRequiresExplicitScope},
        {"session_graph_consumer_wait", RecoveryWaitsForGraphConsumer},
        {"bt_recovery_request_ownership", BehaviorTreeDoesNotSupersedeRecovery},
        {"session_recovery_destruction", DestructionCancelsRecovery},
        {"recovery_configuration_validation", InvalidRecoveryConfigurationIsRejected},
        {"session_recovery_pause_resume", PauseResumeCapturesANewRecoveryBoundary},
        {"session_recovery_deadline_budget", RecoveryTimeoutHonorsBackoffAndAttemptBudget},
        {"session_recovery_map_changes", MapChangesRetireRecoveryRequests},
        {"session_strict_replay_consumer", StrictPolicyRequiresReplayAndConsumerEvidence},
        {"sensor_health_finite_configuration", HealthConfigurationRejectsNonFiniteValues},
    };
    bool failed = false;
    for (const auto& [name, run] : tests) {
        try { run(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            failed = true;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    return failed ? 1 : 0;
}
