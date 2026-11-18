/* Copyright 2026 The Openbot Authors. */
/** @file exploration_input_prefix_test.cpp
 * @brief Session regressions with continuously admitted observation suffixes.
 */
#include "exploration_test_support.hpp"

#include <functional>
#include <iostream>
#include <utility>

namespace {
using namespace exploration_test;

/** @brief Supplies the same concrete observation intent until the owner executes it. */
class TargetGoal final : public e::ExplorationStrategy {
public:
    /** @brief Selects only after the required unit exists in the worker snapshot. */
    e::StrategyDecision Select(const e::Pose2d&, const e::Pose2d&,
        const e::MapReadView& map, const e::CoverageState& coverage,
        p::SensorMask, double, const std::function<bool()>& canceled) override {
        e::StrategyDecision result;
        const e::CoverageId id{10, 5, 2, 0, e::CoverageKind::kSurface};
        if (!coverage.units().count(id) || (canceled && canceled())) return result;
        e::ObservationGoal goal;
        goal.goal_id = 42;
        goal.goal_version = 1;
        goal.map_epoch = map.map_epoch;
        goal.coverage_revision = coverage.revision();
        goal.original_pose = {1, 1, 0};
        goal.required_views = {0};
        goal.required_coverage_units = {id};
        goal.allowed_translation = goal.allowed_yaw_change = 0;
        goal.minimum_gain = .01;
        result.goal = std::move(goal);
        return result;
    }
};

/** @brief Admits one full drain budget while a prior suffix remains queued. */
void ProduceSuffix(Fixture& fixture, double stamp) {
    for (unsigned i = 0; i < 8; ++i) fixture.Observe(stamp, false);
}

/** @brief Adds the target without simulating a predicted coverage receipt. */
void AddTarget(Fixture& fixture) {
    e::CoverageUnit unit;
    unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
    unit.center = {2, 1, .5};
    unit.normal = {-1, 0, 0};
    unit.required_quality = .5;
    Require(fixture.session->PushCoverageUnit(unit), "target requirement admitted");
    fixture.session->Tick(1.01);
}

/** @brief BT select/adapt can finish with a permanently nonempty observation queue. */
void BehaviorTreePhasesProcessFinitePrefixes() {
    Fixture fixture({0}, .05, std::make_shared<TargetGoal>(), false, false,
                    10, 32, e::ExplorationDriver::kBehaviorTree);
    AddTarget(fixture);
    ProduceSuffix(fixture, 1.02);
    ProduceSuffix(fixture, 1.02);
    const auto select = fixture.session->RequestPhase(e::ExplorationPhase::kSelect, 1.02);
    Require(select.has_value(), "select has a phase ticket");
    bool selected = false;
    for (unsigned i = 0; i < 40; ++i) {
        ProduceSuffix(fixture, 1.02);
        fixture.session->Tick(1.02);
        selected = fixture.session->PhaseState(*select).status == e::PhaseStatus::kSucceeded;
        if (selected) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(selected, "continuous input starved BT observation selection");
    const auto adapt = fixture.session->RequestPhase(e::ExplorationPhase::kAdapt, 1.02);
    Require(adapt.has_value(), "adapt has a phase ticket under backlog");
    bool adapted = false;
    for (unsigned i = 0; i < 40; ++i) {
        ProduceSuffix(fixture, 1.02);
        fixture.session->Tick(1.02);
        adapted = fixture.session->PhaseState(*adapt).status == e::PhaseStatus::kSucceeded;
        if (adapted) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(adapted && fixture.submissions.empty(),
            "adaptation starved or dispatched motion before Observe phase");
}

/** @brief Automatic scheduling and result consumption do not require global drain. */
void AutomaticNavigationProcessesFinitePrefixes() {
    Fixture fixture({0}, .05, std::make_shared<TargetGoal>(), false, false);
    AddTarget(fixture);
    ProduceSuffix(fixture, 1.02);
    ProduceSuffix(fixture, 1.02);
    for (unsigned i = 0; i < 40 && fixture.submissions.empty(); ++i) {
        ProduceSuffix(fixture, 1.02);
        fixture.session->Tick(1.02);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture.submissions.size() == 1,
            "continuous input starved automatic navigation dispatch");
    Require(fixture.session->Snapshot().pending_units > 0,
            "processing a prefix cannot fabricate target coverage");
}

/** @brief Later redundant healthy input does not expand an already verified save cut. */
void CompletionAndSaveRetainFinitePrefix() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false);
    ProduceSuffix(fixture, 1.02);
    ProduceSuffix(fixture, 1.02);
    for (unsigned i = 0; i < 40 && !fixture.refresh_request; ++i) {
        ProduceSuffix(fixture, 1.02);
        fixture.session->Tick(1.02);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture.refresh_request.has_value(),
            "continuous redundant observations starved completion");
    p::ObservationCut fused;
    fused.Include(p::SensorKind::kRgbd, 1);
    fixture.refresh_reply = e::MappingReply{*fixture.refresh_request, true, true, fused};
    ProduceSuffix(fixture, 1.03);
    fixture.session->Tick(1.03);
    Require(fixture.save_request.has_value(), "verified finite prefix is saved");
    fixture.save_reply = e::MappingReply{*fixture.save_request, true, true, fused};
    ProduceSuffix(fixture, 1.04);
    const auto result = fixture.session->Tick(1.04);
    Require(result.result.Success(false, true) && result.completion_confirmed,
            "healthy suffix incorrectly revoked an independently verified prefix");
}

/** @brief FIFO processing evidence keeps gaps and earliest losses distinct. */
void ObservationPrefixLossesAreExplicit() {
    p::ObservationBuffer buffer(8);
    buffer.Reset(1);
    p::SensorObservation observation;
    observation.sensor = p::SensorKind::kRgbd;
    observation.map_epoch = 1;
    observation.sequence = 1;
    observation.stamp = 1;
    observation.valid_until = 10;
    observation.valid_fraction = 1;
    observation.localization_valid = true;
    observation.rays.push_back({{0, 0, 0}, {1, 0, 0}, 1, true, 1, 1});
    Require(buffer.Push(observation, 1), "first source observation establishes baseline");
    const auto first = buffer.accepted_cut();
    observation.sequence = 3;
    Require(buffer.Push(observation, 1), "later source sequence admitted with explicit gap");
    const auto third = buffer.accepted_cut();
    Require(!buffer.ProcessedThrough(first), "admission was confused with processing");
    buffer.Drain(1, 1, 1);
    Require(buffer.ProcessedThrough(first) && !buffer.ProcessedThrough(third) &&
                !buffer.Empty(), "a queued suffix blocked a processed finite prefix");
    Require(!buffer.HasLossThrough(first) && buffer.HasLossThrough(third) &&
                buffer.gap_cut().sequence[1] == 2,
            "a source gap was hidden by its latest sequence");
    buffer.Drain(1, 1);
    buffer.Reject(p::SensorKind::kRgbd, 1);
    buffer.Reject(p::SensorKind::kRgbd, 3);
    Require(buffer.ProcessedThrough(first) && buffer.HasLossThrough(first),
            "a later discard hid an earlier prefix loss");
    buffer.Reset(2);
    observation.map_epoch = 2;
    observation.sequence = 5;
    Require(buffer.Push(observation, 1), "new session permits an independent source baseline");
    const auto expired = buffer.accepted_cut();
    Require(buffer.Drain(11, 2).empty() && buffer.ProcessedThrough(expired) &&
                buffer.HasLossThrough(expired), "expired processing was counted as valid evidence");
    observation.sequence = 0;
    Require(!buffer.Push(observation, 1), "zero sequence cannot certify a source prefix");
    observation.sequence = 6;
    observation.sensor = static_cast<p::SensorKind>(3);
    Require(!buffer.Push(observation, 1), "unsupported source acquired a ledger slot");
}

/** @brief Processing and healthy input cannot replace real map fusion confirmation. */
void MissingFusionReceiptPreventsCompletion() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false);
    auto map = fixture.reader.Read(fixture.grid, 1.02, 0, fixture.sequence);
    map.mapping_synchronized = false;
    Require(fixture.session->PushMap(map), "unconfirmed mapper view admitted");
    ProduceSuffix(fixture, 1.02);
    ProduceSuffix(fixture, 1.02);
    for (unsigned i = 0; i < 20; ++i) {
        ProduceSuffix(fixture, 1.02);
        fixture.session->Tick(1.02);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(!fixture.refresh_request && !fixture.session->Snapshot().completion_confirmed,
            "local prefix processing fabricated a map-fusion receipt");
}

/** @brief New required work revokes a finite saved-prefix candidate. */
void NewWorkRevokesFiniteSaveCandidate() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false);
    for (unsigned i = 0; i < 40 && !fixture.refresh_request; ++i) {
        fixture.session->Tick(1.02);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture.refresh_request.has_value(), "completed prefix requests final refresh");
    p::ObservationCut fused;
    fused.Include(p::SensorKind::kRgbd, 1);
    fixture.refresh_reply = e::MappingReply{*fixture.refresh_request, true, true, fused};
    fixture.session->Tick(1.03);
    Require(fixture.save_request.has_value(), "save has a versioned request");
    const auto old = *fixture.save_request;
    e::CoverageUnit unit;
    unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
    unit.center = {2, 1, .5};
    unit.required_quality = .5;
    Require(fixture.session->PushCoverageUnit(unit), "new required work admitted during save");
    fixture.save_reply = e::MappingReply{old, true, true, fused};
    const auto result = fixture.session->Tick(1.04);
    Require(!result.completion_confirmed && !result.result.Success(false, true),
            "old saved prefix concealed new required coverage");
}

/** @brief Repeated healthy pose events may remain queued after a finite phase prefix. */
void ContinuousPoseEventsDoNotStarveSelection() {
    Fixture fixture({0}, .05, std::make_shared<TargetGoal>(), false, false,
                    10, 32, e::ExplorationDriver::kBehaviorTree);
    AddTarget(fixture);
    for (unsigned i = 0; i < 64; ++i) fixture.Pose(1.02, 0);
    const auto ticket = fixture.session->RequestPhase(e::ExplorationPhase::kSelect, 1.02);
    Require(ticket && ticket->version.input_event_sequence > 0, "phase binds ordinary event prefix");
    bool selected = false;
    for (unsigned i = 0; i < 30; ++i) {
        for (unsigned j = 0; j < 32; ++j) fixture.Pose(1.02, 0);
        fixture.session->Tick(1.02);
        selected = fixture.session->PhaseState(*ticket).status == e::PhaseStatus::kSucceeded;
        if (selected) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(selected && fixture.submissions.empty(),
            "continuous healthy pose events starved intent selection");
}

/** @brief Partial measured quality gains do not invalidate a selected work-domain intent. */
void QualityGainsPreserveSelectedIntent() {
    Fixture fixture({0}, .05, std::make_shared<TargetGoal>(), false, false,
                    10, 32, e::ExplorationDriver::kBehaviorTree);
    AddTarget(fixture);
    const auto selected = fixture.session->RequestPhase(e::ExplorationPhase::kSelect, 1.01);
    Require(selected.has_value(), "selection ticket exists");
    for (unsigned i = 0; i < 40; ++i) {
        fixture.session->Tick(1.02);
        if (fixture.session->PhaseState(*selected).status == e::PhaseStatus::kSucceeded) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture.session->PhaseState(*selected).status == e::PhaseStatus::kSucceeded,
            "intent selected before measured quality change");
    p::SensorObservation observation;
    observation.sensor = p::SensorKind::kRgbd;
    observation.sequence = ++fixture.sequence;
    observation.map_epoch = fixture.epoch;
    observation.stamp = 1.03;
    observation.valid_until = 10;
    observation.localization_valid = true;
    observation.valid_fraction = 1;
    observation.rays.push_back({{1, 1, .5}, {1, 0, 0}, 1, true, .1, 1.03});
    Require(fixture.session->PushObservation(observation, 1.03), "partial measured quality admitted");
    fixture.session->Tick(1.03);
    const auto adapted = fixture.session->RequestPhase(e::ExplorationPhase::kAdapt, 1.03);
    Require(adapted.has_value(), "adaptation requested after quality improvement");
    for (unsigned i = 0; i < 40; ++i) {
        fixture.session->Tick(1.03);
        if (fixture.session->PhaseState(*adapted).status != e::PhaseStatus::kPending) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture.session->PhaseState(*adapted).status == e::PhaseStatus::kSucceeded,
            "a quality-only revision invalidated a selected observation intent");
}

/** @brief A newer queued localization failure cannot hide behind a completed prefix. */
void QueuedLocalizationLossPreventsFinalSave() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false);
    for (unsigned i = 0; i < 40 && !fixture.refresh_request; ++i) {
        fixture.session->Tick(1.02);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(fixture.refresh_request.has_value(), "completed prefix requires final refresh");
    p::ObservationCut fused;
    fused.Include(p::SensorKind::kRgbd, 1);
    fixture.refresh_reply = e::MappingReply{*fixture.refresh_request, true, true, fused};
    fixture.session->Tick(1.03);
    Require(fixture.save_request.has_value(), "versioned save is pending");
    for (unsigned i = 0; i < 64; ++i) fixture.Pose(1.04, 0);
    Require(fixture.session->PushPose({1, 1, 0}, 1.04, fixture.epoch, 0),
            "localization loss admitted beyond the completed prefix");
    fixture.save_reply = e::MappingReply{*fixture.save_request, true, true, fused};
    const auto result = fixture.session->Tick(1.04);
    Require(!result.completion_confirmed && !result.result.Success(false, true),
            "a queued health failure was ignored by finite-prefix finalization");
}
/** @brief Queue saturation must leave loss evidence even without a later admission. */
void RejectedCapacityInputIsRecorded() {
    p::ObservationBuffer buffer(1);
    buffer.Reset(1);
    p::SensorObservation observation;
    observation.sensor = p::SensorKind::kRgbd;
    observation.map_epoch = 1;
    observation.sequence = 1;
    observation.stamp = 1;
    observation.valid_until = 10;
    observation.valid_fraction = 1;
    observation.localization_valid = true;
    observation.rays.push_back({{0, 0, 0}, {1, 0, 0}, 1, true, 1, 1});
    Require(buffer.Push(observation, 1), "source baseline admitted");
    observation.sequence = 2;
    Require(!buffer.Push(observation, 1), "full queue rejects input");
    p::ObservationCut attempted;
    attempted.Include(p::SensorKind::kRgbd, 2);
    Require(buffer.HasLossThrough(attempted), "full queue silently lost the rejected source input");
}
}  // namespace

/** @brief Runs actual Session producer/consumer regressions independently. */
int main() {
    unsigned failures = 0;
    for (const auto& test : std::vector<std::pair<const char*, std::function<void()>>>{
             {"bt_phase_finite_prefix", BehaviorTreePhasesProcessFinitePrefixes},
             {"automatic_navigation_finite_prefix", AutomaticNavigationProcessesFinitePrefixes},
             {"completion_save_finite_prefix", CompletionAndSaveRetainFinitePrefix},
             {"observation_prefix_gap_loss", ObservationPrefixLossesAreExplicit},
             {"missing_fusion_receipt", MissingFusionReceiptPreventsCompletion},
             {"new_work_revokes_saved_prefix", NewWorkRevokesFiniteSaveCandidate},
             {"continuous_pose_event_prefix", ContinuousPoseEventsDoNotStarveSelection},
             {"quality_gain_preserves_selected_intent", QualityGainsPreserveSelectedIntent},
             {"queued_localization_loss", QueuedLocalizationLossPreventsFinalSave},
             {"queue_capacity_loss", RejectedCapacityInputIsRecorded}}) {
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
