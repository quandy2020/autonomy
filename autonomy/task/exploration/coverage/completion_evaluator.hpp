/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::task::exploration {
enum class ExplorationOutcome { kCompleted, kPartialBudget, kIncompleteObservation,
    kNoReachableView, kCanceled, kFailedLocalization };
enum class ReturnOutcome { kNotRequested, kReturned, kReturnFailed };
enum class MapSaveOutcome { kNotRequested, kSaved, kSaveFailed };
struct ExplorationResult {
    ExplorationOutcome exploration = ExplorationOutcome::kIncompleteObservation;
    ReturnOutcome returned = ReturnOutcome::kNotRequested;
    MapSaveOutcome saved = MapSaveOutcome::kNotRequested;
    bool Success(bool require_return, bool require_save) const {
        return exploration == ExplorationOutcome::kCompleted &&
            (!require_return || returned == ReturnOutcome::kReturned) &&
            (!require_save || saved == MapSaveOutcome::kSaved);
    }
};
struct CompletionEvidence {
    std::uint64_t map_epoch = 0, geometry_revision = 0;
    std::uint64_t map_watermark = 0, graph_watermark = 0, observation_watermark = 0;
    std::uint64_t sweep_revision = 0, confirmation_round = 0;
    std::size_t reachable_frontiers = 0, pending_observations = 0;
    std::size_t pending_revisits = 0, unresolved_regions = 0;
    bool sensing_healthy = false, localization_healthy = false;
    bool observation_queue_empty = false, grid_fallback_checked = false, sweep_checked = false;
    bool require_source_cuts = false, mapping_synchronized = false;
    perception::ObservationCut map_cut, graph_cut, observation_cut;
    // Accepted/processed input is not necessarily usable sensing evidence.
    perception::ObservationCut valid_observation_cut;
    perception::SensorMask required_observation_sources = 0;
};
struct CompletionDecision { bool complete = false; unsigned stable_rounds = 0; std::string reason; };
class CompletionEvaluator {
public:
    explicit CompletionEvaluator(unsigned required_rounds = 3);
    CompletionDecision Evaluate(const CompletionEvidence& evidence);
    void Reset();
private:
    unsigned required_rounds_, stable_rounds_ = 0;
    std::optional<std::uint64_t> epoch_, revision_, round_;
    std::optional<perception::ObservationCut> last_valid_cut_;
    std::optional<std::uint64_t> last_observation_watermark_;
    perception::SensorMask last_required_sources_ = 0;
};
}  // namespace autonomy::task::exploration
