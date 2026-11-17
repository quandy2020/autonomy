/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/coverage/completion_evaluator.hpp"
#include <stdexcept>

namespace autonomy::task::exploration {
CompletionEvaluator::CompletionEvaluator(unsigned rounds) : required_rounds_(rounds) {
    if (!rounds) throw std::invalid_argument("completion requires stable confirmation rounds");
}
void CompletionEvaluator::Reset() {
    stable_rounds_ = 0;
    epoch_.reset(); revision_.reset(); round_.reset();
    last_valid_cut_.reset(); last_observation_watermark_.reset();
    last_required_sources_ = 0;
}
CompletionDecision CompletionEvaluator::Evaluate(const CompletionEvidence& e) {
    auto fail = [this](const char* reason) {
        stable_rounds_ = 0; return CompletionDecision{false,0,reason};
    };
    if (!epoch_ || *epoch_ != e.map_epoch || !revision_ || *revision_ != e.geometry_revision) {
        Reset(); epoch_ = e.map_epoch; revision_ = e.geometry_revision;
    }
    if (!e.sensing_healthy || !e.localization_healthy) return fail("sensor or localization unavailable");
    if (e.require_source_cuts && (!e.mapping_synchronized ||
        !e.map_cut.Covers(e.observation_cut) || !e.graph_cut.Covers(e.map_cut)))
        return fail("per-sensor map fusion barrier not confirmed");
    if (!e.observation_queue_empty || (!e.require_source_cuts &&
        (e.map_watermark != e.graph_watermark || e.graph_watermark != e.observation_watermark)))
        return fail("update watermarks inconsistent");
    if (e.reachable_frontiers || e.pending_observations || e.pending_revisits)
        return fail("frontier, observation or revisit still pending");
    if (e.unresolved_regions) return fail("unresolved regions cannot be declared complete");
    if (!e.grid_fallback_checked || !e.sweep_checked || e.sweep_revision != e.geometry_revision)
        return fail("grid fallback and low-gain sweep have not confirmed this revision");
    if (round_ && e.confirmation_round <= *round_)
        return {stable_rounds_ >= required_rounds_,stable_rounds_,"duplicate confirmation round"};
    if (e.require_source_cuts) {
        const auto required = e.required_observation_sources;
        if (!required || (required & ~perception::SensorMask{3}) ||
            (e.valid_observation_cut.present & required) != required ||
            !e.observation_cut.Covers(e.valid_observation_cut))
            return fail("required sources have no valid observation evidence");
        if (last_required_sources_ != required) {
            stable_rounds_ = 0;
            last_valid_cut_.reset();
            last_required_sources_ = required;
        }
        for (unsigned source = 0; source < 2; ++source) {
            if (!(required & (1U << source))) continue;
            const auto sequence = e.valid_observation_cut.sequence[source];
            if (!sequence || (last_valid_cut_ &&
                sequence <= last_valid_cut_->sequence[source]))
                return {false,stable_rounds_,"waiting for new valid observations from every required source"};
        }
        last_valid_cut_ = e.valid_observation_cut;
    } else {
        // Compatibility for readers with a single real processed watermark.
        // Native sessions always use the stricter, source-specific contract.
        if (!e.observation_watermark || (last_observation_watermark_ &&
            e.observation_watermark <= *last_observation_watermark_))
            return {false,stable_rounds_,"waiting for a new observation watermark"};
        last_observation_watermark_ = e.observation_watermark;
    }
    round_ = e.confirmation_round; ++stable_rounds_;
    return {stable_rounds_ >= required_rounds_,stable_rounds_,
            stable_rounds_ >= required_rounds_ ? "completion evidence confirmed" : "waiting for stable rounds"};
}
}  // namespace autonomy::task::exploration
