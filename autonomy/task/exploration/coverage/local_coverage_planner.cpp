/* Copyright 2026 The Openbot Authors. */
/**
 * @file local_coverage_planner.cpp
 * @brief Deterministic diversified greedy selection of local coverage views.
 */
#include "autonomy/task/exploration/coverage/local_coverage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "autonomy/task/exploration/coverage/coverage_fusion.hpp"

namespace autonomy::task::exploration {
namespace {

/** @brief Returns contributions of one semantic kind. */
std::vector<CoverageContribution> ContributionsOfKind(
    const Viewpoint& viewpoint, CoverageKind kind) {
    std::vector<CoverageContribution> result;
    for (const auto& contribution : viewpoint.contributions) {
        if (contribution.id.kind == kind)
            result.push_back(contribution);
    }
    return result;
}

/** @brief Returns contributions to one mandatory stable unit. */
std::vector<CoverageContribution> ContributionsToUnit(
    const Viewpoint& viewpoint, const CoverageId& id) {
    std::vector<CoverageContribution> result;
    for (const auto& contribution : viewpoint.contributions) {
        if (contribution.id == id)
            result.push_back(contribution);
    }
    return result;
}

/** @brief Checks whether any semantic component clears its own threshold. */
bool HasAdmissibleIncrement(const Viewpoint& viewpoint,
                            const CoverageState& state,
                            const CoverageSelection& covered,
                            const LocalCoverageSelectionOptions& options) {
    const double surface_gain = CoverageFusion::MarginalGain(
        state, ContributionsOfKind(viewpoint, CoverageKind::kSurface), covered);
    const double frontier_gain = CoverageFusion::MarginalGain(
        state, ContributionsOfKind(viewpoint, CoverageKind::kFrontier),
        covered);
    const double direction_gain = CoverageFusion::MarginalGain(
        state, ContributionsOfKind(viewpoint, CoverageKind::kDirection),
        covered);
    return (surface_gain > 0 &&
            surface_gain >= options.minimum_incremental_surface_gain) ||
           (frontier_gain > 0 &&
            frontier_gain >= options.minimum_incremental_frontier_gain) ||
           direction_gain > 0;
}

/** @brief One currently admissible candidate and its exact marginal score. */
struct ScoredCandidate {
    const Viewpoint* viewpoint = nullptr;
    double gain = 0;
    double utility = 0;
};

}  // namespace

std::vector<Viewpoint> LocalCoveragePlanner::Select(
    const std::vector<Viewpoint>& candidates, const CoverageState& state,
    std::size_t maximum_views,
    const std::optional<CoverageId>& mandatory_frontier,
    LocalCoverageSelectionOptions options,
    const std::vector<Viewpoint>& retained) const {
    if (!std::isfinite(options.minimum_incremental_surface_gain) ||
        options.minimum_incremental_surface_gain < 0 ||
        !std::isfinite(options.minimum_incremental_frontier_gain) ||
        options.minimum_incremental_frontier_gain < 0 ||
        !std::isfinite(options.diversification_ratio) ||
        options.diversification_ratio <= 0 ||
        options.diversification_ratio > 1) {
        throw std::invalid_argument("invalid local coverage selection options");
    }

    std::vector<Viewpoint> selected;
    CoverageSelection covered;
    std::set<std::uint64_t> used;

    auto append = [&](const Viewpoint& candidate, bool forced) {
        if (selected.size() >= maximum_views || used.count(candidate.id)) {
            return false;
        }
        const double gain = CoverageFusion::MarginalGain(
            state, candidate.contributions, covered);
        if (!(gain > 0) ||
            (!forced &&
             !HasAdmissibleIncrement(candidate, state, covered, options))) {
            return false;
        }
        Viewpoint view = candidate;
        view.predicted_gain = gain;
        selected.push_back(std::move(view));
        used.insert(candidate.id);
        CoverageFusion::Accumulate(candidate.contributions, covered);
        return true;
    };

    // Fair work is a hard constraint, not one more thresholded candidate.
    // Choose its best contributor first so every subsequently compared route
    // contains it.
    if (maximum_views && mandatory_frontier) {
        const Viewpoint* forced = nullptr;
        double forced_utility = -1;
        for (const auto& candidate : candidates) {
            const double mandatory_gain = CoverageFusion::MarginalGain(
                state, ContributionsToUnit(candidate, *mandatory_frontier),
                covered);
            const double total_gain = CoverageFusion::MarginalGain(
                state, candidate.contributions, covered);
            const double utility = candidate.Utility(total_gain);
            if (mandatory_gain > 0 && std::isfinite(utility) && utility > 0 &&
                (forced == nullptr || utility > forced_utility ||
                 (utility == forced_utility && candidate.id < forced->id))) {
                forced = &candidate;
                forced_utility = utility;
            }
        }
        if (forced != nullptr)
            append(*forced, true);
    }

    // Preserve stable local work before choosing replacements. Re-evaluation
    // against `covered` removes views made redundant by the mandatory choice or
    // newly measured evidence.
    for (const auto& previous : retained) {
        const auto current =
            std::find_if(candidates.begin(), candidates.end(),
                         [&](const Viewpoint& candidate) {
                             return candidate.id == previous.id;
                         });
        if (current != candidates.end())
            append(*current, false);
    }

    std::size_t greedy_step = 0;
    while (selected.size() < maximum_views) {
        std::vector<ScoredCandidate> eligible;
        double best_utility = -1;
        for (const auto& candidate : candidates) {
            if (used.count(candidate.id) ||
                !HasAdmissibleIncrement(candidate, state, covered, options)) {
                continue;
            }
            const double gain = CoverageFusion::MarginalGain(
                state, candidate.contributions, covered);
            const double utility = candidate.Utility(gain);
            if (!(gain > 0) || !std::isfinite(utility) || !(utility > 0))
                continue;
            eligible.push_back({&candidate, gain, utility});
            best_utility = std::max(best_utility, utility);
        }
        if (eligible.empty())
            break;

        const double near_best = best_utility * options.diversification_ratio;
        eligible.erase(std::remove_if(eligible.begin(), eligible.end(),
                                      [&](const ScoredCandidate& candidate) {
                                          return candidate.utility < near_best;
                                      }),
                       eligible.end());
        std::sort(
            eligible.begin(), eligible.end(),
            [](const ScoredCandidate& left, const ScoredCandidate& right) {
                if (left.utility != right.utility) {
                    return left.utility > right.utility;
                }
                return left.viewpoint->id < right.viewpoint->id;
            });

        const std::size_t choice = static_cast<std::size_t>(
            (options.diversification_seed + greedy_step) % eligible.size());
        append(*eligible[choice].viewpoint, false);
        ++greedy_step;
    }
    return selected;
}

}  // namespace autonomy::task::exploration
