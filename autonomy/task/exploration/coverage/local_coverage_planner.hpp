/* Copyright 2026 The Openbot Authors. */
/**
 * @file local_coverage_planner.hpp
 * @brief Deterministic diversified greedy selection of local coverage views.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "autonomy/task/exploration/viewpoint/viewpoint.hpp"

namespace autonomy::task::exploration {

/** @brief Bounded greedy set-cover policy for one local selection attempt. */
struct LocalCoverageSelectionOptions {
    /** Minimum normalized incremental surface gain admitted after forced work.
     */
    double minimum_incremental_surface_gain = 0.01;
    /** Minimum normalized incremental frontier gain admitted after forced work.
     */
    double minimum_incremental_frontier_gain = 0.01;
    /** Utility ratio defining the near-best deterministic diversification pool.
     */
    double diversification_ratio = 0.9;
    /** Stable seed selecting one member of each near-best pool. */
    std::uint64_t diversification_seed = 0;
};

/** @brief Selects useful views without constructing or ordering navigation
 * routes. */
class LocalCoveragePlanner
{
public:
    /**
     * @brief Selects one deterministic diversified greedy cover set.
     * @param candidates Current executable viewpoints.
     * @param state Measured coverage against which gains are incremental.
     * @param maximum_views Hard bound on the returned set size.
     * @param mandatory_frontier Optional fair-work unit forced into the first
     * view.
     * @param options Gain admission and deterministic diversification policy.
     * @param retained Previously selected views retained while still useful.
     * @return Unordered cover set for subsequent exact Vista matrix ordering.
     *
     * A mandatory contributor bypasses incremental kind thresholds but must
     * still improve its mandatory unit. Retained views are reconsidered against
     * current measured and already reserved coverage; stale views are not
     * retained.
     *
     * @throws std::invalid_argument If a threshold or ratio is invalid.
     */
    std::vector<Viewpoint> Select(
        const std::vector<Viewpoint>& candidates, const CoverageState& state,
        std::size_t maximum_views,
        const std::optional<CoverageId>& mandatory_frontier = {},
        LocalCoverageSelectionOptions options = {},
        const std::vector<Viewpoint>& retained = {}) const;
};

}  // namespace autonomy::task::exploration
