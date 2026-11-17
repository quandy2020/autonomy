/* Copyright 2026 The Openbot Authors. */
/** @file coverage_fusion.hpp
 * @brief Deduplicated quality improvement relative to measured coverage and selected views.
 */
#pragma once
#include <vector>
#include "autonomy/task/exploration/coverage/coverage_state.hpp"

namespace autonomy::task::exploration {
/** @brief Predicted raw maxima reserved by selected views, never measured evidence. */
struct CoverageSelection {
    std::map<CoverageId, double> geometry;  ///< Raw ANY_SENSOR quality maxima in [0, 1].
    std::map<std::pair<CoverageId, perception::SensorKind>, double> sources;  ///< Raw per-source maxima.
};
/** @brief Pure gain operations that cannot modify the real coverage ledger. */
class CoverageFusion {
public:
    /** @brief Maximum raw quality per stable unit; duplicate points/sources are never summed.
     * @throws std::invalid_argument If a category, source or raw quality is invalid.
     */
    static std::map<CoverageId, double> Fuse(const std::vector<CoverageContribution>& contributions);
    /** @brief Positive normalized improvement with optional ANY_SENSOR reservations.
     * @param reference_scales Fixed positive scales for frontier, surface and direction.
     * Source-specific reservations require MarginalGain instead of this overload.
     */
    static double Gain(const CoverageState& state,
                       const std::vector<CoverageContribution>& contributions,
                       const std::map<CoverageId, double>& selected_union = {},
                       const std::array<double, 3>& reference_scales = {1, 1, 1});
    /** @brief Improvement over the maximum of measured and selected raw quality.
     * Normalize quality as min(1, raw / required_quality). ANY_SENSOR uses the best
     * source; PER_SENSOR averages improvement over independently required sources.
     * @throws std::invalid_argument If contributions, reservations or scales are invalid.
     */
    static double MarginalGain(const CoverageState& state,
                       const std::vector<CoverageContribution>& contributions,
                       const CoverageSelection& selected,
                       const std::array<double, 3>& reference_scales = {1, 1, 1});
    /** @brief Reserves predicted maxima without altering real evidence.
     * @throws std::invalid_argument Before mutation if any input/reservation is invalid.
     */
    static void Accumulate(const std::vector<CoverageContribution>& contributions,
                           CoverageSelection& selected);
};
}  // namespace autonomy::task::exploration
