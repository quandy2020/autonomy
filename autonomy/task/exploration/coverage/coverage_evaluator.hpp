/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/coverage/coverage_state.hpp"

namespace autonomy::task::exploration {
struct CoverageRealization {
    double realized_gain = 0;
    std::size_t confirmed_units = 0;
    std::vector<CoverageContribution> evidence;
    std::string reason;
};
class CoverageEvaluator {
public:
    CoverageRealization Apply(const perception::SensorObservation& observation,
                              double now, CoverageState& coverage) const;
};
}  // namespace autonomy::task::exploration
