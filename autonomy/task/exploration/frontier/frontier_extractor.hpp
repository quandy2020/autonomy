/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/coverage/coverage_state.hpp"
#include "autonomy/task/exploration/integration/map_reader.hpp"
namespace autonomy::task::exploration {
class FrontierExtractor {
public:
    std::vector<CoverageUnit> Extract(const MapReadView& map,double observation_height=0.5,
        const std::optional<Bounds2d>& allowed_region={}) const;
};
}  // namespace autonomy::task::exploration
