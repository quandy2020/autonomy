/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/viewpoint/viewpoint.hpp"
namespace autonomy::task::exploration {
class LocalCoveragePlanner {
public:
    std::vector<Viewpoint> Select(const std::vector<Viewpoint>& candidates,const CoverageState& state,
        std::size_t maximum_views,const std::optional<CoverageId>& mandatory_frontier={}) const;
};
}  // namespace autonomy::task::exploration
