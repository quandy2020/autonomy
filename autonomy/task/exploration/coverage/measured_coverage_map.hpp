/* Copyright 2026 The Openbot Authors. */
/** @file measured_coverage_map.hpp
 * @brief Native projection of actual coverage requirements and measured quality.
 */
#pragma once
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include "autonomy/task/exploration/coverage/coverage_state.hpp"
#include "autonomy/perception/data/map_reader.hpp"

namespace autonomy::task::exploration {
/** @brief Project only the owner's measured coverage ledger onto its actual map lattice.
 * Values are coverage percentages, not occupancy probabilities: -1 means no
 * ledger requirement, 0 means required but unobserved, 100 means every requirement
 * in that cell is confirmed. Partially fulfilled cells remain below 100, including
 * multi-source requirements. Geometry-known cells and predictions never populate
 * this output. This target-ledger projection does not prove whole-room coverage.
 */
::automsgs::msgs::map_msgs::OccupancyGrid BuildMeasuredCoverageMap(
    const CoverageState& measured, const perception::MapReadView& map,
    double publication_stamp, std::size_t maximum_cells = 2000000);
}  // namespace autonomy::task::exploration
