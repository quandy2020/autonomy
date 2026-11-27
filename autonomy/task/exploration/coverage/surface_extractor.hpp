/* Copyright 2026 The Openbot Authors. */
/** @file surface_extractor.hpp
 * @brief Measured occupied/free face requirement extraction.
 */
#pragma once

#include <automsgs/msgs/geometry_msgs/polygon.pb.h>

#include <optional>
#include <vector>

#include "autonomy/task/exploration/coverage/coverage_state.hpp"
#include "autonomy/task/exploration/integration/map_reader.hpp"

namespace autonomy::task::exploration {
/** @brief Extracts observed 3-D surface faces without treating unknown as free.
 */
class SurfaceExtractor
{
public:
    /** @brief Returns occupied voxel faces adjacent to explicitly measured
     * free. */
    std::vector<CoverageUnit> Extract(
        const MapReadView& map,
        const std::optional<Bounds2d>& allowed_region = {},
        const std::optional<::automsgs::msgs::geometry_msgs::Polygon>&
            allowed_polygon = {},
        const std::vector<::automsgs::msgs::geometry_msgs::Polygon>&
            no_go_areas = {}) const;
};
}  // namespace autonomy::task::exploration
