/* Copyright 2026 The Openbot Authors. */
/** @file frontier_extractor.hpp
 * @brief Connected frontier requirement extraction.
 */
#pragma once

#include <automsgs/msgs/geometry_msgs/polygon.pb.h>

#include <cstddef>
#include <optional>
#include <vector>

#include "autonomy/task/exploration/coverage/coverage_state.hpp"
#include "autonomy/task/exploration/integration/map_reader.hpp"

namespace autonomy::task::exploration {
/** @brief Extracts stable frontier units after connected-component filtering.
 */
class FrontierExtractor
{
public:
    /** @brief Returns measured-volume 3-D frontiers, or planar frontiers when
     * no volume is supplied, belonging to sufficiently large clusters.
     */
    std::vector<CoverageUnit> Extract(
        const MapReadView& map, double observation_height = .5,
        const std::optional<Bounds2d>& allowed_region = {},
        std::size_t minimum_cluster_size = 1,
        const std::optional<::automsgs::msgs::geometry_msgs::Polygon>&
            allowed_polygon = {},
        const std::vector<::automsgs::msgs::geometry_msgs::Polygon>&
            no_go_areas = {}) const;
};
}  // namespace autonomy::task::exploration
