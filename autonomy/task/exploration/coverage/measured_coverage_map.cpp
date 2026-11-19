/* Copyright 2026 The Openbot Authors. */
/** @file measured_coverage_map.cpp
 * @brief Area-weighted native coverage projection without map/prediction substitution.
 */
#include "autonomy/task/exploration/coverage/measured_coverage_map.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace autonomy::task::exploration {
::automsgs::msgs::map_msgs::OccupancyGrid BuildMeasuredCoverageMap(
    const CoverageState& measured,const perception::MapReadView& map,double stamp,std::size_t maximum_cells) {
    if (!map.grid || !map.map_epoch || measured.epoch() != map.map_epoch || map.frame_id.empty() ||
        map.grid->frame_id() != map.frame_id || !maximum_cells || map.grid->size() > maximum_cells ||
        !std::isfinite(stamp) || stamp < 0 || stamp > std::numeric_limits<std::int32_t>::max())
        throw std::invalid_argument("measured coverage projection requires matching bounded map/ledger identity");
    struct Cell {
        double area = 0;  ///< Actual requirement area assigned to this cell, independent from task priority.
        double observed = 0;  ///< Area-weighted normalized actual measured quality.
        bool complete = true;  ///< True only when every actual requirement is fulfilled.
    };
    std::vector<Cell> cells(map.grid->size());
    for (const auto& item : measured.units()) {
        const auto& unit = item.second;
        const auto index = map.grid->WorldToCell({unit.center.x,unit.center.y,0});
        if (!index) continue;
        auto& cell = cells[map.grid->Index(index->x,index->y)];
        const double remaining = measured.Remaining(item.first);
        cell.area += unit.area; cell.observed += unit.area*std::clamp(1-remaining,0.,1.);
        cell.complete = cell.complete && measured.Complete(item.first);
        if (!std::isfinite(cell.area) || !std::isfinite(cell.observed))
            throw std::overflow_error("measured coverage requirement area overflow");
    }
    ::automsgs::msgs::map_msgs::OccupancyGrid output;
    output.mutable_header()->set_frame_id(map.frame_id);
    auto seconds = static_cast<std::int64_t>(std::floor(stamp));
    auto nanos = static_cast<std::int64_t>(std::llround((stamp-seconds)*1e9));
    seconds += nanos/1000000000LL; nanos %= 1000000000LL;
    if (seconds > std::numeric_limits<std::int32_t>::max()) throw std::out_of_range("coverage publication time exceeds native Header range");
    output.mutable_header()->mutable_stamp()->set_sec(seconds);
    output.mutable_header()->mutable_stamp()->set_nanosec(nanos);
    auto* info = output.mutable_info(); info->set_resolution(map.grid->resolution());
    info->set_width(map.grid->width()); info->set_height(map.grid->height());
    info->mutable_origin()->mutable_position()->set_x(map.grid->origin_x());
    info->mutable_origin()->mutable_position()->set_y(map.grid->origin_y());
    info->mutable_origin()->mutable_orientation()->set_w(1);
    for (const auto& cell : cells) output.add_data(cell.area == 0 ? -1 : cell.complete ? 100 :
        std::clamp(static_cast<int>(std::floor(100*cell.observed/cell.area)),0,99));
    return output;
}
}  // namespace autonomy::task::exploration
