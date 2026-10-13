/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_xy_align.hpp
 * @brief Timestamp join of independent X/Y streams into XY PlotPoints.
 *
 * Inspired by PlotJuggler PointSeriesXY alignment (nearest-in-time pairing).
 */

#pragma once

#include <deque>
#include <optional>
#include <vector>

#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

/**
 * @brief Push a timed scalar into a buffer used for XY alignment.
 *
 * @c PlotPoint::x stores timestamp; @c PlotPoint::y stores the axis value.
 */
void PushTimedSample(std::deque<PlotPoint>& buffer, double timestamp_sec,
                     double value, int max_points);

/**
 * @brief Pair nearest X/Y samples by timestamp within @p max_dt_sec.
 *
 * Consumes matched pairs from both buffers (removes used heads up to match).
 * Output @c PlotPoint uses {x_value, y_value} with @c t set to the Y stamp.
 */
[[nodiscard]] std::vector<PlotPoint> AlignXyNearest(
    std::deque<PlotPoint>& x_samples, std::deque<PlotPoint>& y_samples,
    double max_dt_sec);

/**
 * @brief Find XY point whose sample time is at-or-before @p time_sec.
 */
[[nodiscard]] std::optional<PlotPoint> SampleXyFromTime(
    const std::deque<PlotPoint>& points, double time_sec);

}  // namespace plot
}  // namespace autoviz
