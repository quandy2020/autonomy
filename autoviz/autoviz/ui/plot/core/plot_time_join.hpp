/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_time_join.hpp
 * @brief Binary-search nearest-in-time join helpers for sorted sample deques.
 */

#pragma once

#include <deque>
#include <optional>
#include <utility>

#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

/**
 * @brief Nearest sample Y whose @c PlotPoint::x is within @p max_dt of @p t.
 *
 * @p samples must be non-decreasing in @c x (time-ordered append buffers).
 */
[[nodiscard]] std::optional<double> NearestYByTime(
    const std::deque<PlotPoint>& samples, double t, double max_dt);

/**
 * @brief Lower/upper indices of samples with @c x in [@p min_x, @p max_x].
 *
 * Returns [begin, end) into @p samples; empty when no overlap.
 */
[[nodiscard]] std::pair<std::size_t, std::size_t> VisibleIndexRange(
    const std::deque<PlotPoint>& samples, double min_x, double max_x);

/**
 * @brief Min/max Y over samples whose X lies in [@p min_x, @p max_x].
 */
[[nodiscard]] bool VisibleYRange(const std::deque<PlotPoint>& samples,
                                 double min_x, double max_x, double* min_y,
                                 double* max_y);

}  // namespace plot
}  // namespace autoviz
