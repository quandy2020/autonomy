/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_sample_reducer.hpp
 * @brief Screen-space plot sample reduction for draw paths.
 *
 * Algorithm inspired by PlotJuggler PlotSampleReducer (MPL-2.0): keep finite-run
 * endpoints, bucket XY extrema, never bridge NaN/Inf gaps.
 */

#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include <QPointF>

namespace autoviz {
namespace plot {

struct ReducedPlotSample {
  QPointF point;
  std::size_t source_index = 0;
  bool starts_new_run = false;
};

struct PlotSampleReduction {
  std::vector<ReducedPlotSample> samples;
  std::size_t input_samples = 0;
  std::size_t finite_samples = 0;
  std::size_t input_runs = 0;
  std::size_t retained_runs = 0;
  std::size_t dropped_runs = 0;
  bool reduced = false;
};

using PlotSampleAccessor = std::function<QPointF(std::size_t)>;

/**
 * @brief Reduce already-mapped (typically pixel-space) samples.
 *
 * @param sample_count Number of samples in the accessor domain.
 * @param sample_budget Hard ceiling on retained finite samples.
 * @param horizontal_bucket_count Target horizontal buckets (often chart width).
 * @param sample_at Pure accessor; must be stable for the call duration.
 */
[[nodiscard]] PlotSampleReduction ReducePlotSamples(
    std::size_t sample_count, std::size_t sample_budget,
    std::size_t horizontal_bucket_count, const PlotSampleAccessor& sample_at);

}  // namespace plot
}  // namespace autoviz
