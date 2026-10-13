/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_series_pipeline.hpp
 * @brief Per-series sample extraction, trim, binary ops, and XY alignment.
 */

#pragma once

#include <functional>
#include <string>

#include <QString>

#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

/** Default max retained points per series deque. */
inline constexpr int kDefaultMaxPointsPerSeries = 20000;

/**
 * @brief Context for one panel tick's sampling pass.
 */
struct SeriesPipelineContext {
  PlotXAxisMode x_axis_mode = PlotXAxisMode::kTimestamp;
  PlotMessagePathMode message_path_mode = PlotMessagePathMode::kAccumulated;
  double x_window_sec = 30.0;
  int max_points_per_series = kDefaultMaxPointsPerSeries;
  double receive_time = 0.0;
  double sim_time = 0.0;
  /** Resolve protobuf type name for a channel. */
  std::function<std::string(const std::string& channel)> message_type_for_channel;
};

[[nodiscard]] bool BinaryOpEnabled(const PlotSeriesConfig& config);
[[nodiscard]] QString EffectiveSecondaryChannel(const PlotSeriesConfig& config);
[[nodiscard]] bool UsesCrossChannelBinary(const PlotSeriesConfig& config);
[[nodiscard]] QString EffectiveXChannel(const PlotSeriesConfig& config);
[[nodiscard]] bool UsesCrossChannelXy(const PlotSeriesConfig& config,
                                      PlotXAxisMode x_axis_mode);

/**
 * @brief Resolve sample timestamp for a payload.
 */
[[nodiscard]] double ResolveTimestampSec(const PlotSeriesRuntime& runtime,
                                         const std::string& message_type,
                                         const std::string& payload,
                                         double receive_time, double sim_time);

/**
 * @brief Trim primary/secondary/x sample deques by count and optional time window.
 */
void TrimSeriesPoints(PlotSeriesRuntime& runtime, PlotXAxisMode x_axis_mode,
                      double x_window_sec, int max_points, double latest_x);

/**
 * @brief Drain queues and append plotted samples for one series.
 *
 * When @p paused is true, queues are drained without appending.
 *
 * @return @c true when any plotted sample was appended (chart should repaint).
 */
[[nodiscard]] bool DrainAndSampleSeries(PlotSeriesRuntime& runtime,
                                        const SeriesPipelineContext& ctx,
                                        bool paused);

}  // namespace plot
}  // namespace autoviz
