/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_types.hpp
 * @brief Shared enums, series/panel configs, and runtime types for Plot.
 *
 * Consumed by @ref PlotPanel, @ref PlotChartWidget, settings, legend, and
 * session persist IO (@ref plot_config_io.hpp).
 *
 * @see PlotPanelConfig
 * @see PlotSeriesConfig
 * @see PlotSeriesRuntime
 */

#pragma once

#include <QColor>
#include <QString>
#include <QVector>

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "autoviz/integration/message_queue.hpp"

namespace autoviz {
namespace plot {

/**
 * @brief Opaque subscription id type used by plot series runtimes.
 */
using PlotSubscriptionId = std::uint64_t;

/**
 * @struct PlotPoint
 * @brief One sample in a series polyline (X may be time, index, or path value).
 */
struct PlotPoint {
  double x = 0.0;  /**< Horizontal axis value. */
  double y = 0.0;  /**< Vertical axis value (after modifiers). */
};

/**
 * @enum PlotTimestampMode
 * @brief How a series chooses its timestamp when X-axis is time-based.
 */
enum class PlotTimestampMode {
  kLogTime = 0,      /**< Use log / bag timestamp. */
  kReceiveTime = 1,  /**< Use host receive time. */
  kCustomField = 2,  /**< Use @ref PlotSeriesConfig::custom_timestamp_path. */
};

/**
 * @enum PlotXAxisMode
 * @brief What the chart X axis represents for the panel.
 */
enum class PlotXAxisMode {
  kTimestamp = 0,    /**< Time in seconds (windowed). */
  kIndex = 1,        /**< Monotonic sample index. */
  kMessagePath = 2,  /**< X from another message field path. */
};

/**
 * @enum PlotInteractionMode
 * @brief Mouse tool mode for @ref PlotChartWidget.
 */
enum class PlotInteractionMode {
  kPan,     /**< Drag to pan the view. */
  kZoom,    /**< Box-zoom / wheel zoom. */
  kSelect,  /**< Default: seek / add-series cursor. */
  kInspect, /**< Place A/B inspect points. */
  kBrush,   /**< Drag an X-range brush for interval statistics. */
};

/**
 * @enum PlotMessagePathMode
 * @brief How message-path X mode accumulates samples.
 */
enum class PlotMessagePathMode {
  kCurrent = 0,      /**< Show only the latest message's path values. */
  kAccumulated = 1,  /**< Append path samples over time. */
};

/**
 * @enum PlotBinaryOp
 * @brief Optional binary combine of primary Y with a secondary field / channel.
 */
enum class PlotBinaryOp {
  kNone = 0, /**< No secondary operand. */
  kAdd = 1,  /**< primary + secondary. */
  kSub = 2,  /**< primary - secondary. */
  kMul = 3,  /**< primary * secondary. */
  kDiv = 4,  /**< primary / secondary. */
};

/**
 * @struct PlotSeriesConfig
 * @brief Persisted settings for one plot series.
 */
struct PlotSeriesConfig {
  QString channel;  /**< Source channel name. */
  QString field_path;  /**< Y field path (may include @modifiers). */
  QString x_field_path;  /**< X field path when X-axis is message-path. */
  QString custom_timestamp_path;  /**< Stamp path for @ref PlotTimestampMode::kCustomField. */
  QString label;  /**< Legend / tooltip label. */
  QColor color = QColor(QStringLiteral("#4e98e2"));  /**< Series stroke color. */
  QString line_size = QStringLiteral("auto");  /**< Stroke width hint (@c auto or px). */
  bool show_line = true;  /**< Draw connecting line (vs points only). */
  bool use_right_y = false;  /**< Plot against the right Y-axis. */
  PlotBinaryOp binary_op = PlotBinaryOp::kNone;  /**< Combine with secondary operand. */
  QString secondary_channel;  /**< Secondary channel; empty = same as @ref channel. */
  QString secondary_field_path;  /**< Secondary Y field path. */
  double binary_max_dt_sec = 0.1;  /**< Max |Δt| for cross-channel nearest match. */
  PlotTimestampMode timestamp_mode = PlotTimestampMode::kLogTime;  /**< Stamp source. */
  bool enabled = true;  /**< When @c false, series is not subscribed / drawn. */
};

/**
 * @struct PlotSeriesRuntime
 * @brief Live state for one series: queue, subscription, samples, modifiers.
 */
struct PlotSeriesRuntime {
  PlotSeriesConfig config;  /**< Mirrored series settings. */
  integration::MessageQueue queue;  /**< Incoming payloads. */
  PlotSubscriptionId subscription_id = 0;  /**< Active subscription (@c 0 = none). */
  integration::MessageQueue secondary_queue;  /**< Secondary channel payloads. */
  PlotSubscriptionId secondary_subscription_id = 0;  /**< Secondary subscription. */
  std::deque<PlotPoint> points;  /**< Ring of plotted samples. */
  std::deque<PlotPoint> secondary_samples;  /**< Recent secondary samples for join. */
  int index_counter = 0;  /**< Next X when axis mode is index. */
  double last_raw_value = 0.0;  /**< Previous raw Y for stateful modifiers. */
  double last_timestamp_sec = 0.0;  /**< Previous stamp for derivatives. */
  bool has_last_sample = false;  /**< Whether modifier state is initialized. */
};

/**
 * @struct PlotPanelConfig
 * @brief Full persisted configuration for an @ref PlotPanel instance.
 */
struct PlotPanelConfig {
  QString title = QStringLiteral("Plot");  /**< Panel / dock title. */
  PlotXAxisMode x_axis_mode = PlotXAxisMode::kTimestamp;  /**< X-axis mode. */
  PlotMessagePathMode message_path_mode = PlotMessagePathMode::kAccumulated;  /**< Path X mode. */
  bool sync_with_other_plots = true;  /**< Participate in @ref PlotViewSync. */
  bool show_legend_values = true;  /**< Show live values in the legend. */
  bool settings_visible = false;  /**< Settings pane visibility. */
  int settings_width = 300;  /**< Preferred settings width (pixels). */
  bool lock_axis_scales = false;  /**< Freeze auto-fit after user zoom. */
  bool y_auto_scale = true;  /**< When false, use @ref y_min / @ref y_max. */
  double y_min = 0.0;  /**< Fixed Y minimum when auto-scale is off. */
  double y_max = 1.0;  /**< Fixed Y maximum when auto-scale is off. */
  bool y_auto_scale_right = true;  /**< Auto-fit right Y-axis. */
  double y_min_right = 0.0;  /**< Fixed right Y minimum. */
  double y_max_right = 1.0;  /**< Fixed right Y maximum. */
  bool show_grid = true;  /**< Draw chart grid lines. */
  bool show_reference_y = false;  /**< Draw a horizontal reference line. */
  double reference_y = 0.0;  /**< Y value for the reference line. */
  double x_window_sec = 30.0;  /**< Rolling time window (seconds). */
  QVector<PlotSeriesConfig> series;  /**< Series list. */
};

/**
 * @struct PlotValueRow
 * @brief One legend / tooltip value line for a series.
 */
struct PlotValueRow {
  QString label;       /**< Series label. */
  QString value_text;  /**< Formatted Y value. */
  QColor color;        /**< Series color. */
  bool latched = false; /**< True when showing last-known (not exact) value. */
};

}  // namespace plot
}  // namespace autoviz
