/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

using PlotSubscriptionId = std::uint64_t;

struct PlotPoint {
  double x = 0.0;
  double y = 0.0;
};

enum class PlotTimestampMode {
  kLogTime = 0,
  kReceiveTime = 1,
  kCustomField = 2,
};

enum class PlotXAxisMode {
  kTimestamp = 0,
  kIndex = 1,
  kMessagePath = 2,
};

enum class PlotMessagePathMode {
  kCurrent = 0,
  kAccumulated = 1,
};

struct PlotSeriesConfig {
  QString channel;
  QString field_path;
  QString x_field_path;
  QString custom_timestamp_path;
  QString label;
  QColor color = QColor(QStringLiteral("#4e98e2"));
  QString line_size = QStringLiteral("auto");
  bool show_line = true;
  PlotTimestampMode timestamp_mode = PlotTimestampMode::kLogTime;
  bool enabled = true;
};

struct PlotSeriesRuntime {
  PlotSeriesConfig config;
  integration::MessageQueue queue;
  PlotSubscriptionId subscription_id = 0;
  std::deque<PlotPoint> points;
  int index_counter = 0;
  double last_raw_value = 0.0;
  double last_timestamp_sec = 0.0;
  bool has_last_sample = false;
};

struct PlotPanelConfig {
  QString title = QStringLiteral("Plot");
  PlotXAxisMode x_axis_mode = PlotXAxisMode::kTimestamp;
  PlotMessagePathMode message_path_mode = PlotMessagePathMode::kAccumulated;
  bool sync_with_other_plots = true;
  bool show_legend_values = true;
  bool settings_visible = false;
  int settings_width = 300;
  bool lock_axis_scales = false;
  double x_window_sec = 30.0;
  QVector<PlotSeriesConfig> series;
};

struct PlotValueRow {
  QString label;
  QString value_text;
  QColor color;
  bool latched = false;
};

}  // namespace plot
}  // namespace autoviz
