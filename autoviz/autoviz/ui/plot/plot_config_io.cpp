/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_config_io.hpp"

#include <QColor>

namespace autoviz {
namespace plot {
namespace {

common::PlotSeriesPersistConfig ToPersistSeries(const PlotSeriesConfig& series) {
  common::PlotSeriesPersistConfig persist;
  persist.channel = series.channel.toStdString();
  persist.field_path = series.field_path.toStdString();
  persist.x_field_path = series.x_field_path.toStdString();
  persist.custom_timestamp_path = series.custom_timestamp_path.toStdString();
  persist.label = series.label.toStdString();
  persist.color = series.color.name().toStdString();
  persist.line_size = series.line_size.toStdString();
  persist.show_line = series.show_line;
  persist.timestamp_mode = static_cast<int>(series.timestamp_mode);
  persist.enabled = series.enabled;
  return persist;
}

PlotSeriesConfig FromPersistSeries(const common::PlotSeriesPersistConfig& persist) {
  PlotSeriesConfig series;
  series.channel = QString::fromStdString(persist.channel);
  series.field_path = QString::fromStdString(persist.field_path);
  series.x_field_path = QString::fromStdString(persist.x_field_path);
  series.custom_timestamp_path =
      QString::fromStdString(persist.custom_timestamp_path);
  series.label = QString::fromStdString(persist.label);
  series.color = QColor(QString::fromStdString(persist.color));
  series.line_size = QString::fromStdString(persist.line_size);
  series.show_line = persist.show_line;
  series.timestamp_mode =
      static_cast<PlotTimestampMode>(persist.timestamp_mode);
  series.enabled = persist.enabled;
  return series;
}

}  // namespace

common::PlotPanelPersistConfig ToPersistConfig(const QString& object_name,
                                               const PlotPanelConfig& config) {
  common::PlotPanelPersistConfig persist;
  persist.object_name = object_name.toStdString();
  persist.title = config.title.toStdString();
  persist.x_axis_mode = static_cast<int>(config.x_axis_mode);
  persist.message_path_mode = static_cast<int>(config.message_path_mode);
  persist.sync_with_other_plots = config.sync_with_other_plots;
  persist.show_legend_values = config.show_legend_values;
  persist.settings_visible = config.settings_visible;
  persist.settings_width = config.settings_width;
  persist.lock_axis_scales = config.lock_axis_scales;
  persist.x_window_sec = config.x_window_sec;
  persist.series.reserve(config.series.size());
  for (const PlotSeriesConfig& series : config.series) {
    persist.series.push_back(ToPersistSeries(series));
  }
  return persist;
}

PlotPanelConfig FromPersistConfig(const common::PlotPanelPersistConfig& persist) {
  PlotPanelConfig config;
  config.title = QString::fromStdString(persist.title);
  config.x_axis_mode = static_cast<PlotXAxisMode>(persist.x_axis_mode);
  config.message_path_mode =
      static_cast<PlotMessagePathMode>(persist.message_path_mode);
  config.sync_with_other_plots = persist.sync_with_other_plots;
  config.show_legend_values = persist.show_legend_values;
  config.settings_visible = persist.settings_visible;
  config.settings_width = persist.settings_width > 0 ? persist.settings_width : 300;
  config.lock_axis_scales = persist.lock_axis_scales;
  config.x_window_sec = persist.x_window_sec;
  config.series.reserve(static_cast<int>(persist.series.size()));
  for (const common::PlotSeriesPersistConfig& series : persist.series) {
    config.series.push_back(FromPersistSeries(series));
  }
  return config;
}

}  // namespace plot
}  // namespace autoviz
