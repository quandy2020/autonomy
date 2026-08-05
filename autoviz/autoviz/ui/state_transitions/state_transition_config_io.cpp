/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/state_transitions/state_transition_config_io.hpp"

#include <QColor>

namespace autoviz {
namespace state_transitions {
namespace {

common::StateTransitionMappingPersistConfig ToPersistMapping(
    const StateMappingRule& rule) {
  common::StateTransitionMappingPersistConfig persist;
  persist.kind = static_cast<int>(rule.kind);
  persist.match_value = rule.match_value.toStdString();
  persist.range_min = rule.range_min;
  persist.range_max = rule.range_max;
  persist.label = rule.label.toStdString();
  persist.color = rule.color.name().toStdString();
  return persist;
}

StateMappingRule FromPersistMapping(
    const common::StateTransitionMappingPersistConfig& persist) {
  StateMappingRule rule;
  rule.kind = static_cast<StateMappingKind>(persist.kind);
  rule.match_value = QString::fromStdString(persist.match_value);
  rule.range_min = persist.range_min;
  rule.range_max = persist.range_max;
  rule.label = QString::fromStdString(persist.label);
  rule.color = QColor(QString::fromStdString(persist.color));
  return rule;
}

common::StateTransitionSeriesPersistConfig ToPersistSeries(
    const StateTransitionSeriesConfig& series) {
  common::StateTransitionSeriesPersistConfig persist;
  persist.channel = series.channel.toStdString();
  persist.field_path = series.field_path.toStdString();
  persist.custom_timestamp_path = series.custom_timestamp_path.toStdString();
  persist.label = series.label.toStdString();
  persist.timestamp_mode = static_cast<int>(series.timestamp_mode);
  persist.enabled = series.enabled;
  persist.mappings.reserve(static_cast<std::size_t>(series.mappings.size()));
  for (const StateMappingRule& rule : series.mappings) {
    persist.mappings.push_back(ToPersistMapping(rule));
  }
  return persist;
}

StateTransitionSeriesConfig FromPersistSeries(
    const common::StateTransitionSeriesPersistConfig& persist) {
  StateTransitionSeriesConfig series;
  series.channel = QString::fromStdString(persist.channel);
  series.field_path = QString::fromStdString(persist.field_path);
  series.custom_timestamp_path =
      QString::fromStdString(persist.custom_timestamp_path);
  series.label = QString::fromStdString(persist.label);
  series.timestamp_mode =
      static_cast<plot::PlotTimestampMode>(persist.timestamp_mode);
  series.enabled = persist.enabled;
  series.mappings.reserve(static_cast<int>(persist.mappings.size()));
  for (const common::StateTransitionMappingPersistConfig& rule :
       persist.mappings) {
    series.mappings.push_back(FromPersistMapping(rule));
  }
  return series;
}

}  // namespace

common::StateTransitionPanelPersistConfig ToPersistConfig(
    const QString& object_name, const StateTransitionPanelConfig& config) {
  common::StateTransitionPanelPersistConfig persist;
  persist.object_name = object_name.toStdString();
  persist.title = config.title.toStdString();
  persist.x_axis_mode = static_cast<int>(config.x_axis_mode);
  persist.x_window_sec = config.x_window_sec;
  persist.fixed_min_time = config.fixed_min_time;
  persist.fixed_max_time = config.fixed_max_time;
  persist.settings_visible = config.settings_visible;
  persist.series.reserve(static_cast<std::size_t>(config.series.size()));
  for (const StateTransitionSeriesConfig& series : config.series) {
    persist.series.push_back(ToPersistSeries(series));
  }
  return persist;
}

StateTransitionPanelConfig FromPersistConfig(
    const common::StateTransitionPanelPersistConfig& persist) {
  StateTransitionPanelConfig config;
  config.title = QString::fromStdString(persist.title);
  config.x_axis_mode = static_cast<StateXAxisMode>(persist.x_axis_mode);
  config.x_window_sec = persist.x_window_sec;
  config.fixed_min_time = persist.fixed_min_time;
  config.fixed_max_time = persist.fixed_max_time;
  config.settings_visible = persist.settings_visible;
  config.series.reserve(static_cast<int>(persist.series.size()));
  for (const common::StateTransitionSeriesPersistConfig& series :
       persist.series) {
    config.series.push_back(FromPersistSeries(series));
  }
  return config;
}

}  // namespace state_transitions
}  // namespace autoviz
