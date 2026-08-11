/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/config_session.hpp"

#include <glog/logging.h>

namespace autoviz {
namespace common {
namespace {

void ReadDisplayFromConfig(const Config& node, DisplayConfig* entry) {
  if (entry == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("Type", &value)) {
    entry->type = value.toStdString();
  }
  if (node.mapGetString("Name", &value)) {
    entry->name = value.toStdString();
  } else {
    entry->name = entry->type;
  }
  if (node.mapGetString("Channel", &value)) {
    entry->channel = value.toStdString();
  }
  bool enabled = true;
  if (node.mapGetBool("Enabled", &enabled)) {
    entry->enabled = enabled;
  }
  entry->properties.clear();
  Config props = node.mapGetChild("Properties");
  if (props.isValid()) {
    for (Config::MapIterator it = props.mapIterator(); it.isValid();
         it.advance()) {
      Config child = it.currentChild();
      if (child.getType() == Config::Value) {
        entry->properties[it.currentKey().toStdString()] =
            child.getValue().toString().toStdString();
      }
    }
  }
  entry->children.clear();
  Config children = node.mapGetChild("Children");
  if (children.isValid()) {
    for (int i = 0; i < children.listLength(); ++i) {
      DisplayConfig child;
      ReadDisplayFromConfig(children.listChildAt(i), &child);
      entry->children.push_back(std::move(child));
    }
  }
}

void WriteDisplayToConfig(const DisplayConfig& display, Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Type", QString::fromStdString(display.type));
  node->mapSetValue("Name", QString::fromStdString(display.name));
  node->mapSetValue("Channel", QString::fromStdString(display.channel));
  node->mapSetValue("Enabled", display.enabled);
  if (!display.properties.empty()) {
    Config props = node->mapMakeChild("Properties");
    for (const auto& prop : display.properties) {
      props.mapSetValue(QString::fromStdString(prop.first),
                        QString::fromStdString(prop.second));
    }
  }
  if (!display.children.empty()) {
    Config children = node->mapMakeChild("Children");
    for (const auto& child : display.children) {
      Config child_node = children.listAppendNew();
      WriteDisplayToConfig(child, &child_node);
    }
  }
}

void ReadViewFromConfig(const Config& node, SavedViewConfig* view) {
  if (view == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("Name", &value)) {
    view->name = value.toStdString();
  }
  if (node.mapGetString("Type", &value)) {
    view->type = value.toStdString();
  }
  node.mapGetFloat("NearClipDistance", &view->near_clip_distance);
  node.mapGetBool("InvertZAxis", &view->invert_z_axis);
  node.mapGetFloat("Yaw", &view->yaw);
  node.mapGetFloat("Pitch", &view->pitch);
  node.mapGetFloat("Distance", &view->distance);
  node.mapGetFloat("FocalShapeSize", &view->focal_shape_size);
  node.mapGetBool("FocalShapeFixedSize", &view->focal_shape_fixed_size);
  node.mapGetFloat("FpsYaw", &view->fps_yaw);
  node.mapGetFloat("FpsPitch", &view->fps_pitch);
  if (node.mapGetString("TargetFrame", &value)) {
    view->target_frame = value.toStdString();
  }
  Config target = node.mapGetChild("Target");
  target.mapGetFloat("X", &view->target_x);
  target.mapGetFloat("Y", &view->target_y);
  target.mapGetFloat("Z", &view->target_z);
  Config fps = node.mapGetChild("FpsPosition");
  fps.mapGetFloat("X", &view->fps_position_x);
  fps.mapGetFloat("Y", &view->fps_position_y);
  fps.mapGetFloat("Z", &view->fps_position_z);
}

void WriteViewToConfig(const SavedViewConfig& view, Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Name", QString::fromStdString(view.name));
  node->mapSetValue("Type", QString::fromStdString(view.type));
  node->mapSetValue("NearClipDistance", view.near_clip_distance);
  node->mapSetValue("InvertZAxis", view.invert_z_axis);
  node->mapSetValue("Yaw", view.yaw);
  node->mapSetValue("Pitch", view.pitch);
  node->mapSetValue("Distance", view.distance);
  node->mapSetValue("FocalShapeSize", view.focal_shape_size);
  node->mapSetValue("FocalShapeFixedSize", view.focal_shape_fixed_size);
  node->mapSetValue("FpsYaw", view.fps_yaw);
  node->mapSetValue("FpsPitch", view.fps_pitch);
  if (!view.target_frame.empty()) {
    node->mapSetValue("TargetFrame",
                      QString::fromStdString(view.target_frame));
  }
  Config target = node->mapMakeChild("Target");
  target.mapSetValue("X", view.target_x);
  target.mapSetValue("Y", view.target_y);
  target.mapSetValue("Z", view.target_z);
  Config fps = node->mapMakeChild("FpsPosition");
  fps.mapSetValue("X", view.fps_position_x);
  fps.mapSetValue("Y", view.fps_position_y);
  fps.mapSetValue("Z", view.fps_position_z);
}

std::string MapRvizTransformerClass(const std::string& rviz_class) {
  if (rviz_class.find("Identity") != std::string::npos) {
    return "autoviz/Identity";
  }
  if (rviz_class.find("TF") != std::string::npos ||
      rviz_class.find("Tf") != std::string::npos) {
    return "autoviz/AutolinkTf";
  }
  return "autoviz/AutolinkTf";
}

void ReadPlotSeriesFromConfig(const Config& node,
                              PlotSeriesPersistConfig* series) {
  if (series == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("Channel", &value)) {
    series->channel = value.toStdString();
  }
  if (node.mapGetString("FieldPath", &value)) {
    series->field_path = value.toStdString();
  }
  if (node.mapGetString("XFieldPath", &value)) {
    series->x_field_path = value.toStdString();
  }
  if (node.mapGetString("CustomTimestampPath", &value)) {
    series->custom_timestamp_path = value.toStdString();
  }
  if (node.mapGetString("Label", &value)) {
    series->label = value.toStdString();
  }
  if (node.mapGetString("Color", &value)) {
    series->color = value.toStdString();
  }
  if (node.mapGetString("LineSize", &value)) {
    series->line_size = value.toStdString();
  }
  node.mapGetBool("ShowLine", &series->show_line);
  node.mapGetInt("TimestampMode", &series->timestamp_mode);
  node.mapGetBool("Enabled", &series->enabled);
}

void WritePlotSeriesToConfig(const PlotSeriesPersistConfig& series,
                             Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Channel", QString::fromStdString(series.channel));
  node->mapSetValue("FieldPath", QString::fromStdString(series.field_path));
  node->mapSetValue("XFieldPath", QString::fromStdString(series.x_field_path));
  node->mapSetValue("CustomTimestampPath",
                    QString::fromStdString(series.custom_timestamp_path));
  node->mapSetValue("Label", QString::fromStdString(series.label));
  node->mapSetValue("Color", QString::fromStdString(series.color));
  node->mapSetValue("LineSize", QString::fromStdString(series.line_size));
  node->mapSetValue("ShowLine", series.show_line);
  node->mapSetValue("TimestampMode", series.timestamp_mode);
  node->mapSetValue("Enabled", series.enabled);
}

void ReadPlotPanelFromConfig(const Config& node,
                             PlotPanelPersistConfig* panel) {
  if (panel == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("ObjectName", &value)) {
    panel->object_name = value.toStdString();
  }
  if (node.mapGetString("Title", &value)) {
    panel->title = value.toStdString();
  }
  node.mapGetInt("XAxisMode", &panel->x_axis_mode);
  node.mapGetInt("MessagePathMode", &panel->message_path_mode);
  node.mapGetBool("SyncWithOtherPlots", &panel->sync_with_other_plots);
  node.mapGetBool("ShowLegendValues", &panel->show_legend_values);
  node.mapGetBool("SettingsVisible", &panel->settings_visible);
  node.mapGetInt("SettingsWidth", &panel->settings_width);
  node.mapGetBool("LockAxisScales", &panel->lock_axis_scales);
  float x_window_sec = static_cast<float>(panel->x_window_sec);
  if (node.mapGetFloat("XWindowSec", &x_window_sec)) {
    panel->x_window_sec = x_window_sec;
  }
  panel->series.clear();
  Config series_list = node.mapGetChild("Series");
  for (int i = 0; i < series_list.listLength(); ++i) {
    PlotSeriesPersistConfig series;
    ReadPlotSeriesFromConfig(series_list.listChildAt(i), &series);
    panel->series.push_back(std::move(series));
  }
}

void ReadVariableFromConfig(const Config& node, VariablePersistConfig* variable) {
  if (variable == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("Name", &value)) {
    variable->name = value.toStdString();
  }
  if (node.mapGetString("Type", &value)) {
    variable->type = value.toStdString();
  }
  if (node.mapGetString("Value", &value)) {
    variable->value = value.toStdString();
  }
}

void WriteVariableToConfig(const VariablePersistConfig& variable, Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Name", QString::fromStdString(variable.name));
  node->mapSetValue("Type", QString::fromStdString(variable.type));
  node->mapSetValue("Value", QString::fromStdString(variable.value));
}

void ReadStateTransitionMappingFromConfig(
    const Config& node, StateTransitionMappingPersistConfig* mapping) {
  if (mapping == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  node.mapGetInt("Kind", &mapping->kind);
  if (node.mapGetString("MatchValue", &value)) {
    mapping->match_value = value.toStdString();
  }
  float range_min = static_cast<float>(mapping->range_min);
  float range_max = static_cast<float>(mapping->range_max);
  if (node.mapGetFloat("RangeMin", &range_min)) {
    mapping->range_min = range_min;
  }
  if (node.mapGetFloat("RangeMax", &range_max)) {
    mapping->range_max = range_max;
  }
  if (node.mapGetString("Label", &value)) {
    mapping->label = value.toStdString();
  }
  if (node.mapGetString("Color", &value)) {
    mapping->color = value.toStdString();
  }
}

void WriteStateTransitionMappingToConfig(
    const StateTransitionMappingPersistConfig& mapping, Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Kind", mapping.kind);
  node->mapSetValue("MatchValue", QString::fromStdString(mapping.match_value));
  node->mapSetValue("RangeMin", mapping.range_min);
  node->mapSetValue("RangeMax", mapping.range_max);
  node->mapSetValue("Label", QString::fromStdString(mapping.label));
  node->mapSetValue("Color", QString::fromStdString(mapping.color));
}

void ReadStateTransitionSeriesFromConfig(
    const Config& node, StateTransitionSeriesPersistConfig* series) {
  if (series == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("Channel", &value)) {
    series->channel = value.toStdString();
  }
  if (node.mapGetString("FieldPath", &value)) {
    series->field_path = value.toStdString();
  }
  if (node.mapGetString("CustomTimestampPath", &value)) {
    series->custom_timestamp_path = value.toStdString();
  }
  if (node.mapGetString("Label", &value)) {
    series->label = value.toStdString();
  }
  node.mapGetInt("TimestampMode", &series->timestamp_mode);
  node.mapGetBool("Enabled", &series->enabled);
  series->mappings.clear();
  Config mappings = node.mapGetChild("Mappings");
  for (int i = 0; i < mappings.listLength(); ++i) {
    StateTransitionMappingPersistConfig mapping;
    ReadStateTransitionMappingFromConfig(mappings.listChildAt(i), &mapping);
    series->mappings.push_back(std::move(mapping));
  }
}

void WriteStateTransitionSeriesToConfig(
    const StateTransitionSeriesPersistConfig& series, Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Channel", QString::fromStdString(series.channel));
  node->mapSetValue("FieldPath", QString::fromStdString(series.field_path));
  node->mapSetValue("CustomTimestampPath",
                    QString::fromStdString(series.custom_timestamp_path));
  node->mapSetValue("Label", QString::fromStdString(series.label));
  node->mapSetValue("TimestampMode", series.timestamp_mode);
  node->mapSetValue("Enabled", series.enabled);
  if (!series.mappings.empty()) {
    Config mappings = node->mapMakeChild("Mappings");
    for (const auto& mapping : series.mappings) {
      Config mapping_node = mappings.listAppendNew();
      WriteStateTransitionMappingToConfig(mapping, &mapping_node);
    }
  }
}

void ReadStateTransitionPanelFromConfig(
    const Config& node, StateTransitionPanelPersistConfig* panel) {
  if (panel == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("ObjectName", &value)) {
    panel->object_name = value.toStdString();
  }
  if (node.mapGetString("Title", &value)) {
    panel->title = value.toStdString();
  }
  node.mapGetInt("XAxisMode", &panel->x_axis_mode);
  float x_window_sec = static_cast<float>(panel->x_window_sec);
  if (node.mapGetFloat("XWindowSec", &x_window_sec)) {
    panel->x_window_sec = x_window_sec;
  }
  float fixed_min = static_cast<float>(panel->fixed_min_time);
  float fixed_max = static_cast<float>(panel->fixed_max_time);
  if (node.mapGetFloat("FixedMinTime", &fixed_min)) {
    panel->fixed_min_time = fixed_min;
  }
  if (node.mapGetFloat("FixedMaxTime", &fixed_max)) {
    panel->fixed_max_time = fixed_max;
  }
  node.mapGetBool("SettingsVisible", &panel->settings_visible);
  panel->series.clear();
  Config series_list = node.mapGetChild("Series");
  for (int i = 0; i < series_list.listLength(); ++i) {
    StateTransitionSeriesPersistConfig series;
    ReadStateTransitionSeriesFromConfig(series_list.listChildAt(i), &series);
    panel->series.push_back(std::move(series));
  }
}

void WriteStateTransitionPanelToConfig(
    const StateTransitionPanelPersistConfig& panel, Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("ObjectName", QString::fromStdString(panel.object_name));
  node->mapSetValue("Title", QString::fromStdString(panel.title));
  node->mapSetValue("XAxisMode", panel.x_axis_mode);
  node->mapSetValue("XWindowSec", panel.x_window_sec);
  node->mapSetValue("FixedMinTime", panel.fixed_min_time);
  node->mapSetValue("FixedMaxTime", panel.fixed_max_time);
  node->mapSetValue("SettingsVisible", panel.settings_visible);
  if (!panel.series.empty()) {
    Config series_list = node->mapMakeChild("Series");
    for (const auto& series : panel.series) {
      Config series_node = series_list.listAppendNew();
      WriteStateTransitionSeriesToConfig(series, &series_node);
    }
  }
}

void ReadPublishPresetFromConfig(const Config& node,
                                 PublishPresetPersistConfig* preset) {
  if (preset == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("Name", &value)) {
    preset->name = value.toStdString();
  }
  if (node.mapGetString("Channel", &value)) {
    preset->channel = value.toStdString();
  }
  if (node.mapGetString("MessageType", &value)) {
    preset->message_type = value.toStdString();
  }
  if (node.mapGetString("MessageJson", &value)) {
    preset->message_json = value.toStdString();
  }
  node.mapGetBool("LoopPublish", &preset->loop_publish);
  float publish_rate = static_cast<float>(preset->publish_rate_hz);
  if (node.mapGetFloat("PublishRateHz", &publish_rate)) {
    preset->publish_rate_hz = publish_rate;
  }
  if (node.mapGetString("ButtonLabel", &value)) {
    preset->button_label = value.toStdString();
  }
  if (node.mapGetString("ButtonTooltip", &value)) {
    preset->button_tooltip = value.toStdString();
  }
  if (node.mapGetString("ButtonColor", &value)) {
    preset->button_color = value.toStdString();
  }
}

void WritePublishPresetToConfig(const PublishPresetPersistConfig& preset,
                                Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Name", QString::fromStdString(preset.name));
  node->mapSetValue("Channel", QString::fromStdString(preset.channel));
  node->mapSetValue("MessageType", QString::fromStdString(preset.message_type));
  node->mapSetValue("MessageJson", QString::fromStdString(preset.message_json));
  node->mapSetValue("LoopPublish", preset.loop_publish);
  node->mapSetValue("PublishRateHz", preset.publish_rate_hz);
  node->mapSetValue("ButtonLabel", QString::fromStdString(preset.button_label));
  node->mapSetValue("ButtonTooltip",
                    QString::fromStdString(preset.button_tooltip));
  node->mapSetValue("ButtonColor", QString::fromStdString(preset.button_color));
}

void ReadPublishEntryFromConfig(const Config& node,
                                PublishEntryPersistConfig* entry) {
  if (entry == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("Id", &value)) {
    entry->id = value.toStdString();
  }
  if (node.mapGetString("Channel", &value)) {
    entry->channel = value.toStdString();
  }
  if (node.mapGetString("MessageType", &value)) {
    entry->message_type = value.toStdString();
  }
  float publish_rate = static_cast<float>(entry->publish_rate_hz);
  if (node.mapGetFloat("PublishRateHz", &publish_rate)) {
    entry->publish_rate_hz = publish_rate;
  }
  if (node.mapGetString("MessageJson", &value)) {
    entry->message_json = value.toStdString();
  }
  node.mapGetBool("Publishing", &entry->publishing);
}

void WritePublishEntryToConfig(const PublishEntryPersistConfig& entry,
                               Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Id", QString::fromStdString(entry.id));
  node->mapSetValue("Channel", QString::fromStdString(entry.channel));
  node->mapSetValue("MessageType", QString::fromStdString(entry.message_type));
  node->mapSetValue("PublishRateHz", entry.publish_rate_hz);
  node->mapSetValue("MessageJson", QString::fromStdString(entry.message_json));
  node->mapSetValue("Publishing", entry.publishing);
}

void ReadPublishPanelFromConfig(const Config& node,
                                PublishPanelPersistConfig* panel) {
  if (panel == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("ObjectName", &value)) {
    panel->object_name = value.toStdString();
  }
  if (node.mapGetString("Title", &value)) {
    panel->title = value.toStdString();
  }
  if (node.mapGetString("Channel", &value)) {
    panel->channel = value.toStdString();
  }
  if (node.mapGetString("MessageType", &value)) {
    panel->message_type = value.toStdString();
  }
  if (node.mapGetString("MessageJson", &value)) {
    panel->message_json = value.toStdString();
  }
  node.mapGetBool("EditingMode", &panel->editing_mode);
  node.mapGetBool("LoopPublish", &panel->loop_publish);
  float publish_rate = static_cast<float>(panel->publish_rate_hz);
  if (node.mapGetFloat("PublishRateHz", &publish_rate)) {
    panel->publish_rate_hz = publish_rate;
  }
  if (node.mapGetString("ButtonLabel", &value)) {
    panel->button_label = value.toStdString();
  }
  if (node.mapGetString("ButtonTooltip", &value)) {
    panel->button_tooltip = value.toStdString();
  }
  if (node.mapGetString("ButtonColor", &value)) {
    panel->button_color = value.toStdString();
  }
  node.mapGetBool("SettingsVisible", &panel->settings_visible);
  if (node.mapGetString("ActivePresetName", &value)) {
    panel->active_preset_name = value.toStdString();
  }
  panel->saved_presets.clear();
  Config presets = node.mapGetChild("SavedPresets");
  for (int i = 0; i < presets.listLength(); ++i) {
    PublishPresetPersistConfig preset;
    ReadPublishPresetFromConfig(presets.listChildAt(i), &preset);
    panel->saved_presets.push_back(std::move(preset));
  }
  panel->custom_channels.clear();
  Config custom_channels = node.mapGetChild("CustomChannels");
  for (int i = 0; i < custom_channels.listLength(); ++i) {
    panel->custom_channels.push_back(
        custom_channels.listChildAt(i).getValue().toString().toStdString());
  }
  panel->publishers.clear();
  Config publishers = node.mapGetChild("Publishers");
  for (int i = 0; i < publishers.listLength(); ++i) {
    PublishEntryPersistConfig entry;
    ReadPublishEntryFromConfig(publishers.listChildAt(i), &entry);
    panel->publishers.push_back(std::move(entry));
  }
  int selected_index = panel->selected_publisher_index;
  if (node.mapGetInt("SelectedPublisherIndex", &selected_index)) {
    panel->selected_publisher_index = selected_index;
  }
}

void WritePublishPanelToConfig(const PublishPanelPersistConfig& panel,
                               Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("ObjectName", QString::fromStdString(panel.object_name));
  node->mapSetValue("Title", QString::fromStdString(panel.title));
  node->mapSetValue("Channel", QString::fromStdString(panel.channel));
  node->mapSetValue("MessageType", QString::fromStdString(panel.message_type));
  node->mapSetValue("MessageJson", QString::fromStdString(panel.message_json));
  node->mapSetValue("EditingMode", panel.editing_mode);
  node->mapSetValue("LoopPublish", panel.loop_publish);
  node->mapSetValue("PublishRateHz", panel.publish_rate_hz);
  node->mapSetValue("ButtonLabel", QString::fromStdString(panel.button_label));
  node->mapSetValue("ButtonTooltip",
                    QString::fromStdString(panel.button_tooltip));
  node->mapSetValue("ButtonColor", QString::fromStdString(panel.button_color));
  node->mapSetValue("SettingsVisible", panel.settings_visible);
  node->mapSetValue("ActivePresetName",
                    QString::fromStdString(panel.active_preset_name));
  if (!panel.saved_presets.empty()) {
    Config presets = node->mapMakeChild("SavedPresets");
    for (const auto& preset : panel.saved_presets) {
      Config preset_node = presets.listAppendNew();
      WritePublishPresetToConfig(preset, &preset_node);
    }
  }
  if (!panel.custom_channels.empty()) {
    Config custom_channels = node->mapMakeChild("CustomChannels");
    for (const auto& channel : panel.custom_channels) {
      custom_channels.listAppendNew().setValue(QString::fromStdString(channel));
    }
  }
  if (!panel.publishers.empty()) {
    Config publishers = node->mapMakeChild("Publishers");
    for (const auto& entry : panel.publishers) {
      Config entry_node = publishers.listAppendNew();
      WritePublishEntryToConfig(entry, &entry_node);
    }
  }
  node->mapSetValue("SelectedPublisherIndex", panel.selected_publisher_index);
}

void WritePlotPanelToConfig(const PlotPanelPersistConfig& panel, Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("ObjectName", QString::fromStdString(panel.object_name));
  node->mapSetValue("Title", QString::fromStdString(panel.title));
  node->mapSetValue("XAxisMode", panel.x_axis_mode);
  node->mapSetValue("MessagePathMode", panel.message_path_mode);
  node->mapSetValue("SyncWithOtherPlots", panel.sync_with_other_plots);
  node->mapSetValue("ShowLegendValues", panel.show_legend_values);
  node->mapSetValue("SettingsVisible", panel.settings_visible);
  node->mapSetValue("SettingsWidth", panel.settings_width);
  node->mapSetValue("LockAxisScales", panel.lock_axis_scales);
  node->mapSetValue("XWindowSec", panel.x_window_sec);
  if (!panel.series.empty()) {
    Config series_list = node->mapMakeChild("Series");
    for (const auto& series : panel.series) {
      Config series_node = series_list.listAppendNew();
      WritePlotSeriesToConfig(series, &series_node);
    }
  }
}

void ReadImageOverlayFromConfig(const Config& node,
                                ImageOverlayPersistConfig* overlay) {
  if (overlay == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("Channel", &value)) {
    overlay->channel = value.toStdString();
  }
  float opacity = static_cast<float>(overlay->opacity);
  if (node.mapGetFloat("Opacity", &opacity)) {
    overlay->opacity = opacity;
  }
  node.mapGetInt("BlendMode", &overlay->blend_mode);
  node.mapGetInt("PixelAlpha", &overlay->pixel_alpha);
  node.mapGetBool("Enabled", &overlay->enabled);
}

void WriteImageOverlayToConfig(const ImageOverlayPersistConfig& overlay,
                               Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("Channel", QString::fromStdString(overlay.channel));
  node->mapSetValue("Opacity", overlay.opacity);
  node->mapSetValue("BlendMode", overlay.blend_mode);
  node->mapSetValue("PixelAlpha", overlay.pixel_alpha);
  node->mapSetValue("Enabled", overlay.enabled);
}

void ReadImagePanelFromConfig(const Config& node,
                              ImagePanelPersistConfig* panel) {
  if (panel == nullptr || !node.isValid()) {
    return;
  }
  QString value;
  if (node.mapGetString("ObjectName", &value)) {
    panel->object_name = value.toStdString();
  }
  if (node.mapGetString("Title", &value)) {
    panel->title = value.toStdString();
  }
  if (node.mapGetString("ImageChannel", &value)) {
    panel->image_channel = value.toStdString();
  }
  if (node.mapGetString("CalibrationChannel", &value)) {
    panel->calibration_channel = value.toStdString();
  }
  node.mapGetBool("StrictTimeSync", &panel->strict_time_sync);
  node.mapGetBool("FlipHorizontal", &panel->flip_horizontal);
  node.mapGetBool("FlipVertical", &panel->flip_vertical);
  node.mapGetInt("Rotation", &panel->rotation);
  node.mapGetInt("ColorMode", &panel->color_mode);
  float color_min = static_cast<float>(panel->color_min);
  float color_max = static_cast<float>(panel->color_max);
  if (node.mapGetFloat("ColorMin", &color_min)) {
    panel->color_min = color_min;
  }
  if (node.mapGetFloat("ColorMax", &color_max)) {
    panel->color_max = color_max;
  }
  if (node.mapGetString("BackgroundColor", &value)) {
    panel->background_color = value.toStdString();
  }
  float label_scale = static_cast<float>(panel->label_scale);
  if (node.mapGetFloat("LabelScale", &label_scale)) {
    panel->label_scale = label_scale;
  }
  if (node.mapGetString("ClickPublishChannel", &value)) {
    panel->click_publish_channel = value.toStdString();
  }
  if (node.mapGetString("HoverPublishChannel", &value)) {
    panel->hover_publish_channel = value.toStdString();
  }
  node.mapGetBool("EnableUndistort", &panel->enable_undistort);
  node.mapGetBool("SettingsVisible", &panel->settings_visible);
  panel->overlays.clear();
  Config overlays = node.mapGetChild("Overlays");
  for (int i = 0; i < overlays.listLength(); ++i) {
    ImageOverlayPersistConfig overlay;
    ReadImageOverlayFromConfig(overlays.listChildAt(i), &overlay);
    panel->overlays.push_back(std::move(overlay));
  }
  panel->annotation_channels.clear();
  Config annotations = node.mapGetChild("AnnotationChannels");
  for (int i = 0; i < annotations.listLength(); ++i) {
    panel->annotation_channels.push_back(
        annotations.listChildAt(i).getValue().toString().toStdString());
  }
  panel->marker_channels.clear();
  Config markers = node.mapGetChild("MarkerChannels");
  for (int i = 0; i < markers.listLength(); ++i) {
    panel->marker_channels.push_back(
        markers.listChildAt(i).getValue().toString().toStdString());
  }
}

void WriteImagePanelToConfig(const ImagePanelPersistConfig& panel, Config* node) {
  if (node == nullptr) {
    return;
  }
  node->mapSetValue("ObjectName", QString::fromStdString(panel.object_name));
  node->mapSetValue("Title", QString::fromStdString(panel.title));
  node->mapSetValue("ImageChannel", QString::fromStdString(panel.image_channel));
  node->mapSetValue("CalibrationChannel",
                    QString::fromStdString(panel.calibration_channel));
  node->mapSetValue("StrictTimeSync", panel.strict_time_sync);
  node->mapSetValue("FlipHorizontal", panel.flip_horizontal);
  node->mapSetValue("FlipVertical", panel.flip_vertical);
  node->mapSetValue("Rotation", panel.rotation);
  node->mapSetValue("ColorMode", panel.color_mode);
  node->mapSetValue("ColorMin", panel.color_min);
  node->mapSetValue("ColorMax", panel.color_max);
  node->mapSetValue("BackgroundColor",
                    QString::fromStdString(panel.background_color));
  node->mapSetValue("LabelScale", panel.label_scale);
  node->mapSetValue("ClickPublishChannel",
                    QString::fromStdString(panel.click_publish_channel));
  node->mapSetValue("HoverPublishChannel",
                    QString::fromStdString(panel.hover_publish_channel));
  node->mapSetValue("EnableUndistort", panel.enable_undistort);
  node->mapSetValue("SettingsVisible", panel.settings_visible);
  if (!panel.overlays.empty()) {
    Config overlays = node->mapMakeChild("Overlays");
    for (const auto& overlay : panel.overlays) {
      Config overlay_node = overlays.listAppendNew();
      WriteImageOverlayToConfig(overlay, &overlay_node);
    }
  }
  if (!panel.annotation_channels.empty()) {
    Config annotations = node->mapMakeChild("AnnotationChannels");
    for (const std::string& channel : panel.annotation_channels) {
      annotations.listAppendNew().setValue(QString::fromStdString(channel));
    }
  }
  if (!panel.marker_channels.empty()) {
    Config markers = node->mapMakeChild("MarkerChannels");
    for (const std::string& channel : panel.marker_channels) {
      markers.listAppendNew().setValue(QString::fromStdString(channel));
    }
  }
}

bool SessionConfigFromNativeConfig(const Config& root, SessionConfig* config) {
  if (config == nullptr) {
    return false;
  }
  Config global = root.mapGetChild("Global");
  if (global.isValid()) {
    QString value;
    if (global.mapGetString("FixedFrame", &value)) {
      config->fixed_frame = value.toStdString();
    }
    global.mapGetBool("ShowGrid", &config->show_grid);
    global.mapGetInt("FrameRate", &config->frame_rate);
    if (global.mapGetString("BackgroundColor", &value)) {
      config->background_color = value.toStdString();
    }
    if (global.mapGetString("ViewController", &value)) {
      config->view_controller = value.toStdString();
    }
    if (global.mapGetString("RenderBackend", &value)) {
      config->render_backend = value.toStdString();
    }
    if (global.mapGetString("ActiveTool", &value)) {
      config->active_tool = value.toStdString();
    }
    if (global.mapGetString("Transformer", &value)) {
      config->transformer_id = value.toStdString();
    }
  }

  Config window = root.mapGetChild("Window");
  if (window.isValid()) {
    QString value;
    if (window.mapGetString("State", &value)) {
      config->window_state_b64 = value.toStdString();
    }
    if (window.mapGetString("MainPanelState", &value)) {
      config->main_panel_state_b64 = value.toStdString();
    }
    if (window.mapGetString("Geometry", &value)) {
      config->window_geometry_b64 = value.toStdString();
    }
    window.mapGetBool("HideLeftDock", &config->hide_left_dock);
    window.mapGetBool("HideRightDock", &config->hide_right_dock);
    window.mapGetInt("X", &config->window_x);
    window.mapGetInt("Y", &config->window_y);
    window.mapGetInt("Width", &config->window_width);
    window.mapGetInt("Height", &config->window_height);
    window.mapGetBool("PlotSettingsVisible", &config->plot_settings_visible);
    config->visible_panels.clear();
    Config visible = window.mapGetChild("VisiblePanels");
    for (int i = 0; i < visible.listLength(); ++i) {
      config->visible_panels.push_back(
          visible.listChildAt(i).getValue().toString().toStdString());
    }
    config->panel_layouts.clear();
    Config panels = window.mapGetChild("Panels");
    for (int i = 0; i < panels.listLength(); ++i) {
      PanelLayoutConfig panel;
      Config node = panels.listChildAt(i);
      QString name;
      if (node.mapGetString("Name", &name)) {
        panel.object_name = name.toStdString();
      }
      node.mapGetBool("Collapsed", &panel.collapsed);
      config->panel_layouts.push_back(std::move(panel));
    }
  }

  config->displays.clear();
  Config displays = root.mapGetChild("Displays");
  for (int i = 0; i < displays.listLength(); ++i) {
    DisplayConfig entry;
    ReadDisplayFromConfig(displays.listChildAt(i), &entry);
    config->displays.push_back(std::move(entry));
  }

  Config current = root.mapGetChild("CurrentView");
  if (current.isValid()) {
    config->has_current_view = true;
    config->current_view.name = "Current";
    ReadViewFromConfig(current, &config->current_view);
    config->view_controller = config->current_view.type;
  }

  config->views.clear();
  Config views = root.mapGetChild("Views");
  for (int i = 0; i < views.listLength(); ++i) {
    SavedViewConfig view;
    ReadViewFromConfig(views.listChildAt(i), &view);
    config->views.push_back(std::move(view));
  }

  config->toolbar_tools.clear();
  Config toolbar = root.mapGetChild("Toolbar");
  for (int i = 0; i < toolbar.listLength(); ++i) {
    config->toolbar_tools.push_back(
        toolbar.listChildAt(i).getValue().toString().toStdString());
  }

  config->tools.clear();
  Config tools = root.mapGetChild("Tools");
  for (int i = 0; i < tools.listLength(); ++i) {
    ToolConfig tool;
    Config node = tools.listChildAt(i);
    QString id;
    if (node.mapGetString("Id", &id)) {
      tool.id = id.toStdString();
    }
    Config props = node.mapGetChild("Properties");
    for (Config::MapIterator it = props.mapIterator(); it.isValid();
         it.advance()) {
      tool.properties[it.currentKey().toStdString()] =
          it.currentChild().getValue().toString().toStdString();
    }
    config->tools.push_back(std::move(tool));
  }

  config->plot_panels.clear();
  Config plot_panels = root.mapGetChild("PlotPanels");
  for (int i = 0; i < plot_panels.listLength(); ++i) {
    PlotPanelPersistConfig panel;
    ReadPlotPanelFromConfig(plot_panels.listChildAt(i), &panel);
    config->plot_panels.push_back(std::move(panel));
  }

  config->variables.clear();
  Config variables = root.mapGetChild("Variables");
  for (int i = 0; i < variables.listLength(); ++i) {
    VariablePersistConfig variable;
    ReadVariableFromConfig(variables.listChildAt(i), &variable);
    config->variables.push_back(std::move(variable));
  }
  config->image_panels.clear();
  Config image_panels = root.mapGetChild("ImagePanels");
  for (int i = 0; i < image_panels.listLength(); ++i) {
    ImagePanelPersistConfig panel;
    ReadImagePanelFromConfig(image_panels.listChildAt(i), &panel);
    config->image_panels.push_back(std::move(panel));
  }

  config->state_transition_panels.clear();
  Config state_panels = root.mapGetChild("StateTransitionPanels");
  for (int i = 0; i < state_panels.listLength(); ++i) {
    StateTransitionPanelPersistConfig panel;
    ReadStateTransitionPanelFromConfig(state_panels.listChildAt(i), &panel);
    config->state_transition_panels.push_back(std::move(panel));
  }

  config->publish_panels.clear();
  Config publish_panels = root.mapGetChild("PublishPanels");
  for (int i = 0; i < publish_panels.listLength(); ++i) {
    PublishPanelPersistConfig panel;
    ReadPublishPanelFromConfig(publish_panels.listChildAt(i), &panel);
    config->publish_panels.push_back(std::move(panel));
  }
  return true;
}

bool SessionConfigFromRvizConfig(const Config& root, SessionConfig* config) {
  if (config == nullptr) {
    return false;
  }
  Config manager = root.mapGetChild("Visualization Manager");
  if (!manager.isValid()) {
    return false;
  }

  Config global = manager.mapGetChild("Global Options");
  if (global.isValid()) {
    QString value;
    if (global.mapGetString("Fixed Frame", &value)) {
      config->fixed_frame = value.toStdString();
      if (!config->fixed_frame.empty() && config->fixed_frame.front() == '/') {
        config->fixed_frame.erase(config->fixed_frame.begin());
      }
    }
    if (global.mapGetString("Background Color", &value)) {
      config->background_color = value.toStdString();
    }
    global.mapGetInt("Frame Rate", &config->frame_rate);
  }

  Config transformation = manager.mapGetChild("Transformation");
  if (transformation.isValid()) {
    Config current = transformation.mapGetChild("Current");
    QString class_id;
    if (current.mapGetString("Class", &class_id)) {
      config->transformer_id = MapRvizTransformerClass(class_id.toStdString());
    }
  }

  Config window = root.mapGetChild("Window Geometry");
  if (window.isValid()) {
    window.mapGetBool("Hide Left Dock", &config->hide_left_dock);
    window.mapGetBool("Hide Right Dock", &config->hide_right_dock);
    QString state;
    if (window.mapGetString("QMainWindow State", &state)) {
      config->window_state_b64 = state.toStdString();
    }
    window.mapGetInt("Width", &config->window_width);
    window.mapGetInt("Height", &config->window_height);
    window.mapGetInt("X", &config->window_x);
    window.mapGetInt("Y", &config->window_y);
  }
  return true;
}

}  // namespace

void SessionConfigToConfig(const SessionConfig& session, Config* root) {
  if (root == nullptr) {
    return;
  }
  root->setType(Config::Map);

  Config global = root->mapMakeChild("Global");
  global.mapSetValue("FixedFrame", QString::fromStdString(session.fixed_frame));
  global.mapSetValue("ShowGrid", session.show_grid);
  global.mapSetValue("FrameRate", session.frame_rate);
  global.mapSetValue("BackgroundColor",
                     QString::fromStdString(session.background_color));
  global.mapSetValue("ViewController",
                     QString::fromStdString(session.view_controller));
  global.mapSetValue("RenderBackend",
                     QString::fromStdString(session.render_backend));
  global.mapSetValue("ActiveTool",
                     QString::fromStdString(session.active_tool));
  global.mapSetValue("Transformer",
                     QString::fromStdString(session.transformer_id));

  Config window = root->mapMakeChild("Window");
  if (!session.window_state_b64.empty()) {
    window.mapSetValue("State", QString::fromStdString(session.window_state_b64));
  }
  if (!session.main_panel_state_b64.empty()) {
    window.mapSetValue("MainPanelState",
                       QString::fromStdString(session.main_panel_state_b64));
  }
  if (!session.window_geometry_b64.empty()) {
    window.mapSetValue("Geometry",
                       QString::fromStdString(session.window_geometry_b64));
  }
  window.mapSetValue("HideLeftDock", session.hide_left_dock);
  window.mapSetValue("HideRightDock", session.hide_right_dock);
  window.mapSetValue("PlotSettingsVisible", session.plot_settings_visible);
  if (session.window_x >= 0) {
    window.mapSetValue("X", session.window_x);
  }
  if (session.window_y >= 0) {
    window.mapSetValue("Y", session.window_y);
  }
  if (session.window_width > 0) {
    window.mapSetValue("Width", session.window_width);
  }
  if (session.window_height > 0) {
    window.mapSetValue("Height", session.window_height);
  }
  if (!session.visible_panels.empty()) {
    Config visible = window.mapMakeChild("VisiblePanels");
    for (const auto& panel : session.visible_panels) {
      visible.listAppendNew().setValue(QString::fromStdString(panel));
    }
  }
  if (!session.panel_layouts.empty()) {
    Config panels = window.mapMakeChild("Panels");
    for (const auto& panel : session.panel_layouts) {
      Config node = panels.listAppendNew();
      node.mapSetValue("Name", QString::fromStdString(panel.object_name));
      if (panel.collapsed) {
        node.mapSetValue("Collapsed", true);
      }
    }
  }

  Config displays = root->mapMakeChild("Displays");
  for (const auto& display : session.displays) {
    Config node = displays.listAppendNew();
    WriteDisplayToConfig(display, &node);
  }

  if (session.has_current_view) {
    Config current_view = root->mapMakeChild("CurrentView");
    WriteViewToConfig(session.current_view, &current_view);
  }

  Config views = root->mapMakeChild("Views");
  for (const auto& view : session.views) {
    Config node = views.listAppendNew();
    WriteViewToConfig(view, &node);
  }

  Config toolbar = root->mapMakeChild("Toolbar");
  for (const auto& tool_id : session.toolbar_tools) {
    toolbar.listAppendNew().setValue(QString::fromStdString(tool_id));
  }

  Config tools = root->mapMakeChild("Tools");
  for (const auto& tool : session.tools) {
    Config node = tools.listAppendNew();
    node.mapSetValue("Id", QString::fromStdString(tool.id));
    if (!tool.properties.empty()) {
      Config props = node.mapMakeChild("Properties");
      for (const auto& prop : tool.properties) {
        props.mapSetValue(QString::fromStdString(prop.first),
                          QString::fromStdString(prop.second));
      }
    }
  }

  if (!session.plot_panels.empty()) {
    Config plot_panels = root->mapMakeChild("PlotPanels");
    for (const auto& panel : session.plot_panels) {
      Config node = plot_panels.listAppendNew();
      WritePlotPanelToConfig(panel, &node);
    }
  }
  if (!session.image_panels.empty()) {
    Config image_panels = root->mapMakeChild("ImagePanels");
    for (const auto& panel : session.image_panels) {
      Config node = image_panels.listAppendNew();
      WriteImagePanelToConfig(panel, &node);
    }
  }
  if (!session.variables.empty()) {
    Config variables = root->mapMakeChild("Variables");
    for (const auto& variable : session.variables) {
      Config node = variables.listAppendNew();
      WriteVariableToConfig(variable, &node);
    }
  }
  if (!session.state_transition_panels.empty()) {
    Config state_panels = root->mapMakeChild("StateTransitionPanels");
    for (const auto& panel : session.state_transition_panels) {
      Config node = state_panels.listAppendNew();
      WriteStateTransitionPanelToConfig(panel, &node);
    }
  }
  if (!session.publish_panels.empty()) {
    Config publish_panels = root->mapMakeChild("PublishPanels");
    for (const auto& panel : session.publish_panels) {
      Config node = publish_panels.listAppendNew();
      WritePublishPanelToConfig(panel, &node);
    }
  }
}

bool SessionConfigFromConfig(const Config& root, SessionConfig* config) {
  if (config == nullptr || !root.isValid()) {
    return false;
  }
  if (root.mapGetChild("Visualization Manager").isValid()) {
    *config = SessionConfig{};
    if (!SessionConfigFromRvizConfig(root, config)) {
      return false;
    }
    LOG(INFO) << "Imported RViz config via Config tree";
    return true;
  }
  return SessionConfigFromNativeConfig(root, config);
}

}  // namespace common
}  // namespace autoviz
