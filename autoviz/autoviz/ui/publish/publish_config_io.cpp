/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_config_io.hpp"

#include <QColor>

namespace autoviz {
namespace publish_panel {
namespace {

common::PublishPresetPersistConfig ToPersistPreset(const PublishPreset& preset) {
  common::PublishPresetPersistConfig persist;
  persist.name = preset.name.toStdString();
  persist.channel = preset.channel.toStdString();
  persist.message_type = preset.message_type.toStdString();
  persist.message_json = preset.message_json.toStdString();
  persist.loop_publish = preset.loop_publish;
  persist.publish_rate_hz = preset.publish_rate_hz;
  persist.button_label = preset.button_label.toStdString();
  persist.button_tooltip = preset.button_tooltip.toStdString();
  if (preset.button_color.isValid()) {
    persist.button_color = preset.button_color.name().toStdString();
  }
  return persist;
}

PublishPreset FromPersistPreset(const common::PublishPresetPersistConfig& persist) {
  PublishPreset preset;
  preset.name = QString::fromStdString(persist.name);
  preset.channel = QString::fromStdString(persist.channel);
  preset.message_type = QString::fromStdString(persist.message_type);
  preset.message_json = QString::fromStdString(persist.message_json);
  preset.loop_publish = persist.loop_publish;
  preset.publish_rate_hz = persist.publish_rate_hz;
  preset.button_label = QString::fromStdString(persist.button_label);
  preset.button_tooltip = QString::fromStdString(persist.button_tooltip);
  if (!persist.button_color.empty()) {
    preset.button_color = QColor(QString::fromStdString(persist.button_color));
  }
  return preset;
}

common::PublishEntryPersistConfig ToPersistEntry(const PublishEntry& entry) {
  common::PublishEntryPersistConfig persist;
  persist.id = entry.id.toStdString();
  persist.channel = entry.channel.toStdString();
  persist.message_type = entry.message_type.toStdString();
  persist.publish_rate_hz = entry.publish_rate_hz;
  persist.message_json = entry.message_json.toStdString();
  persist.publishing = entry.publishing;
  return persist;
}

PublishEntry FromPersistEntry(const common::PublishEntryPersistConfig& persist) {
  PublishEntry entry;
  entry.id = persist.id.empty() ? NewPublishEntryId()
                                : QString::fromStdString(persist.id);
  entry.channel = QString::fromStdString(persist.channel);
  entry.message_type = QString::fromStdString(persist.message_type);
  entry.publish_rate_hz = persist.publish_rate_hz;
  entry.message_json = QString::fromStdString(persist.message_json);
  entry.publishing = persist.publishing;
  return entry;
}

}  // namespace

common::PublishPanelPersistConfig ToPersistConfig(
    const QString& object_name, const PublishPanelConfig& config) {
  common::PublishPanelPersistConfig persist;
  persist.object_name = object_name.toStdString();
  persist.title = config.title.toStdString();
  persist.channel = config.channel.toStdString();
  persist.message_type = config.message_type.toStdString();
  persist.message_json = config.message_json.toStdString();
  persist.editing_mode = config.editing_mode;
  persist.loop_publish = config.loop_publish;
  persist.publish_rate_hz = config.publish_rate_hz;
  persist.button_label = config.button_label.toStdString();
  persist.button_tooltip = config.button_tooltip.toStdString();
  if (config.button_color.isValid()) {
    persist.button_color = config.button_color.name().toStdString();
  }
  persist.active_preset_name = config.active_preset_name.toStdString();
  persist.saved_presets.reserve(
      static_cast<std::size_t>(config.saved_presets.size()));
  for (const PublishPreset& preset : config.saved_presets) {
    persist.saved_presets.push_back(ToPersistPreset(preset));
  }
  persist.custom_channels.reserve(
      static_cast<std::size_t>(config.custom_channels.size()));
  for (const QString& channel : config.custom_channels) {
    persist.custom_channels.push_back(channel.toStdString());
  }
  persist.publishers.reserve(static_cast<std::size_t>(config.publishers.size()));
  for (const PublishEntry& entry : config.publishers) {
    persist.publishers.push_back(ToPersistEntry(entry));
  }
  persist.selected_publisher_index = config.selected_publisher_index;
  return persist;
}

PublishPanelConfig FromPersistConfig(
    const common::PublishPanelPersistConfig& persist) {
  PublishPanelConfig config;
  config.title = QString::fromStdString(persist.title);
  config.channel = QString::fromStdString(persist.channel);
  config.message_type = QString::fromStdString(persist.message_type);
  config.message_json = QString::fromStdString(persist.message_json);
  config.editing_mode = persist.editing_mode;
  config.loop_publish = persist.loop_publish;
  config.publish_rate_hz = persist.publish_rate_hz;
  config.button_label = QString::fromStdString(persist.button_label);
  config.button_tooltip = QString::fromStdString(persist.button_tooltip);
  if (!persist.button_color.empty()) {
    config.button_color = QColor(QString::fromStdString(persist.button_color));
  }
  config.active_preset_name = QString::fromStdString(persist.active_preset_name);
  config.saved_presets.reserve(static_cast<int>(persist.saved_presets.size()));
  for (const common::PublishPresetPersistConfig& preset : persist.saved_presets) {
    config.saved_presets.push_back(FromPersistPreset(preset));
  }
  for (const std::string& channel : persist.custom_channels) {
    config.custom_channels.push_back(QString::fromStdString(channel));
  }
  config.publishers.reserve(static_cast<int>(persist.publishers.size()));
  for (const common::PublishEntryPersistConfig& entry : persist.publishers) {
    config.publishers.push_back(FromPersistEntry(entry));
  }
  config.selected_publisher_index = persist.selected_publisher_index;
  return config;
}

}  // namespace publish_panel
}  // namespace autoviz
