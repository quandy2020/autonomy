/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/service/service_config_io.hpp"

#include <QColor>

namespace autoviz {
namespace service_panel {

common::ServicePanelPersistConfig ToPersistConfig(
    const QString& object_name, const ServiceCallPanelConfig& config,
    bool settings_visible) {
  common::ServicePanelPersistConfig persist;
  persist.object_name = object_name.toStdString();
  persist.title = config.title.toStdString();
  persist.service_name = config.service_name.toStdString();
  persist.request_type = config.request_type.toStdString();
  persist.response_type = config.response_type.toStdString();
  persist.request_json = config.request_json.toStdString();
  persist.editing_mode = config.editing_mode;
  persist.vertical_layout = config.vertical_layout;
  persist.timeout_sec = config.timeout_sec;
  persist.button_label = config.button_label.toStdString();
  persist.button_tooltip = config.button_tooltip.toStdString();
  if (config.button_color.isValid()) {
    persist.button_color = config.button_color.name().toStdString();
  }
  persist.settings_visible = settings_visible;
  return persist;
}

ServiceCallPanelConfig FromPersistConfig(
    const common::ServicePanelPersistConfig& persist) {
  ServiceCallPanelConfig config;
  config.title = QString::fromStdString(persist.title);
  config.service_name = QString::fromStdString(persist.service_name);
  config.request_type = QString::fromStdString(persist.request_type);
  config.response_type = QString::fromStdString(persist.response_type);
  config.request_json = QString::fromStdString(persist.request_json);
  config.editing_mode = persist.editing_mode;
  config.vertical_layout = persist.vertical_layout;
  config.timeout_sec = persist.timeout_sec > 0 ? persist.timeout_sec : 5;
  config.button_label = QString::fromStdString(persist.button_label);
  config.button_tooltip = QString::fromStdString(persist.button_tooltip);
  if (!persist.button_color.empty()) {
    config.button_color = QColor(QString::fromStdString(persist.button_color));
  }
  return config;
}

}  // namespace service_panel
}  // namespace autoviz
