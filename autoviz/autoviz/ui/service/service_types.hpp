/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>

namespace autoviz {
namespace service_panel {

struct ServiceCallPanelConfig {
  QString title;
  QString service_name;
  QString request_type;
  QString response_type;
  QString request_json = QStringLiteral("{}");
  QString response_json;
  bool editing_mode = true;
  bool vertical_layout = true;
  int timeout_sec = 5;
  QString button_label = QStringLiteral("Call service");
  QString button_tooltip;
  QColor button_color;
};

ServiceCallPanelConfig DefaultServiceCallPanelConfig();

}  // namespace service_panel
}  // namespace autoviz
