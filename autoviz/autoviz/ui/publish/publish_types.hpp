/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>

namespace autoviz {
namespace publish_panel {

struct PublishPanelConfig {
  QString title;
  QString channel;
  QString message_type;
  QString message_json;
  bool editing_mode = true;
  QString button_label = QStringLiteral("Publish");
  QString button_tooltip;
  QColor button_color;
};

PublishPanelConfig DefaultPublishPanelConfig();

}  // namespace publish_panel
}  // namespace autoviz
