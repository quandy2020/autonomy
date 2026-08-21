/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_types.hpp"

#include <QUuid>

namespace autoviz {
namespace publish_panel {

QString NewPublishEntryId() {
  return QUuid::createUuid().toString(QUuid::WithoutBraces);
}

QString ExpressionPreview(const QString& json) {
  QString preview = json;
  preview.replace(QLatin1Char('\n'), QLatin1Char(' '));
  preview.replace(QLatin1Char('\r'), QLatin1Char(' '));
  preview = preview.simplified();
  if (preview.size() > 96) {
    return preview.left(93) + QStringLiteral("...");
  }
  return preview;
}

PublishPanelConfig DefaultPublishPanelConfig() {
  PublishPanelConfig config;
  config.channel = QStringLiteral("/cmd_vel");
  config.message_type =
      QStringLiteral("automsgs.msgs.geometry_msgs.TwistStamped");
  return config;
}

}  // namespace publish_panel
}  // namespace autoviz
