/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>
#include <QStringList>
#include <QVector>

namespace autoviz {
namespace publish_panel {

/** One active publisher row (rqt Message Publisher table entry). */
struct PublishEntry {
  QString id;
  QString channel;
  QString message_type;
  double publish_rate_hz = 1.0;
  QString message_json;
  bool publishing = true;
};

struct PublishPreset {
  QString name;
  QString channel;
  QString message_type;
  QString message_json;
  bool loop_publish = false;
  double publish_rate_hz = 1.0;
  QString button_label;
  QString button_tooltip;
  QColor button_color;
};

struct PublishPanelConfig {
  QString title;
  QString channel;
  QString message_type;
  QString message_json;
  bool editing_mode = false;
  bool loop_publish = false;
  double publish_rate_hz = 1.0;
  QString button_label = QStringLiteral("Send");
  QString button_tooltip;
  QColor button_color;
  QVector<PublishPreset> saved_presets;
  QString active_preset_name;
  QStringList custom_channels;
  QVector<PublishEntry> publishers;
  int selected_publisher_index = -1;
};

QString NewPublishEntryId();
QString ExpressionPreview(const QString& json);

PublishPanelConfig DefaultPublishPanelConfig();

}  // namespace publish_panel
}  // namespace autoviz
