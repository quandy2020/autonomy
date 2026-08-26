/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>
#include <QVector>

namespace autoviz {
namespace log_panel {

enum class LogLevel {
  kUnknown = 0,
  kDebug = 1,
  kInfo = 2,
  kWarn = 3,
  kError = 4,
  kFatal = 5,
};

struct LogEntry {
  qint64 timestamp_ns = 0;
  LogLevel level = LogLevel::kInfo;
  QString message;
  QString name;
  QString file;
  quint32 line = 0;
  QString source;
};

struct LogPanelConfig {
  QString title = QStringLiteral("Log");
  /** Preferred foxglove.Log channel; empty means auto-discover (/rosout + all Log types). */
  QString topic;
  LogLevel min_level = LogLevel::kDebug;
  double font_size = 11.0;
  bool capture_glog = true;
  bool follow_playback = true;
  bool settings_visible = false;
  /** When true (default), also subscribe every discovered foxglove.Log channel. */
  bool auto_subscribe_log_channels = true;
};

LogPanelConfig DefaultLogPanelConfig();

QString logLevelLabel(LogLevel level);
LogLevel logLevelFromFoxgloveValue(int value);
LogLevel logLevelFromString(const QString& text);
int logLevelRank(LogLevel level);

}  // namespace log_panel
}  // namespace autoviz
