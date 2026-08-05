/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/log/log_types.hpp"

namespace autoviz {
namespace log_panel {

LogPanelConfig DefaultLogPanelConfig() { return LogPanelConfig{}; }

QString logLevelLabel(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return QStringLiteral("DEBUG");
    case LogLevel::kInfo:
      return QStringLiteral("INFO");
    case LogLevel::kWarn:
      return QStringLiteral("WARN");
    case LogLevel::kError:
      return QStringLiteral("ERROR");
    case LogLevel::kFatal:
      return QStringLiteral("FATAL");
    default:
      return QStringLiteral("UNKNOWN");
  }
}

LogLevel logLevelFromFoxgloveValue(int value) {
  switch (value) {
    case 1:
      return LogLevel::kDebug;
    case 2:
      return LogLevel::kInfo;
    case 3:
      return LogLevel::kWarn;
    case 4:
      return LogLevel::kError;
    case 5:
      return LogLevel::kFatal;
    default:
      return LogLevel::kUnknown;
  }
}

LogLevel logLevelFromString(const QString& text) {
  const QString upper = text.trimmed().toUpper();
  if (upper == QLatin1String("DEBUG")) {
    return LogLevel::kDebug;
  }
  if (upper == QLatin1String("INFO")) {
    return LogLevel::kInfo;
  }
  if (upper == QLatin1String("WARN") || upper == QLatin1String("WARNING")) {
    return LogLevel::kWarn;
  }
  if (upper == QLatin1String("ERROR")) {
    return LogLevel::kError;
  }
  if (upper == QLatin1String("FATAL")) {
    return LogLevel::kFatal;
  }
  return LogLevel::kUnknown;
}

int logLevelRank(LogLevel level) { return static_cast<int>(level); }

}  // namespace log_panel
}  // namespace autoviz
