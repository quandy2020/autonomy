/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/log/log_parser.hpp"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

namespace autoviz {
namespace log_panel {
namespace {

qint64 TimestampNsFromJson(const QJsonObject& object) {
  const QJsonObject timestamp = object.value(QStringLiteral("timestamp")).toObject();
  if (timestamp.isEmpty()) {
    return 0;
  }
  const qint64 sec = timestamp.value(QStringLiteral("sec")).toVariant().toLongLong();
  const qint64 nsec = timestamp.value(QStringLiteral("nsec")).toVariant().toLongLong();
  return sec * 1000000000LL + nsec;
}

LogLevel LevelFromJson(const QJsonValue& value) {
  if (value.isDouble()) {
    return logLevelFromFoxgloveValue(value.toInt());
  }
  if (value.isString()) {
    return logLevelFromString(value.toString());
  }
  return LogLevel::kUnknown;
}

LogEntry FromFoxgloveJsonObject(const QJsonObject& root, const QString& source) {
  LogEntry entry;
  entry.source = source;
  entry.timestamp_ns = TimestampNsFromJson(root);
  entry.level = LevelFromJson(root.value(QStringLiteral("level")));
  entry.message = root.value(QStringLiteral("message")).toString();
  entry.name = root.value(QStringLiteral("name")).toString();
  entry.file = root.value(QStringLiteral("file")).toString();
  entry.line = static_cast<quint32>(root.value(QStringLiteral("line")).toInt());
  return entry;
}

}  // namespace

bool isLogMessageType(const std::string& message_type) {
  return message_type == "foxglove.Log" ||
         message_type.find("foxglove.Log") != std::string::npos ||
         message_type.find(".Log") != std::string::npos;
}

LogEntry logEntryFromPayload(const std::string& message_type,
                             const std::string& payload,
                             const QString& source) {
  if (payload.empty()) {
    return {};
  }
  if (payload.front() != '{') {
    return {};
  }
  const QJsonDocument document =
      QJsonDocument::fromJson(QByteArray::fromStdString(payload));
  if (!document.isObject()) {
    return {};
  }
  LogEntry entry = FromFoxgloveJsonObject(document.object(), source);
  if (entry.message.isEmpty() && !isLogMessageType(message_type)) {
    return {};
  }
  return entry;
}

}  // namespace log_panel
}  // namespace autoviz
