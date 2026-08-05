/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/log/log_hub.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include <QMetaObject>
#include <QThread>

#include <glog/logging.h>

namespace autoviz {
namespace log_panel {
namespace {

LogLevel LevelFromGlog(google::LogSeverity severity) {
  switch (severity) {
    case google::GLOG_INFO:
      return LogLevel::kInfo;
    case google::GLOG_WARNING:
      return LogLevel::kWarn;
    case google::GLOG_ERROR:
      return LogLevel::kError;
    case google::GLOG_FATAL:
      return LogLevel::kFatal;
    default:
      return LogLevel::kDebug;
  }
}

qint64 NowTimestampNs() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

class AutovizGlogSink : public google::LogSink {
 public:
  void send(google::LogSeverity severity, const char* /*full_filename*/,
            const char* base_filename, int line, const struct tm* /*tm_time*/,
            const char* message, size_t message_len) override {
    LogEntry entry;
    entry.timestamp_ns = NowTimestampNs();
    entry.level = LevelFromGlog(severity);
    entry.message = QString::fromUtf8(message, static_cast<int>(message_len)).trimmed();
    entry.name = QStringLiteral("autoviz");
    if (base_filename != nullptr) {
      entry.file = QString::fromUtf8(base_filename);
    }
    entry.line = static_cast<quint32>(std::max(line, 0));
    entry.source = QStringLiteral("glog");
    LogHub::instance().append(std::move(entry));
  }
};

}  // namespace

LogHub& LogHub::instance() {
  static LogHub hub;
  return hub;
}

LogHub::LogHub() = default;

LogHub::~LogHub() { uninstallGlogCapture(); }

void LogHub::installGlogCapture() {
  if (glog_sink_ != nullptr) {
    return;
  }
  glog_sink_ = std::make_unique<AutovizGlogSink>();
  google::AddLogSink(glog_sink_.get());
}

void LogHub::uninstallGlogCapture() {
  if (glog_sink_ == nullptr) {
    return;
  }
  google::RemoveLogSink(glog_sink_.get());
  glog_sink_.reset();
}

void LogHub::append(LogEntry entry) {
  if (entry.message.isEmpty()) {
    return;
  }
  if (QThread::currentThread() == thread()) {
    emit logAppended(entry);
    return;
  }
  QMetaObject::invokeMethod(
      this, [this, entry]() { emit logAppended(entry); }, Qt::QueuedConnection);
}

}  // namespace log_panel
}  // namespace autoviz
