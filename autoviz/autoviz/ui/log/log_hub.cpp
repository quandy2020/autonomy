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

/** Pull leading [module] from glog/AINFO stream text into entry.name. */
void ExtractModuleName(LogEntry* entry) {
  if (entry == nullptr || entry->message.isEmpty()) {
    return;
  }
  if (!entry->message.startsWith(QLatin1Char('['))) {
    return;
  }
  const int close = entry->message.indexOf(QLatin1Char(']'));
  if (close <= 1) {
    return;
  }
  const QString module = entry->message.mid(1, close - 1).trimmed();
  if (module.isEmpty()) {
    return;
  }
  entry->name = module;
  entry->message = entry->message.mid(close + 1).trimmed();
}

class AutovizGlogSink : public google::LogSink {
 public:
  void send(google::LogSeverity severity, const char* /*full_filename*/,
            const char* base_filename, int line, const struct tm* /*tm_time*/,
            const char* message, size_t message_len) override {
    LogEntry entry;
    entry.timestamp_ns = NowTimestampNs();
    entry.level = LevelFromGlog(severity);
    entry.message =
        QString::fromUtf8(message, static_cast<int>(message_len)).trimmed();
    ExtractModuleName(&entry);
    if (entry.name.isEmpty()) {
      entry.name = QStringLiteral("autoviz");
    }
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

bool LogHub::isDuplicateLocked(const LogEntry& entry) const {
  if (recent_.empty()) {
    return false;
  }
  const LogEntry& last = recent_.back();
  if (last.message != entry.message || last.level != entry.level ||
      last.name != entry.name) {
    return false;
  }
  return std::llabs(last.timestamp_ns - entry.timestamp_ns) < 80000000LL;
}

void LogHub::storeLocked(const LogEntry& entry) {
  recent_.push_back(entry);
  while (static_cast<int>(recent_.size()) > kMaxRecent) {
    recent_.pop_front();
  }
}

QVector<LogEntry> LogHub::recentEntries() const {
  std::lock_guard<std::mutex> lock(mutex_);
  QVector<LogEntry> out;
  out.reserve(static_cast<int>(recent_.size()));
  for (const LogEntry& entry : recent_) {
    out.push_back(entry);
  }
  return out;
}

void LogHub::append(LogEntry entry) {
  if (entry.message.isEmpty()) {
    return;
  }
  ExtractModuleName(&entry);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (isDuplicateLocked(entry)) {
      return;
    }
    storeLocked(entry);
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
