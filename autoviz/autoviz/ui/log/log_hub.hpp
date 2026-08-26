/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <deque>
#include <memory>
#include <mutex>

#include <QObject>
#include <QVector>

#include "autoviz/ui/log/log_types.hpp"

namespace google {
class LogSink;
}

namespace autoviz {
namespace log_panel {

/** Central hub for glog capture and foxglove.Log distribution to Log panels. */
class LogHub : public QObject {
  Q_OBJECT

 public:
  static LogHub& instance();

  void installGlogCapture();
  void uninstallGlogCapture();
  void append(LogEntry entry);

  /** Snapshot of recent entries for late-joining panels. */
  QVector<LogEntry> recentEntries() const;

 signals:
  void logAppended(const LogEntry& entry);

 private:
  LogHub();
  ~LogHub() override;

  bool isDuplicateLocked(const LogEntry& entry) const;
  void storeLocked(const LogEntry& entry);

  std::unique_ptr<google::LogSink> glog_sink_;
  mutable std::mutex mutex_;
  std::deque<LogEntry> recent_;
  static constexpr int kMaxRecent = 2000;
};

}  // namespace log_panel
}  // namespace autoviz
