/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>

#include <QObject>

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

 signals:
  void logAppended(const LogEntry& entry);

 private:
  LogHub();
  ~LogHub() override;

  std::unique_ptr<google::LogSink> glog_sink_;
};

}  // namespace log_panel
}  // namespace autoviz
