/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractTableModel>
#include <QDateTime>
#include <QString>
#include <QVector>
#include <QWidget>
#include <deque>
#include <optional>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QSortFilterProxyModel;
class QTableView;

namespace autoviz::widget {

class Button;
class SearchBox;

enum class LogLevel {
  kTrace = 0,
  kDebug,
  kInformation,
  kWarning,
  kError,
  kCritical,
};

struct LogEntry {
  QDateTime timestamp;
  LogLevel level = LogLevel::kInformation;
  QString source;
  QString message;
};

// Bounded chronological log model. Removing the oldest rows is O(k) in the
// number removed and does not move the retained entries. Mutations invoked
// from another thread are queued onto the model's owning thread.
class LogModel final : public QAbstractTableModel {
 public:
  enum Column {
    kTimeColumn = 0,
    kLevelColumn,
    kSourceColumn,
    kMessageColumn,
    kColumnCount,
  };

  enum Role {
    kLevelRole = Qt::UserRole + 1,
    kTimestampRole,
    kSourceRole,
    kMessageRole,
  };

  explicit LogModel(QObject* parent = nullptr);

  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  int columnCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index,
                int role = Qt::DisplayRole) const override;
  QVariant headerData(int section, Qt::Orientation orientation,
                      int role = Qt::DisplayRole) const override;

  int maximum_entries() const noexcept { return maximum_entries_; }
  void SetMaximumEntries(int maximum_entries);
  std::optional<LogEntry> EntryAt(int row) const;
  void Append(LogEntry entry);
  void AppendBatch(QVector<LogEntry> entries);
  void Clear();

 private:
  std::deque<LogEntry> entries_;
  int maximum_entries_ = 5000;
};

// Virtualized log viewer with level/text filtering, bounded storage, paused
// auto-scroll, and row-copy support. It never creates per-cell widgets.
class LogView final : public QWidget {
  Q_OBJECT

 public:
  explicit LogView(QWidget* parent = nullptr);

  LogModel* log_model() const noexcept { return log_model_; }
  QSortFilterProxyModel* proxy_model() const noexcept { return proxy_model_; }
  QTableView* view() const noexcept { return view_; }

  void Append(LogEntry entry);
  void AppendBatch(QVector<LogEntry> entries);
  void Clear();
  void SetMaximumEntries(int maximum_entries);
  void SetMinimumLevel(LogLevel level);
  LogLevel minimum_level() const noexcept { return minimum_level_; }
  void SetFilterText(const QString& text);
  QString filter_text() const;
  void SetAutoScroll(bool enabled);
  bool auto_scroll() const noexcept { return auto_scroll_; }
  void SetTone(style::Tone tone);
  void CopySelectedRows() const;

 signals:
  void AutoScrollChanged(bool enabled);
  void EntryActivated(const LogEntry& entry);

 private:
  void ScrollToBottomIfEnabled();
  void UpdatePauseButton();

  LogModel* log_model_ = nullptr;
  QSortFilterProxyModel* proxy_model_ = nullptr;
  QTableView* view_ = nullptr;
  SearchBox* search_box_ = nullptr;
  Button* pause_button_ = nullptr;
  Button* clear_button_ = nullptr;
  Button* copy_button_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  LogLevel minimum_level_ = LogLevel::kTrace;
  bool auto_scroll_ = true;
  bool scroll_scheduled_ = false;
};

}  // namespace autoviz::widget

Q_DECLARE_METATYPE(autoviz::widget::LogEntry)
Q_DECLARE_METATYPE(autoviz::widget::LogLevel)
