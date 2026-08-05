/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <QVector>
#include <QSet>

#include "autoviz/ui/log/log_types.hpp"

class QFrame;
class QLabel;
class QListWidget;
class QListWidgetItem;
class QPushButton;

namespace autoviz {
namespace log_panel {

class LogViewWidget : public QWidget {
  Q_OBJECT

 public:
  explicit LogViewWidget(QWidget* parent = nullptr);

  void setFontSize(double size);
  void setMinLevel(LogLevel level);
  void setSearchTerms(const QStringList& terms);
  void setEnabledNamespaces(const QSet<QString>& enabled);
  void addEntry(const LogEntry& entry);
  void clearEntries();
  void scrollToTimestamp(double timestamp_sec);
  void followTimestamp(double timestamp_sec);

  int totalEntryCount() const;
  int visibleEntryCount() const;
  bool pinnedToBottom() const;

 signals:
  void entryClicked(const LogEntry& entry);
  void entryHovered(const LogEntry& entry);
  void scrollPinnedChanged(bool pinned_to_bottom);
  void statsChanged(int visible_count, int total_count, bool pinned);

 private slots:
  void onItemClicked(QListWidgetItem* item);
  void onItemEntered(QListWidgetItem* item);
  void onScrollToBottomClicked();
  void onScrollValueChanged(int value);

 private:
  struct StoredEntry {
    LogEntry entry;
  };

  bool passesFilters(const LogEntry& entry) const;
  bool passesSearch(const LogEntry& entry) const;
  bool namespaceEnabled(const LogEntry& entry) const;
  QString formatTimestamp(const LogEntry& entry) const;
  void populateListItem(QListWidgetItem* item, int storage_index) const;
  void appendListItem(int storage_index);
  void refreshList();
  void updateEmptyState();
  void updateScrollFooterVisibility();
  void emitStats();
  bool isPinnedToBottomInternal() const;

  QFrame* list_container_ = nullptr;
  QListWidget* list_ = nullptr;
  QLabel* empty_label_ = nullptr;
  QFrame* scroll_footer_ = nullptr;
  QPushButton* scroll_bottom_button_ = nullptr;
  QVector<StoredEntry> entries_;
  LogLevel min_level_ = LogLevel::kDebug;
  QStringList search_terms_;
  QSet<QString> enabled_namespaces_;
  double font_size_ = 12.0;
  bool pinned_to_bottom_ = true;
};

}  // namespace log_panel
}  // namespace autoviz
