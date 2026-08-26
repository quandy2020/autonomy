/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/log/log_view_widget.hpp"

#include <algorithm>
#include <limits>

#include <QDateTime>
#include <QFrame>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QScrollBar>
#include <QVBoxLayout>

#include "autoviz/ui/log/log_entry_delegate.hpp"
#include "autoviz/ui/log/log_types.hpp"

namespace autoviz {
namespace log_panel {
namespace {

constexpr int kRoleEntryIndex = Qt::UserRole;
constexpr int kRoleTimestamp = Qt::UserRole + 1;
constexpr int kRoleLevel = Qt::UserRole + 2;
constexpr int kRoleName = Qt::UserRole + 3;
constexpr int kRoleMessage = Qt::UserRole + 4;

QString ListStyleSheet() {
  return QStringLiteral(
      "QListWidget {"
      "  background: #ffffff;"
      "  border: none;"
      "  outline: none;"
      "  padding: 0px;"
      "}"
      "QListWidget::item {"
      "  border: none;"
      "  padding: 0px;"
      "  margin: 0px;"
      "}"
      "QScrollBar:vertical {"
      "  background: #f8f9fb;"
      "  width: 10px;"
      "  margin: 0px;"
      "}"
      "QScrollBar::handle:vertical {"
      "  background: #cbd5e1;"
      "  min-height: 24px;"
      "  border-radius: 5px;"
      "  margin: 2px;"
      "}"
      "QScrollBar::handle:vertical:hover {"
      "  background: #0891b2;"
      "}"
      "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {"
      "  height: 0px;"
      "}"
      "QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {"
      "  background: none;"
      "}");
}

}  // namespace

LogViewWidget::LogViewWidget(QWidget* parent) : QWidget(parent) {
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(QStringLiteral("LogViewWidget { background: #ffffff; }"));

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  list_container_ = new QFrame(this);
  list_container_->setFrameShape(QFrame::NoFrame);
  list_container_->setStyleSheet(QStringLiteral("QFrame { background: #ffffff; }"));
  list_container_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  auto* container_layout = new QVBoxLayout(list_container_);
  container_layout->setContentsMargins(0, 0, 0, 0);
  container_layout->setSpacing(0);

  list_ = new QListWidget(list_container_);
  list_->setFrameShape(QFrame::NoFrame);
  list_->setUniformItemSizes(true);
  list_->setSelectionMode(QAbstractItemView::SingleSelection);
  list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  list_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  list_->setMouseTracking(true);
  list_->setSpacing(0);
  list_->setStyleSheet(ListStyleSheet());
  list_->setItemDelegate(new LogEntryDelegate(list_));
  container_layout->addWidget(list_, 1);

  empty_label_ = new QLabel(tr("No log entries yet"), list_container_);
  empty_label_->setAlignment(Qt::AlignCenter);
  empty_label_->setStyleSheet(
      QStringLiteral("color: #64748b; font-size: 12px; background: transparent;"));
  empty_label_->hide();
  container_layout->addWidget(empty_label_, 1, Qt::AlignCenter);

  scroll_footer_ = new QFrame(list_container_);
  scroll_footer_->setVisible(false);
  scroll_footer_->setStyleSheet(
      QStringLiteral(
          "QFrame {"
          "  background: #f8f9fb;"
          "  border-top: 1px solid #cbd5e1;"
          "}"));
  auto* footer_layout = new QHBoxLayout(scroll_footer_);
  footer_layout->setContentsMargins(6, 4, 6, 4);
  scroll_bottom_button_ = new QPushButton(tr("↓ Latest"), scroll_footer_);
  scroll_bottom_button_->setCursor(Qt::PointingHandCursor);
  scroll_bottom_button_->setFlat(true);
  scroll_bottom_button_->setStyleSheet(
      QStringLiteral(
          "QPushButton {"
          "  color: #0891b2;"
          "  background: rgba(8,145,178,0.10);"
          "  border: 1px solid rgba(8,145,178,0.28);"
          "  border-radius: 8px;"
          "  padding: 4px 12px;"
          "  font-size: 11px;"
          "  font-weight: 600;"
          "}"
          "QPushButton:hover {"
          "  background: rgba(8,145,178,0.18);"
          "}"));
  footer_layout->addStretch();
  footer_layout->addWidget(scroll_bottom_button_);
  footer_layout->addStretch();
  container_layout->addWidget(scroll_footer_);

  root->addWidget(list_container_, 1);

  connect(list_, &QListWidget::itemClicked, this, &LogViewWidget::onItemClicked);
  connect(list_, &QListWidget::itemEntered, this, &LogViewWidget::onItemEntered);
  connect(scroll_bottom_button_, &QPushButton::clicked, this,
          &LogViewWidget::onScrollToBottomClicked);
  connect(list_->verticalScrollBar(), &QScrollBar::valueChanged, this,
          &LogViewWidget::onScrollValueChanged);

  setFontSize(font_size_);
  updateEmptyState();
}

void LogViewWidget::setFontSize(double size) {
  font_size_ = std::clamp(size, 8.0, 24.0);
  QFont font = list_->font();
  font.setPointSizeF(font_size_);
  list_->setFont(font);
  if (list_->count() > 0) {
    list_->doItemsLayout();
  }
}

void LogViewWidget::setMinLevel(LogLevel level) {
  if (min_level_ == level) {
    return;
  }
  min_level_ = level;
  refreshList();
}

void LogViewWidget::setSearchTerms(const QStringList& terms) {
  search_terms_ = terms;
  refreshList();
}

void LogViewWidget::setEnabledNamespaces(const QSet<QString>& enabled) {
  enabled_namespaces_ = enabled;
  refreshList();
}

bool LogViewWidget::namespaceEnabled(const LogEntry& entry) const {
  if (entry.name.isEmpty() || enabled_namespaces_.isEmpty()) {
    return true;
  }
  return enabled_namespaces_.contains(entry.name);
}

bool LogViewWidget::passesSearch(const LogEntry& entry) const {
  if (search_terms_.isEmpty()) {
    return true;
  }
  const QString haystack =
      entry.message + QLatin1Char(' ') + entry.name + QLatin1Char(' ') + entry.file;
  for (const QString& term : search_terms_) {
    if (!term.isEmpty() && haystack.contains(term, Qt::CaseInsensitive)) {
      return true;
    }
  }
  return false;
}

bool LogViewWidget::passesFilters(const LogEntry& entry) const {
  if (logLevelRank(entry.level) < logLevelRank(min_level_)) {
    return false;
  }
  if (!namespaceEnabled(entry)) {
    return false;
  }
  return passesSearch(entry);
}

QString LogViewWidget::formatTimestamp(const LogEntry& entry) const {
  if (entry.timestamp_ns > 0) {
    const QDateTime date_time =
        QDateTime::fromMSecsSinceEpoch(entry.timestamp_ns / 1000000LL, Qt::UTC);
    return date_time.toString(QStringLiteral("HH:mm:ss.zzz"));
  }
  return QStringLiteral("--:--:--.---");
}

void LogViewWidget::populateListItem(QListWidgetItem* item, int storage_index) const {
  if (storage_index < 0 || storage_index >= entries_.size()) {
    return;
  }
  const StoredEntry& stored = entries_.at(storage_index);
  item->setData(kRoleEntryIndex, storage_index);
  item->setData(kRoleTimestamp, formatTimestamp(stored.entry));
  item->setData(kRoleLevel, static_cast<int>(stored.entry.level));
  item->setData(kRoleName, stored.entry.name);
  item->setData(kRoleMessage, stored.entry.message);
  const QString timestamp = formatTimestamp(stored.entry);
  const QString body = stored.entry.name.isEmpty()
                           ? stored.entry.message
                           : QStringLiteral("[%1] %2")
                                 .arg(stored.entry.name, stored.entry.message);
  item->setToolTip(QStringLiteral("%1  %2\n%3")
                       .arg(timestamp, logLevelLabel(stored.entry.level), body));
}

void LogViewWidget::appendListItem(int storage_index) {
  auto* item = new QListWidgetItem(list_);
  populateListItem(item, storage_index);
}

void LogViewWidget::addEntry(const LogEntry& entry) {
  StoredEntry stored;
  stored.entry = entry;
  entries_.push_back(stored);
  const int storage_index = entries_.size() - 1;
  if (passesFilters(entry)) {
    appendListItem(storage_index);
    if (pinned_to_bottom_) {
      list_->scrollToBottom();
    }
  }
  updateEmptyState();
  emitStats();
}

void LogViewWidget::clearEntries() {
  entries_.clear();
  list_->clear();
  updateEmptyState();
  emitStats();
}

void LogViewWidget::refreshList() {
  list_->clear();
  for (int i = 0; i < entries_.size(); ++i) {
    if (passesFilters(entries_.at(i).entry)) {
      appendListItem(i);
    }
  }
  if (pinned_to_bottom_) {
    list_->scrollToBottom();
  }
  updateEmptyState();
  emitStats();
}

void LogViewWidget::scrollToTimestamp(double timestamp_sec) {
  if (entries_.isEmpty()) {
    return;
  }
  const qint64 target_ns = static_cast<qint64>(timestamp_sec * 1e9);
  int best_row = -1;
  qint64 best_delta = std::numeric_limits<qint64>::max();
  for (int i = 0; i < entries_.size(); ++i) {
    const qint64 ts = entries_.at(i).entry.timestamp_ns;
    if (ts <= 0) {
      continue;
    }
    const qint64 delta = std::llabs(ts - target_ns);
    if (delta < best_delta) {
      best_delta = delta;
      best_row = i;
    }
  }
  if (best_row < 0) {
    return;
  }
  for (int row = 0; row < list_->count(); ++row) {
    if (list_->item(row)->data(kRoleEntryIndex).toInt() == best_row) {
      list_->setCurrentRow(row);
      list_->scrollToItem(list_->item(row), QAbstractItemView::PositionAtCenter);
      pinned_to_bottom_ = false;
      updateScrollFooterVisibility();
      emitStats();
      break;
    }
  }
}

void LogViewWidget::followTimestamp(double timestamp_sec) {
  if (!pinned_to_bottom_) {
    return;
  }
  scrollToTimestamp(timestamp_sec);
}

int LogViewWidget::totalEntryCount() const { return entries_.size(); }

int LogViewWidget::visibleEntryCount() const { return list_->count(); }

bool LogViewWidget::pinnedToBottom() const { return pinned_to_bottom_; }

void LogViewWidget::onItemClicked(QListWidgetItem* item) {
  if (item == nullptr) {
    return;
  }
  const int index = item->data(kRoleEntryIndex).toInt();
  if (index < 0 || index >= entries_.size()) {
    return;
  }
  emit entryClicked(entries_.at(index).entry);
}

void LogViewWidget::onItemEntered(QListWidgetItem* item) {
  if (item == nullptr) {
    return;
  }
  const int index = item->data(kRoleEntryIndex).toInt();
  if (index < 0 || index >= entries_.size()) {
    return;
  }
  emit entryHovered(entries_.at(index).entry);
}

void LogViewWidget::onScrollToBottomClicked() {
  pinned_to_bottom_ = true;
  list_->scrollToBottom();
  updateScrollFooterVisibility();
  emit scrollPinnedChanged(true);
  emitStats();
}

bool LogViewWidget::isPinnedToBottomInternal() const {
  QScrollBar* bar = list_->verticalScrollBar();
  if (bar == nullptr) {
    return true;
  }
  return bar->value() >= bar->maximum() - 2;
}

void LogViewWidget::onScrollValueChanged(int /*value*/) {
  const bool pinned = isPinnedToBottomInternal();
  if (pinned_to_bottom_ != pinned) {
    pinned_to_bottom_ = pinned;
    emit scrollPinnedChanged(pinned_to_bottom_);
    emitStats();
  }
  updateScrollFooterVisibility();
}

void LogViewWidget::updateScrollFooterVisibility() {
  const bool show_footer = !pinned_to_bottom_ && list_->count() > 0;
  scroll_footer_->setVisible(show_footer);
}

void LogViewWidget::updateEmptyState() {
  const bool has_entries = !entries_.isEmpty();
  const bool visible = list_->count() > 0;
  if (!has_entries) {
    empty_label_->setText(tr("No log entries yet"));
  } else if (!visible) {
    empty_label_->setText(tr("No entries match the current filters"));
  }
  empty_label_->setVisible(!visible);
  list_->setVisible(visible);
  updateScrollFooterVisibility();
}

void LogViewWidget::emitStats() {
  emit statsChanged(visibleEntryCount(), totalEntryCount(), pinned_to_bottom_);
}

}  // namespace log_panel
}  // namespace autoviz
