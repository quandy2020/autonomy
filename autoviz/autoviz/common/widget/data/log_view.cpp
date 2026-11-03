/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/data/log_view.hpp"

#include <QApplication>
#include <QClipboard>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QMetaObject>
#include <QShortcut>
#include <QSortFilterProxyModel>
#include <QTableView>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QVariant>
#include <utility>

#include "autoviz/common/widget/foundation/widget_theme.hpp"
#include "autoviz/common/widget/input/button.hpp"
#include "autoviz/common/widget/input/search_box.hpp"

namespace autoviz::widget {
namespace {

QString LevelText(LogLevel level) {
  switch (level) {
    case LogLevel::kTrace:
      return LogModel::tr("Trace");
    case LogLevel::kDebug:
      return LogModel::tr("Debug");
    case LogLevel::kInformation:
      return LogModel::tr("Info");
    case LogLevel::kWarning:
      return LogModel::tr("Warning");
    case LogLevel::kError:
      return LogModel::tr("Error");
    case LogLevel::kCritical:
      return LogModel::tr("Critical");
  }
  return {};
}

class LogFilterProxyModel final : public QSortFilterProxyModel {
 public:
  explicit LogFilterProxyModel(QObject* parent = nullptr)
      : QSortFilterProxyModel(parent) {
    setDynamicSortFilter(true);
    setFilterCaseSensitivity(Qt::CaseInsensitive);
  }

  void SetMinimumLevel(LogLevel level) {
    if (minimum_level_ == level) {
      return;
    }
    minimum_level_ = level;
    RefreshFilter();
  }

  void SetText(const QString& text) {
    const QString normalized = text.trimmed();
    if (text_ == normalized) {
      return;
    }
    text_ = normalized;
    RefreshFilter();
  }

 protected:
  bool filterAcceptsRow(int source_row,
                        const QModelIndex& source_parent) const override {
    const QAbstractItemModel* model = sourceModel();
    if (model == nullptr) {
      return false;
    }
    const QModelIndex first = model->index(source_row, 0, source_parent);
    const LogLevel level =
        static_cast<LogLevel>(first.data(LogModel::kLevelRole).toInt());
    if (static_cast<int>(level) < static_cast<int>(minimum_level_)) {
      return false;
    }
    if (text_.isEmpty()) {
      return true;
    }
    return first.data(LogModel::kSourceRole)
               .toString()
               .contains(text_, Qt::CaseInsensitive) ||
           first.data(LogModel::kMessageRole)
               .toString()
               .contains(text_, Qt::CaseInsensitive);
  }

 private:
  void RefreshFilter() {
#if QT_VERSION >= QT_VERSION_CHECK(6, 10, 0)
    beginFilterChange();
    endFilterChange(QSortFilterProxyModel::Direction::Rows);
#else
    invalidateFilter();
#endif
  }

  LogLevel minimum_level_ = LogLevel::kTrace;
  QString text_;
};

}  // namespace

LogModel::LogModel(QObject* parent) : QAbstractTableModel(parent) {}

int LogModel::rowCount(const QModelIndex& parent) const {
  return parent.isValid() ? 0 : static_cast<int>(entries_.size());
}

int LogModel::columnCount(const QModelIndex& parent) const {
  return parent.isValid() ? 0 : kColumnCount;
}

QVariant LogModel::data(const QModelIndex& index, int role) const {
  if (!index.isValid() || index.row() < 0 ||
      index.row() >= static_cast<int>(entries_.size())) {
    return {};
  }
  const LogEntry& entry = entries_[static_cast<std::size_t>(index.row())];
  if (role == kLevelRole) {
    return static_cast<int>(entry.level);
  }
  if (role == kTimestampRole) {
    return entry.timestamp;
  }
  if (role == kSourceRole) {
    return entry.source;
  }
  if (role == kMessageRole || role == Qt::ToolTipRole) {
    return entry.message;
  }
  if (role != Qt::DisplayRole) {
    return {};
  }
  switch (index.column()) {
    case kTimeColumn:
      return entry.timestamp.toString(QStringLiteral("HH:mm:ss.zzz"));
    case kLevelColumn:
      return LevelText(entry.level);
    case kSourceColumn:
      return entry.source;
    case kMessageColumn:
      return entry.message;
    default:
      return {};
  }
}

QVariant LogModel::headerData(int section, Qt::Orientation orientation,
                              int role) const {
  if (orientation != Qt::Horizontal || role != Qt::DisplayRole) {
    return {};
  }
  switch (section) {
    case kTimeColumn:
      return tr("Time");
    case kLevelColumn:
      return tr("Level");
    case kSourceColumn:
      return tr("Source");
    case kMessageColumn:
      return tr("Message");
    default:
      return {};
  }
}

void LogModel::SetMaximumEntries(int maximum_entries) {
  if (QThread::currentThread() != thread()) {
    QMetaObject::invokeMethod(
        this, [this, maximum_entries]() { SetMaximumEntries(maximum_entries); },
        Qt::QueuedConnection);
    return;
  }
  const int bounded = qBound(1, maximum_entries, 1000000);
  if (maximum_entries_ == bounded) {
    return;
  }
  maximum_entries_ = bounded;
  const int overflow =
      qMax(0, static_cast<int>(entries_.size()) - maximum_entries_);
  if (overflow == 0) {
    return;
  }
  beginRemoveRows(QModelIndex(), 0, overflow - 1);
  for (int index = 0; index < overflow; ++index) {
    entries_.pop_front();
  }
  endRemoveRows();
}

std::optional<LogEntry> LogModel::EntryAt(int row) const {
  Q_ASSERT(QThread::currentThread() == thread());
  if (row < 0 || row >= static_cast<int>(entries_.size())) {
    return std::nullopt;
  }
  return entries_[static_cast<std::size_t>(row)];
}

void LogModel::Append(LogEntry entry) {
  QVector<LogEntry> batch;
  batch.push_back(std::move(entry));
  AppendBatch(std::move(batch));
}

void LogModel::AppendBatch(QVector<LogEntry> entries) {
  if (QThread::currentThread() != thread()) {
    QMetaObject::invokeMethod(
        this,
        [this, entries = std::move(entries)]() mutable {
          AppendBatch(std::move(entries));
        },
        Qt::QueuedConnection);
    return;
  }
  if (entries.isEmpty()) {
    return;
  }
  const qsizetype maximum_entries = static_cast<qsizetype>(maximum_entries_);
  if (entries.size() >= maximum_entries) {
    beginResetModel();
    entries_.clear();
    const qsizetype first = entries.size() - maximum_entries;
    for (qsizetype index = first; index < entries.size(); ++index) {
      entries_.push_back(std::move(entries[index]));
    }
    endResetModel();
    return;
  }

  const int batch_size = static_cast<int>(entries.size());
  const int overflow = qMax(
      0, static_cast<int>(entries_.size()) + batch_size - maximum_entries_);
  if (overflow > 0) {
    beginRemoveRows(QModelIndex(), 0, overflow - 1);
    for (int index = 0; index < overflow; ++index) {
      entries_.pop_front();
    }
    endRemoveRows();
  }

  const int first_row = static_cast<int>(entries_.size());
  beginInsertRows(QModelIndex(), first_row, first_row + batch_size - 1);
  for (LogEntry& entry : entries) {
    entries_.push_back(std::move(entry));
  }
  endInsertRows();
}

void LogModel::Clear() {
  if (QThread::currentThread() != thread()) {
    QMetaObject::invokeMethod(
        this, [this]() { Clear(); }, Qt::QueuedConnection);
    return;
  }
  if (entries_.empty()) {
    return;
  }
  beginResetModel();
  entries_.clear();
  endResetModel();
}

LogView::LogView(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizLogView"));
  setProperty("autovizRole", QStringLiteral("logView"));
  setAccessibleName(tr("Runtime log"));

  log_model_ = new LogModel(this);
  auto* filter = new LogFilterProxyModel(this);
  proxy_model_ = filter;
  filter->setSourceModel(log_model_);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(GetThemeTokens(tone_).spacing_sm);
  auto* toolbar = new QHBoxLayout;
  toolbar->setContentsMargins(0, 0, 0, 0);
  toolbar->setSpacing(GetThemeTokens(tone_).spacing_sm);

  search_box_ = new SearchBox(this);
  search_box_->SetSearchBoxSize(SearchBox::Size::kSmall);
  search_box_->setAccessibleName(tr("Filter log"));
  search_box_->setPlaceholderText(tr("Filter source or message"));
  toolbar->addWidget(search_box_, 1);

  pause_button_ = new Button(this);
  pause_button_->SetButtonSize(Button::Size::kSmall);
  pause_button_->setCheckable(true);
  connect(pause_button_, &Button::toggled, this,
          [this](bool paused) { SetAutoScroll(!paused); });
  toolbar->addWidget(pause_button_);

  copy_button_ = new Button(tr("Copy"), this);
  copy_button_->SetButtonSize(Button::Size::kSmall);
  connect(copy_button_, &Button::clicked, this, &LogView::CopySelectedRows);
  toolbar->addWidget(copy_button_);

  clear_button_ = new Button(tr("Clear"), this);
  clear_button_->SetButtonSize(Button::Size::kSmall);
  connect(clear_button_, &Button::clicked, this, &LogView::Clear);
  toolbar->addWidget(clear_button_);
  root->addLayout(toolbar);

  view_ = new QTableView(this);
  view_->setModel(proxy_model_);
  view_->setAlternatingRowColors(true);
  view_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  view_->setSelectionBehavior(QAbstractItemView::SelectRows);
  view_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  view_->setSortingEnabled(false);
  view_->setWordWrap(false);
  view_->setTextElideMode(Qt::ElideRight);
  view_->verticalHeader()->setVisible(false);
  view_->verticalHeader()->setDefaultSectionSize(
      GetThemeTokens(tone_).control_height_small);
  QHeaderView* header = view_->horizontalHeader();
  header->setStretchLastSection(true);
  header->setSectionResizeMode(LogModel::kTimeColumn,
                               QHeaderView::ResizeToContents);
  header->setSectionResizeMode(LogModel::kLevelColumn,
                               QHeaderView::ResizeToContents);
  header->setSectionResizeMode(LogModel::kSourceColumn,
                               QHeaderView::Interactive);
  header->resizeSection(LogModel::kSourceColumn, 140);
  root->addWidget(view_, 1);

  connect(search_box_, &QLineEdit::textChanged, this,
          [this](const QString& text) { SetFilterText(text); });
  connect(proxy_model_, &QAbstractItemModel::rowsInserted, this,
          [this]() { ScrollToBottomIfEnabled(); });
  connect(proxy_model_, &QAbstractItemModel::modelReset, this,
          [this]() { ScrollToBottomIfEnabled(); });
  connect(view_, &QTableView::doubleClicked, this,
          [this](const QModelIndex& proxy_index) {
            const QModelIndex source_index =
                proxy_model_->mapToSource(proxy_index);
            const std::optional<LogEntry> entry =
                log_model_->EntryAt(source_index.row());
            if (entry.has_value()) {
              emit EntryActivated(*entry);
            }
          });

  auto* copy_shortcut = new QShortcut(QKeySequence::Copy, view_);
  connect(copy_shortcut, &QShortcut::activated, this,
          &LogView::CopySelectedRows);
  ApplyWidgetTheme(this, tone_);
  UpdatePauseButton();
}

void LogView::Append(LogEntry entry) {
  if (QThread::currentThread() != thread()) {
    QMetaObject::invokeMethod(
        this,
        [this, entry = std::move(entry)]() mutable {
          Append(std::move(entry));
        },
        Qt::QueuedConnection);
    return;
  }
  log_model_->Append(std::move(entry));
}

void LogView::AppendBatch(QVector<LogEntry> entries) {
  if (QThread::currentThread() != thread()) {
    QMetaObject::invokeMethod(
        this,
        [this, entries = std::move(entries)]() mutable {
          AppendBatch(std::move(entries));
        },
        Qt::QueuedConnection);
    return;
  }
  log_model_->AppendBatch(std::move(entries));
}

void LogView::Clear() {
  if (QThread::currentThread() != thread()) {
    QMetaObject::invokeMethod(
        this, [this]() { Clear(); }, Qt::QueuedConnection);
    return;
  }
  log_model_->Clear();
}

void LogView::SetMaximumEntries(int maximum_entries) {
  if (QThread::currentThread() != thread()) {
    QMetaObject::invokeMethod(
        this, [this, maximum_entries]() { SetMaximumEntries(maximum_entries); },
        Qt::QueuedConnection);
    return;
  }
  log_model_->SetMaximumEntries(maximum_entries);
}

void LogView::SetMinimumLevel(LogLevel level) {
  if (minimum_level_ == level) {
    return;
  }
  minimum_level_ = level;
  static_cast<LogFilterProxyModel*>(proxy_model_)->SetMinimumLevel(level);
}

void LogView::SetFilterText(const QString& text) {
  static_cast<LogFilterProxyModel*>(proxy_model_)->SetText(text);
}

QString LogView::filter_text() const { return search_box_->text(); }

void LogView::SetAutoScroll(bool enabled) {
  if (auto_scroll_ == enabled) {
    return;
  }
  auto_scroll_ = enabled;
  pause_button_->setChecked(!enabled);
  UpdatePauseButton();
  if (enabled) {
    view_->scrollToBottom();
  }
  emit AutoScrollChanged(enabled);
}

void LogView::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  search_box_->SetTone(tone);
  pause_button_->SetTone(tone);
  clear_button_->SetTone(tone);
  copy_button_->SetTone(tone);
  ApplyWidgetTheme(view_, tone);
}

void LogView::CopySelectedRows() const {
  const QModelIndexList selected = view_->selectionModel()->selectedRows();
  if (selected.isEmpty()) {
    return;
  }
  QString text;
  for (const QModelIndex& row : selected) {
    QStringList columns;
    columns.reserve(static_cast<qsizetype>(LogModel::kColumnCount));
    for (int column = 0; column < LogModel::kColumnCount; ++column) {
      columns.push_back(
          proxy_model_->index(row.row(), column).data().toString());
    }
    if (!text.isEmpty()) {
      text += QLatin1Char('\n');
    }
    text += columns.join(QLatin1Char('\t'));
  }
  QApplication::clipboard()->setText(text);
}

void LogView::ScrollToBottomIfEnabled() {
  if (!auto_scroll_ || !isVisible() || scroll_scheduled_) {
    return;
  }
  scroll_scheduled_ = true;
  QTimer::singleShot(0, view_, [this]() {
    scroll_scheduled_ = false;
    if (auto_scroll_ && isVisible()) {
      view_->scrollToBottom();
    }
  });
}

void LogView::UpdatePauseButton() {
  pause_button_->setText(auto_scroll_ ? tr("Pause") : tr("Resume"));
  pause_button_->setAccessibleName(auto_scroll_
                                       ? tr("Pause automatic scroll")
                                       : tr("Resume automatic scroll"));
}

}  // namespace autoviz::widget
