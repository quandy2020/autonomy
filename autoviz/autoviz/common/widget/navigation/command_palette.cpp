/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/navigation/command_palette.hpp"

#include <QAbstractItemModel>
#include <QEvent>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QLabel>
#include <QListView>
#include <QModelIndex>
#include <QScreen>
#include <QShowEvent>
#include <QSortFilterProxyModel>
#include <QVBoxLayout>
#include <QVariant>
#include <utility>

#include "autoviz/common/widget/foundation/widget_theme.hpp"
#include "autoviz/common/widget/input/search_box.hpp"

namespace autoviz::widget {

class CommandPalette::CommandFilterProxyModel final
    : public QSortFilterProxyModel {
 public:
  explicit CommandFilterProxyModel(QObject* parent)
      : QSortFilterProxyModel(parent) {
    setDynamicSortFilter(true);
    setFilterCaseSensitivity(Qt::CaseInsensitive);
  }

  void SetQuery(QString query) {
    query_ = std::move(query).trimmed();
    RefreshFilter();
  }

  void SetMatchMode(MatchMode mode) {
    if (mode_ == mode) {
      return;
    }
    mode_ = mode;
    RefreshFilter();
  }

 protected:
  bool filterAcceptsRow(int source_row,
                        const QModelIndex& source_parent) const override {
    if (query_.isEmpty()) {
      return true;
    }
    const QAbstractItemModel* source = sourceModel();
    if (source == nullptr) {
      return false;
    }
    const QString candidate = source->index(source_row, 0, source_parent)
                                  .data(Qt::DisplayRole)
                                  .toString();
    if (mode_ == MatchMode::kContains) {
      return candidate.contains(query_, Qt::CaseInsensitive);
    }

    qsizetype query_index = 0;
    for (const QChar character : candidate) {
      if (query_index < query_.size() &&
          character.toCaseFolded() == query_[query_index].toCaseFolded()) {
        ++query_index;
      }
    }
    return query_index == query_.size();
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

  QString query_;
  MatchMode mode_ = MatchMode::kFuzzy;
};

CommandPalette::CommandPalette(QWidget* parent)
    : QDialog(parent, Qt::Dialog | Qt::FramelessWindowHint),
      proxy_model_(new CommandFilterProxyModel(this)),
      search_box_(new SearchBox(this)),
      list_view_(new QListView(this)) {
  setObjectName(QStringLiteral("AutovizCommandPalette"));
  setProperty("autovizRole", QStringLiteral("commandPalette"));
  setAccessibleName(tr("Command palette"));
  setModal(false);
  setSizeGripEnabled(false);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

  auto* layout = new QVBoxLayout(this);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  layout->setContentsMargins(tokens.spacing_md, tokens.spacing_md,
                             tokens.spacing_md, tokens.spacing_md);
  layout->setSpacing(tokens.spacing_sm);

  auto* title = new QLabel(tr("Commands"), this);
  QFont title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);
  layout->addWidget(title);

  search_box_->setPlaceholderText(tr("Type a command"));
  search_box_->setAccessibleName(tr("Search commands"));
  search_box_->installEventFilter(this);
  layout->addWidget(search_box_);

  list_view_->setModel(proxy_model_);
  list_view_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  list_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
  list_view_->setSelectionMode(QAbstractItemView::SingleSelection);
  list_view_->setUniformItemSizes(true);
  list_view_->setVerticalScrollMode(QAbstractItemView::ScrollPerItem);
  list_view_->setAccessibleName(tr("Matching commands"));
  list_view_->installEventFilter(this);
  layout->addWidget(list_view_, 1);

  connect(search_box_, &QLineEdit::textChanged, this,
          [this](const QString& query) {
            proxy_model_->SetQuery(query);
            SelectFirstVisible();
          });
  connect(search_box_, &SearchBox::SearchRequested, this,
          [this](const QString&) { ActivateCurrent(); });
  connect(list_view_, &QListView::activated, this,
          [this](const QModelIndex&) { ActivateCurrent(); });
  const auto select_when_ready = [this]() {
    if (!list_view_->currentIndex().isValid()) {
      SelectFirstVisible();
    }
  };
  connect(proxy_model_, &QAbstractItemModel::rowsInserted, this,
          [select_when_ready](const QModelIndex&, int, int) {
            select_when_ready();
          });
  connect(proxy_model_, &QAbstractItemModel::modelReset, this,
          select_when_ready);

  ApplyWidgetTheme(this, tone_, true);
}

void CommandPalette::SetModel(QAbstractItemModel* model) {
  if (model_.data() == model) {
    return;
  }
  model_ = model;
  proxy_model_->setSourceModel(model);
  SelectFirstVisible();
}

void CommandPalette::SetMatchMode(MatchMode mode) {
  if (match_mode_ == mode) {
    return;
  }
  match_mode_ = mode;
  proxy_model_->SetMatchMode(mode);
  SelectFirstVisible();
}

void CommandPalette::SetRoles(int id_role, int data_role) {
  id_role_ = id_role;
  data_role_ = data_role;
}

void CommandPalette::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  search_box_->SetTone(tone);
  ApplyWidgetTheme(this, tone_, true);
}

void CommandPalette::ShowPalette() {
  search_box_->clear();
  proxy_model_->SetQuery(QString());
  SelectFirstVisible();
  adjustSize();
  show();
  raise();
  activateWindow();
  search_box_->setFocus(Qt::ShortcutFocusReason);
  search_box_->selectAll();
}

void CommandPalette::Dismiss() { hide(); }

QSize CommandPalette::sizeHint() const { return QSize(640, 420); }

bool CommandPalette::eventFilter(QObject* watched, QEvent* event) {
  if ((watched == search_box_ || watched == list_view_) &&
      event->type() == QEvent::KeyPress) {
    auto* key_event = static_cast<QKeyEvent*>(event);
    switch (key_event->key()) {
      case Qt::Key_Escape:
        Dismiss();
        return true;
      case Qt::Key_Down:
        MoveCurrent(1);
        return true;
      case Qt::Key_Up:
        MoveCurrent(-1);
        return true;
      case Qt::Key_Return:
      case Qt::Key_Enter:
        ActivateCurrent();
        return true;
      default:
        break;
    }
  }
  return QDialog::eventFilter(watched, event);
}

void CommandPalette::showEvent(QShowEvent* event) {
  QDialog::showEvent(event);
  QWidget* host = parentWidget();
  if (host != nullptr) {
    const QPoint center = host->mapToGlobal(host->rect().center());
    move(center.x() - width() / 2, center.y() - height() / 3);
    return;
  }
  QScreen* screen = QGuiApplication::primaryScreen();
  if (screen != nullptr) {
    const QRect available = screen->availableGeometry();
    move(available.center().x() - width() / 2,
         available.top() + available.height() / 5);
  }
}

void CommandPalette::ActivateCurrent() {
  const QModelIndex proxy_index = list_view_->currentIndex();
  if (!proxy_index.isValid() || model_ == nullptr) {
    return;
  }
  const QModelIndex source_index = proxy_model_->mapToSource(proxy_index);
  if (!source_index.isValid()) {
    return;
  }
  const QString command_id = source_index.data(id_role_).toString();
  const QVariant data = source_index.data(data_role_);
  QPointer<CommandPalette> guard(this);
  emit SourceIndexActivated(source_index);
  if (guard.isNull()) {
    return;
  }
  emit CommandActivated(command_id, data);
  if (guard.isNull()) {
    return;
  }
  Dismiss();
}

void CommandPalette::MoveCurrent(int offset) {
  const int rows = proxy_model_->rowCount();
  if (rows <= 0) {
    return;
  }
  int row = list_view_->currentIndex().row();
  if (row < 0) {
    row = offset > 0 ? 0 : rows - 1;
  } else {
    row = qBound(0, row + offset, rows - 1);
  }
  const QModelIndex next = proxy_model_->index(row, 0);
  list_view_->setCurrentIndex(next);
  list_view_->scrollTo(next, QAbstractItemView::EnsureVisible);
}

void CommandPalette::SelectFirstVisible() {
  const QModelIndex first = proxy_model_->index(0, 0);
  list_view_->setCurrentIndex(first);
  if (first.isValid()) {
    list_view_->scrollTo(first, QAbstractItemView::PositionAtTop);
  }
}

}  // namespace autoviz::widget
