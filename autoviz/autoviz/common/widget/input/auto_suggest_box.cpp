/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/auto_suggest_box.hpp"

#include <QAbstractItemModel>
#include <QFocusEvent>
#include <QFrame>
#include <QGuiApplication>
#include <QHideEvent>
#include <QKeyEvent>
#include <QListView>
#include <QMoveEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QScreen>
#include <QStyleOptionViewItem>
#include <QStyledItemDelegate>
#include <QTimer>
#include <QVBoxLayout>
#include <QVariant>
#include <algorithm>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {
namespace {

QPalette TransparentListPalette(const QPalette& palette,
                                const ThemeTokens& tokens) {
  QPalette transparent = palette;
  transparent.setColor(QPalette::Base, Qt::transparent);
  transparent.setColor(QPalette::AlternateBase, Qt::transparent);
  transparent.setColor(QPalette::Window, Qt::transparent);
  transparent.setColor(QPalette::Highlight, tokens.surface_selected);
  transparent.setColor(QPalette::HighlightedText, tokens.text);
  return transparent;
}

class SuggestionPopup final : public QFrame {
 public:
  explicit SuggestionPopup(QWidget* parent)
      : QFrame(parent, Qt::Popup | Qt::FramelessWindowHint) {
    setAttribute(Qt::WA_TranslucentBackground);
    setAutoFillBackground(false);
    setFrameShape(QFrame::NoFrame);
  }

  void SetTone(style::Tone tone) {
    if (tone_ == tone) {
      return;
    }
    tone_ = tone;
    update();
  }

 protected:
  void paintEvent(QPaintEvent* event) override {
    Q_UNUSED(event);
    const ThemeTokens& tokens = GetThemeTokens(this, tone_);
    const QRectF surface_rect = QRectF(rect()).adjusted(1.5, 1.5, -1.5, -3.0);
    QColor shadow = tokens.shadow_ambient;
    shadow.setAlpha(qRound(shadow.alpha() * 0.65));

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::NoPen);
    painter.setBrush(shadow);
    painter.drawRoundedRect(surface_rect.translated(0.0, 1.5),
                            tokens.radius_medium, tokens.radius_medium);

    painter.setPen(QPen(tokens.border_subtle, 1.0));
    painter.setBrush(tokens.surface_raised);
    painter.drawRoundedRect(surface_rect, tokens.radius_medium,
                            tokens.radius_medium);
  }

 private:
  style::Tone tone_ = style::Tone::Shell;
};

class SuggestionItemDelegate final : public QStyledItemDelegate {
 public:
  SuggestionItemDelegate(style::Tone tone, QObject* parent)
      : QStyledItemDelegate(parent), tone_(tone) {}

  void SetTone(style::Tone tone) { tone_ = tone; }

  QSize sizeHint(const QStyleOptionViewItem& option,
                 const QModelIndex& index) const override {
    QSize hint = QStyledItemDelegate::sizeHint(option, index);
    const ThemeTokens& tokens = GetThemeTokens(option.widget, tone_);
    hint.setHeight(qMax(tokens.control_height_medium, hint.height()));
    return hint;
  }

 private:
  style::Tone tone_ = style::Tone::Shell;
};

}  // namespace

AutoSuggestBox::AutoSuggestBox(QWidget* parent)
    : QLineEdit(parent),
      popup_(new SuggestionPopup(this)),
      list_view_(new QListView(popup_)),
      debounce_timer_(new QTimer(this)) {
  setObjectName(QStringLiteral("AutovizAutoSuggestBox"));
  setProperty("autovizRole", QStringLiteral("autoSuggestBox"));
  setAccessibleName(tr("Suggestions"));
  setClearButtonEnabled(true);
  setFocusPolicy(Qt::StrongFocus);

  popup_->setObjectName(QStringLiteral("AutovizAutoSuggestPopup"));
  popup_->setAttribute(Qt::WA_ShowWithoutActivating);
  popup_->hide();
  auto* popup_layout = new QVBoxLayout(popup_);
  popup_layout->setContentsMargins(4, 4, 4, 6);
  popup_layout->setSpacing(0);
  popup_layout->addWidget(list_view_);

  list_view_->setObjectName(QStringLiteral("AutovizAutoSuggestList"));
  list_view_->setAttribute(Qt::WA_TranslucentBackground);
  list_view_->setAutoFillBackground(false);
  list_view_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  list_view_->setFrameShape(QFrame::NoFrame);
  list_view_->setFocusPolicy(Qt::NoFocus);
  list_view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  list_view_->setMouseTracking(true);
  list_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
  list_view_->setSelectionMode(QAbstractItemView::SingleSelection);
  list_view_->setUniformItemSizes(true);
  list_view_->setVerticalScrollMode(QAbstractItemView::ScrollPerItem);
  list_view_->viewport()->setAttribute(Qt::WA_TranslucentBackground);
  list_view_->viewport()->setAutoFillBackground(false);
  list_view_->setItemDelegate(new SuggestionItemDelegate(tone_, list_view_));

  debounce_timer_->setSingleShot(true);
  debounce_timer_->setInterval(debounce_ms_);
  connect(debounce_timer_, &QTimer::timeout, this,
          [this]() { emit QueryRequested(text().trimmed()); });
  connect(this, &QLineEdit::textEdited, this, &AutoSuggestBox::ScheduleQuery);
  connect(list_view_, &QListView::clicked, this,
          &AutoSuggestBox::ActivateIndex);

  const QPalette themed = BuildWidgetPalette(tone_, palette());
  setPalette(themed);
  popup_->setPalette(themed);
  const QPalette list_palette =
      TransparentListPalette(themed, GetThemeTokens(this, tone_));
  list_view_->setPalette(list_palette);
  list_view_->viewport()->setPalette(list_palette);
}

AutoSuggestBox::~AutoSuggestBox() {
  debounce_timer_->stop();
  DisconnectModelSignals();
  RemovePositionWatchers();
  popup_->hide();
}

void AutoSuggestBox::SetModel(QAbstractItemModel* model) {
  if (model_.data() == model) {
    return;
  }
  DisconnectModelSignals();
  model_ = model;
  list_view_->setModel(model);
  if (model == nullptr) {
    HideSuggestions();
    return;
  }

  const auto refresh = [this]() {
    if (hasFocus() && !text().isEmpty()) {
      ShowSuggestions();
    } else {
      HideSuggestions();
    }
  };
  model_reset_connection_ =
      connect(model, &QAbstractItemModel::modelReset, this, refresh);
  rows_inserted_connection_ =
      connect(model, &QAbstractItemModel::rowsInserted, this,
              [refresh](const QModelIndex&, int, int) { refresh(); });
  rows_removed_connection_ =
      connect(model, &QAbstractItemModel::rowsRemoved, this,
              [refresh](const QModelIndex&, int, int) { refresh(); });
  model_destroyed_connection_ =
      connect(model, &QObject::destroyed, this, [this]() {
        model_ = nullptr;
        HideSuggestions();
      });
  refresh();
}

void AutoSuggestBox::SetDebounceMs(int debounce_ms) {
  debounce_ms_ = std::max(0, debounce_ms);
  debounce_timer_->setInterval(debounce_ms_);
}

void AutoSuggestBox::SetMaximumVisibleItems(int maximum_items) {
  maximum_visible_items_ = std::max(1, maximum_items);
  if (popup_->isVisible()) {
    ShowSuggestions();
  }
}

void AutoSuggestBox::SetDataRole(int data_role) { data_role_ = data_role; }

void AutoSuggestBox::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  const QPalette themed = BuildWidgetPalette(tone_, palette());
  setPalette(themed);
  popup_->setPalette(themed);
  static_cast<SuggestionPopup*>(popup_)->SetTone(tone_);
  const QPalette list_palette =
      TransparentListPalette(themed, GetThemeTokens(this, tone_));
  list_view_->setPalette(list_palette);
  list_view_->viewport()->setPalette(list_palette);
  static_cast<SuggestionItemDelegate*>(list_view_->itemDelegate())
      ->SetTone(tone_);
  list_view_->doItemsLayout();
  list_view_->viewport()->update();
}

QSize AutoSuggestBox::sizeHint() const {
  QSize hint = QLineEdit::sizeHint();
  hint.setWidth(qMax(200, hint.width()));
  hint.setHeight(GetThemeTokens(this, tone_).control_height_medium);
  return hint;
}

void AutoSuggestBox::focusOutEvent(QFocusEvent* event) {
  QLineEdit::focusOutEvent(event);
  QTimer::singleShot(0, this, [this]() {
    if (!hasFocus() && !popup_->isActiveWindow() && !popup_->underMouse()) {
      HideSuggestions();
    }
  });
}

bool AutoSuggestBox::eventFilter(QObject* watched, QEvent* event) {
  Q_UNUSED(watched);
  if (popup_->isVisible()) {
    switch (event->type()) {
      case QEvent::Move:
      case QEvent::Resize:
      case QEvent::ScreenChangeInternal:
        RepositionPopup();
        break;
      case QEvent::ParentChange:
        QTimer::singleShot(0, this, &AutoSuggestBox::InstallPositionWatchers);
        break;
      default:
        break;
    }
  }
  return QLineEdit::eventFilter(watched, event);
}

void AutoSuggestBox::hideEvent(QHideEvent* event) {
  debounce_timer_->stop();
  HideSuggestions();
  QLineEdit::hideEvent(event);
}

void AutoSuggestBox::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Down:
      MoveCurrent(1);
      event->accept();
      return;
    case Qt::Key_Up:
      MoveCurrent(-1);
      event->accept();
      return;
    case Qt::Key_Return:
    case Qt::Key_Enter: {
      const QModelIndex current = list_view_->currentIndex();
      if (popup_->isVisible() && current.isValid()) {
        ActivateIndex(current);
      } else {
        debounce_timer_->stop();
        emit QueryRequested(text().trimmed());
      }
      event->accept();
      return;
    }
    case Qt::Key_Escape:
      debounce_timer_->stop();
      if (popup_->isVisible()) {
        HideSuggestions();
      } else if (!text().isEmpty()) {
        clear();
        ScheduleQuery(QString());
      }
      event->accept();
      return;
    default:
      QLineEdit::keyPressEvent(event);
  }
}

void AutoSuggestBox::moveEvent(QMoveEvent* event) {
  QLineEdit::moveEvent(event);
  if (popup_->isVisible()) {
    RepositionPopup();
  }
}

void AutoSuggestBox::resizeEvent(QResizeEvent* event) {
  QLineEdit::resizeEvent(event);
  if (popup_->isVisible()) {
    RepositionPopup();
  }
}

void AutoSuggestBox::ActivateIndex(const QModelIndex& index) {
  if (!index.isValid() || model_ == nullptr) {
    return;
  }
  const QString display = index.data(Qt::DisplayRole).toString();
  const QVariant data = index.data(data_role_);
  setText(display);
  HideSuggestions();
  emit SuggestionActivated(index, data);
}

void AutoSuggestBox::DisconnectModelSignals() {
  QObject::disconnect(model_reset_connection_);
  QObject::disconnect(rows_inserted_connection_);
  QObject::disconnect(rows_removed_connection_);
  QObject::disconnect(model_destroyed_connection_);
  model_reset_connection_ = {};
  rows_inserted_connection_ = {};
  rows_removed_connection_ = {};
  model_destroyed_connection_ = {};
}

void AutoSuggestBox::HideSuggestions() {
  popup_->hide();
  if (list_view_->model() != nullptr) {
    list_view_->setCurrentIndex(QModelIndex());
  }
  RemovePositionWatchers();
}

void AutoSuggestBox::InstallPositionWatchers() {
  RemovePositionWatchers();
  for (QWidget* ancestor = parentWidget(); ancestor != nullptr;
       ancestor = ancestor->parentWidget()) {
    ancestor->installEventFilter(this);
    position_watchers_.push_back(ancestor);
  }
}

void AutoSuggestBox::MoveCurrent(int offset) {
  if (model_ == nullptr || model_->rowCount() <= 0) {
    return;
  }
  ShowSuggestions();
  const int row_count = model_->rowCount();
  int row = list_view_->currentIndex().row();
  if (row < 0) {
    row = offset > 0 ? 0 : row_count - 1;
  } else {
    row = qBound(0, row + offset, row_count - 1);
  }
  const QModelIndex next = model_->index(row, 0);
  list_view_->setCurrentIndex(next);
  list_view_->scrollTo(next, QAbstractItemView::EnsureVisible);
}

void AutoSuggestBox::RepositionPopup() {
  if (!popup_->isVisible()) {
    return;
  }
  const QPoint anchor = mapToGlobal(QPoint(0, height()));
  QPoint position = anchor;
  QScreen* screen = QGuiApplication::screenAt(anchor);
  if (screen == nullptr) {
    screen = QGuiApplication::primaryScreen();
  }
  if (screen != nullptr) {
    const QRect available = screen->availableGeometry();
    position.setX(qBound(
        available.left(), position.x(),
        qMax(available.left(), available.right() - popup_->width() + 1)));
    if (position.y() + popup_->height() > available.bottom()) {
      position.setY(mapToGlobal(QPoint(0, 0)).y() - popup_->height());
    }
    position.setY(qBound(
        available.top(), position.y(),
        qMax(available.top(), available.bottom() - popup_->height() + 1)));
  }
  popup_->move(position);
}

void AutoSuggestBox::RemovePositionWatchers() {
  for (const QPointer<QObject>& watched : position_watchers_) {
    if (watched != nullptr) {
      watched->removeEventFilter(this);
    }
  }
  position_watchers_.clear();
}

void AutoSuggestBox::ScheduleQuery(const QString& query) {
  debounce_timer_->stop();
  HideSuggestions();
  if (query.trimmed().isEmpty()) {
    emit QueryRequested(QString());
    return;
  }
  if (debounce_ms_ == 0) {
    emit QueryRequested(query.trimmed());
  } else if (isVisible()) {
    debounce_timer_->start();
  }
}

void AutoSuggestBox::ShowSuggestions() {
  if (model_ == nullptr || model_->rowCount() <= 0 || text().isEmpty()) {
    HideSuggestions();
    return;
  }
  const int visible_rows = std::min(maximum_visible_items_, model_->rowCount());
  int row_height = list_view_->sizeHintForRow(0);
  if (row_height <= 0) {
    row_height = fontMetrics().height() + 10;
  }
  const QMargins margins = popup_->layout()->contentsMargins();
  popup_->resize(width(),
                 visible_rows * row_height + margins.top() + margins.bottom());
  if (!list_view_->currentIndex().isValid()) {
    list_view_->setCurrentIndex(model_->index(0, 0));
  }
  popup_->show();
  popup_->raise();
  InstallPositionWatchers();
  RepositionPopup();
}

}  // namespace autoviz::widget
