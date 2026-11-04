/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel/change_menu.hpp"

#include <QApplication>
#include <QEvent>
#include <QGuiApplication>
#include <QLineEdit>
#include <QScreen>
#include <QShowEvent>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/widget/input/search_box.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/panel/catalog.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

constexpr int kPickerWidthPx = 260;
constexpr int kRowHeightPx = 28;

}  // namespace

ChangePanelMenuWidget::ChangePanelMenuWidget(QWidget* parent)
    : QWidget(parent, Qt::Popup | Qt::FramelessWindowHint) {
  setObjectName(QStringLiteral("AutovizChangePanelPopup"));
  setAttribute(Qt::WA_DeleteOnClose, false);
  setAttribute(Qt::WA_StyledBackground, true);
  setAutoFillBackground(false);
  setMinimumWidth(kPickerWidthPx);
  setMaximumWidth(320);
  setStyleSheet(style::sheet(QStringLiteral("change_panel")));

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(6);

  search_ = new widget::SearchBox(this);
  search_->setAccessibleName(tr("Search panels"));
  search_->setPlaceholderText(tr("Search panels"));
  root->addWidget(search_);

  rows_host_ = new QWidget(this);
  rows_host_->setObjectName(QStringLiteral("changePanelRows"));
  rows_layout_ = new QVBoxLayout(rows_host_);
  rows_layout_->setContentsMargins(0, 0, 0, 0);
  rows_layout_->setSpacing(0);
  root->addWidget(rows_host_);

  connect(search_, &QLineEdit::textChanged, this,
          &ChangePanelMenuWidget::onFilterChanged);

  populate();
}

void ChangePanelMenuWidget::popupBelow(QWidget* anchor) {
  refitToContents();
  const QSize hint = sizeHint();
  resize(hint);

  QPoint pos;
  if (anchor != nullptr) {
    pos = anchor->mapToGlobal(QPoint(0, anchor->height()));
  } else if (QWidget* w = QApplication::activeWindow()) {
    pos = w->mapToGlobal(QPoint(8, 8));
  }

  // Keep the full column on-screen: prefer below the anchor, else above, and
  // never shrink into a scrollable viewport.
  if (QScreen* screen = QGuiApplication::screenAt(pos)) {
    const QRect avail = screen->availableGeometry();
    if (pos.x() + hint.width() > avail.right()) {
      pos.setX(avail.right() - hint.width());
    }
    if (pos.x() < avail.left()) {
      pos.setX(avail.left());
    }
    if (pos.y() + hint.height() > avail.bottom()) {
      // Flip above the anchor when there is room.
      if (anchor != nullptr) {
        const QPoint above =
            anchor->mapToGlobal(QPoint(0, -hint.height()));
        if (above.y() >= avail.top()) {
          pos.setY(above.y());
        } else {
          pos.setY(avail.top());
        }
      } else {
        pos.setY(avail.bottom() - hint.height());
      }
    }
    if (pos.y() < avail.top()) {
      pos.setY(avail.top());
    }
  }

  move(pos);
  show();
  raise();
  activateWindow();
  if (search_ != nullptr) {
    search_->setFocus(Qt::PopupFocusReason);
  }
}

QSize ChangePanelMenuWidget::sizeHint() const {
  const QMargins margins =
      layout() != nullptr ? layout()->contentsMargins() : QMargins();
  const int spacing = layout() != nullptr ? layout()->spacing() : 0;
  const int search_h =
      search_ != nullptr ? search_->sizeHint().height() : 0;

  int rows_h = 0;
  int visible = 0;
  for (const Row& row : rows_) {
    if (row.button == nullptr || row.button->isHidden()) {
      continue;
    }
    rows_h += kRowHeightPx;
    ++visible;
  }
  if (visible == 0) {
    rows_h = kRowHeightPx;
  }

  return QSize(qMax(kPickerWidthPx, minimumWidth()),
               margins.top() + margins.bottom() + search_h + spacing + rows_h);
}

QSize ChangePanelMenuWidget::minimumSizeHint() const { return sizeHint(); }

void ChangePanelMenuWidget::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  refitToContents();
}

bool ChangePanelMenuWidget::event(QEvent* event) {
  // Qt::Popup closes on outside click; keep wheel from being swallowed oddly.
  if (event != nullptr && event->type() == QEvent::Wheel) {
    event->ignore();
    return true;
  }
  return QWidget::event(event);
}

void ChangePanelMenuWidget::refitToContents() {
  setMinimumHeight(0);
  setMaximumHeight(QWIDGETSIZE_MAX);
  if (rows_host_ != nullptr) {
    int rows_h = 0;
    for (const Row& row : rows_) {
      if (row.button == nullptr || row.button->isHidden()) {
        continue;
      }
      rows_h += kRowHeightPx;
    }
    rows_host_->setFixedHeight(qMax(rows_h, kRowHeightPx));
  }
  updateGeometry();
  const QSize hint = sizeHint();
  setFixedSize(hint);
}

void ChangePanelMenuWidget::populate() {
  while (QLayoutItem* item = rows_layout_->takeAt(0)) {
    if (QWidget* w = item->widget()) {
      w->deleteLater();
    }
    delete item;
  }
  rows_.clear();

  for (const PanelCatalogEntry& entry : PanelCatalog()) {
    if (!entry.isImplemented()) {
      continue;
    }
    if (QLatin1String(entry.object_name) == QLatin1String("TeleopDock")) {
      continue;
    }

    const QString label = tr(entry.label);
    const QString object_name = QString::fromLatin1(entry.object_name);

    auto* button = new QToolButton(rows_host_);
    button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    button->setIcon(IconLoader::panelsMenuDockIcon(object_name));
    button->setIconSize(QSize(16, 16));
    button->setText(label);
    button->setAutoRaise(true);
    button->setCursor(Qt::PointingHandCursor);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    button->setFixedHeight(kRowHeightPx);

    connect(button, &QToolButton::clicked, this, [this, object_name]() {
      if (selecting_) {
        return;
      }
      selecting_ = true;
      emit panelSelected(object_name);
      selecting_ = false;
      close();
    });

    rows_layout_->addWidget(button);
    rows_.push_back(Row{button, label, object_name});
  }

  onFilterChanged(search_ != nullptr ? search_->text() : QString());
}

void ChangePanelMenuWidget::onFilterChanged(const QString& text) {
  const QString needle = text.trimmed();
  for (const Row& row : rows_) {
    if (row.button == nullptr) {
      continue;
    }
    const bool match =
        needle.isEmpty() ||
        row.label.contains(needle, Qt::CaseInsensitive) ||
        row.object_name.contains(needle, Qt::CaseInsensitive);
    row.button->setVisible(match);
  }
  refitToContents();
}

void ShowChangePanelPopup(
    QWidget* anchor,
    const std::function<void(const QString& object_name)>& on_selected) {
  auto* picker = new ChangePanelMenuWidget(nullptr);
  // Popup is a window; keep lifetime tied to the call, delete on close.
  picker->setAttribute(Qt::WA_DeleteOnClose, true);
  QObject::connect(picker, &ChangePanelMenuWidget::panelSelected, picker,
                   [on_selected](const QString& object_name) {
                     if (on_selected) {
                       on_selected(object_name);
                     }
                   });
  picker->popupBelow(anchor);
}

}  // namespace autoviz
