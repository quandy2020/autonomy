/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_dock_widget.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QResizeEvent>
#include <QShowEvent>
#include <QTimer>
#include <QToolButton>

#include <algorithm>

#include "autoviz/ui/icon_loader.hpp"

namespace autoviz {

PanelDockWidget::PanelDockWidget(const QString& name, QWidget* parent)
    : QDockWidget(name, parent) {
  title_bar_ = new QWidget(this);
  QPalette pal = palette();
  pal.setColor(QPalette::Window, QApplication::palette().color(QPalette::Mid));
  title_bar_->setAutoFillBackground(true);
  title_bar_->setPalette(pal);

  icon_label_ = new QLabel(title_bar_);
  icon_label_->setContentsMargins(2, 2, 0, 0);
  icon_label_->setVisible(false);

  title_label_ = new QLabel(name, title_bar_);

  auto* close_button = new QToolButton(title_bar_);
  close_button->setIcon(IconLoader::load(QStringLiteral(":/autoviz/icons/close.svg")));
  close_button->setIconSize(QSize(16, 16));
  close_button->setAutoRaise(true);
  connect(close_button, &QToolButton::clicked, this, &QDockWidget::close);

  auto* title_layout = new QHBoxLayout(title_bar_);
  title_layout->setContentsMargins(2, 2, 2, 2);
  title_layout->addWidget(icon_label_);
  title_layout->addWidget(title_label_, 1);
  title_layout->addWidget(close_button);
  setTitleBarWidget(title_bar_);
}

void PanelDockWidget::setContentWidget(QWidget* child) {
  if (widget() != nullptr) {
    disconnect(widget(), &QObject::destroyed, this,
               &PanelDockWidget::onChildDestroyed);
  }
  setWidget(child);
  if (child != nullptr) {
    connect(child, &QObject::destroyed, this,
            &PanelDockWidget::onChildDestroyed);
    applyFixedContentHeight();
    enforceFixedHeight();
  }
}

QMainWindow* PanelDockWidget::mainWindow() const {
  return qobject_cast<QMainWindow*>(parentWidget());
}

int PanelDockWidget::fixedDockHeight() const {
  if (fixed_content_height_ <= 0) {
    return 0;
  }
  int title_h = 0;
  if (title_bar_ != nullptr && titleBarWidget() == title_bar_) {
    title_h = title_bar_->sizeHint().height();
    if (title_h <= 0) {
      title_h = 24;
    }
  }
  return title_h + fixed_content_height_;
}

void PanelDockWidget::applyFixedContentHeight() {
  if (fixed_content_height_ <= 0) {
    if (QWidget* content = widget()) {
      content->setMinimumHeight(0);
      content->setMaximumHeight(QWIDGETSIZE_MAX);
      content->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    }
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    setMinimumHeight(0);
    setMaximumHeight(QWIDGETSIZE_MAX);
    return;
  }

  if (QWidget* content = widget()) {
    content->setFixedHeight(fixed_content_height_);
    content->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }

  // Qt applies dock sizing from the child widget; avoid min/max on the dock itself.
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  setMinimumHeight(0);
  setMaximumHeight(QWIDGETSIZE_MAX);
}

void PanelDockWidget::installFixedHeightHooks() {
  if (fixed_height_hooks_installed_ || fixed_content_height_ <= 0) {
    return;
  }
  fixed_height_hooks_installed_ = true;

  connect(this, &QDockWidget::dockLocationChanged, this,
          [this](Qt::DockWidgetArea) { enforceFixedHeight(); });
  connect(this, &QDockWidget::topLevelChanged, this,
          [this](bool) { enforceFixedHeight(); });

  if (QMainWindow* mw = mainWindow()) {
    mw->installEventFilter(this);
  }
}

void PanelDockWidget::setFixedContentHeight(int height) {
  fixed_content_height_ = std::max(0, height);
  applyFixedContentHeight();
  installFixedHeightHooks();
  enforceFixedHeight();
}

void PanelDockWidget::enforceFixedHeight() {
  if (height_enforcing_ || fixed_content_height_ <= 0 || isFloating() ||
      !isVisible()) {
    return;
  }

  QMainWindow* mw = mainWindow();
  if (mw == nullptr) {
    return;
  }

  const int target = fixedDockHeight();
  if (target <= 0) {
    return;
  }

  if (qAbs(height() - target) <= 1) {
    return;
  }

  height_enforcing_ = true;
  mw->resizeDocks({this}, {target}, Qt::Vertical);
  height_enforcing_ = false;
}

void PanelDockWidget::setPanelIcon(const QIcon& icon) {
  if (icon.isNull()) {
    icon_label_->setVisible(false);
    return;
  }
  icon_label_->setVisible(true);
  icon_label_->setPixmap(icon.pixmap(16, 16));
}

void PanelDockWidget::setCollapsed(bool collapse) {
  if (collapsed_ == collapse || isFloating()) {
    return;
  }
  if (collapse) {
    if (isVisible()) {
      PanelDockWidget::setVisible(false);
      collapsed_ = true;
    }
  } else {
    PanelDockWidget::setVisible(true);
    collapsed_ = false;
  }
}

void PanelDockWidget::overrideVisibility(bool hidden) {
  forced_hidden_ = hidden;
  setVisible(requested_visibility_);
}

void PanelDockWidget::setVisible(bool visible) {
  requested_visibility_ = visible;
  QDockWidget::setVisible(requested_visibility_ && !forced_hidden_);
}

void PanelDockWidget::showEvent(QShowEvent* event) {
  QDockWidget::showEvent(event);
  applyFixedContentHeight();
  installFixedHeightHooks();
  if (QMainWindow* mw = mainWindow()) {
    mw->installEventFilter(this);
  }
  enforceFixedHeight();
}

void PanelDockWidget::resizeEvent(QResizeEvent* event) {
  QDockWidget::resizeEvent(event);
  if (fixed_content_height_ <= 0 || isFloating() || height_enforcing_) {
    return;
  }
  const int target = fixedDockHeight();
  if (target > 0 && qAbs(height() - target) > 1) {
    QTimer::singleShot(0, this, &PanelDockWidget::enforceFixedHeight);
  }
}

bool PanelDockWidget::eventFilter(QObject* watched, QEvent* event) {
  if (fixed_content_height_ > 0 && watched == mainWindow()) {
    switch (event->type()) {
      case QEvent::Resize:
      case QEvent::LayoutRequest:
        QTimer::singleShot(0, this, &PanelDockWidget::enforceFixedHeight);
        break;
      default:
        break;
    }
  }
  return QDockWidget::eventFilter(watched, event);
}

void PanelDockWidget::closeEvent(QCloseEvent* event) {
  QDockWidget::closeEvent(event);
  emit closed();
}

void PanelDockWidget::onChildDestroyed(QObject* /*child*/) { deleteLater(); }

}  // namespace autoviz
