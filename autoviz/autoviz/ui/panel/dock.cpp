/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel/dock.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QEvent>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QResizeEvent>
#include <QShowEvent>
#include <QSizePolicy>
#include <QTimer>
#include <QToolButton>

#include <algorithm>

#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

class DockGlassTitleBar : public QWidget {
 public:
  explicit DockGlassTitleBar(QWidget* parent = nullptr) : QWidget(parent) {
    setObjectName(QString::fromLatin1(AppThemeIds::kDockTitleBar));
    setAttribute(Qt::WA_StyledBackground, false);
    setAutoFillBackground(false);
    setFixedHeight(28);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setMinimumWidth(0);
  }

  void setTitleLabel(QLabel* label) { title_label_ = label; }

  void setFullTitle(const QString& title) {
    full_title_ = title;
    elideTitle();
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    const glass::ShellTokens shell = glass::Shell();
    // Flat slim strip — not a full-width rounded glass “card”, which looks
    // oversized and empty on wide panels (e.g. 3D View).
    painter.fillRect(rect(), shell.dock_title);
    painter.setPen(QPen(shell.glass_rim, 1.0));
    painter.drawLine(0, height() - 1, width(), height() - 1);
  }

  void resizeEvent(QResizeEvent* event) override {
    QWidget::resizeEvent(event);
    elideTitle();
  }

 private:
  void elideTitle() {
    if (title_label_ == nullptr) {
      return;
    }
    const int avail = title_label_->width();
    if (avail <= 0) {
      title_label_->setText(full_title_);
      return;
    }
    const QFontMetrics metrics(title_label_->font());
    title_label_->setText(
        metrics.elidedText(full_title_, Qt::ElideRight, avail));
  }

  QLabel* title_label_ = nullptr;
  QString full_title_;
};

}  // namespace

PanelDockWidget::PanelDockWidget(const QString& name, QWidget* parent)
    : QDockWidget(name, parent) {
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(style::sheet(QStringLiteral("chrome/dock")));
  title_bar_ = new DockGlassTitleBar(this);

  icon_label_ = new QLabel(title_bar_);
  // Keep icon flush with title-bar left inset so it lines up with inspector
  // content (Plot title, Title row, etc. use PanelSettingsLayout::kOuterMargin).
  icon_label_->setContentsMargins(0, 2, 0, 0);
  icon_label_->setVisible(false);
  // Labels must not steal mouse events — QDockWidget drag/snap (吸附) is
  // driven by the title-bar widget (eventFilter / ignored mouse events).
  icon_label_->setAttribute(Qt::WA_TransparentForMouseEvents);

  title_label_ = new QLabel(name, title_bar_);
  title_label_->setAttribute(Qt::WA_TransparentForMouseEvents);
  title_label_->setStyleSheet(DockTitleLabelStyle());
  title_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  title_label_->setMinimumWidth(0);
  if (auto* glass = dynamic_cast<DockGlassTitleBar*>(title_bar_)) {
    glass->setTitleLabel(title_label_);
    glass->setFullTitle(name);
  }

  auto* close_button = new QToolButton(title_bar_);
  close_button->setIcon(
      IconLoader::panelTitleIcon(QStringLiteral("panel.close")));
  close_button->setIconSize(QSize(16, 16));
  close_button->setAutoRaise(true);
  close_button->setFixedSize(QSize(24, 22));
  close_button->setToolTip(tr("Close"));
  close_button->setStyleSheet(
      style::sheet(QStringLiteral("chrome/dock_close_button")));
  connect(close_button, &QToolButton::clicked, this, &QDockWidget::close);

  auto* title_layout = new QHBoxLayout(title_bar_);
  // Same left inset as Property Inspector content (Plot / Title / …).
  title_layout->setContentsMargins(PanelSettingsLayout::kOuterMargin, 2, 4, 2);
  title_layout->setSpacing(2);
  title_layout->addWidget(icon_label_, 0);
  // Title yields width so tools + close stay visible on narrow docks.
  title_layout->addWidget(title_label_, 1);
  title_tools_host_ = new QWidget(title_bar_);
  title_tools_host_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
  title_layout->addWidget(title_tools_host_, 0);
  title_layout->addWidget(close_button, 0);
  setTitleBarWidget(title_bar_);
  title_bar_->setMouseTracking(true);
  title_bar_->installEventFilter(this);
}

void PanelDockWidget::setTitleBarTools(QWidget* tools) {
  if (title_tools_host_ == nullptr) {
    return;
  }
  if (QLayout* existing = title_tools_host_->layout()) {
    QLayoutItem* item = nullptr;
    while ((item = existing->takeAt(0)) != nullptr) {
      if (item->widget() != nullptr) {
        item->widget()->deleteLater();
      }
      delete item;
    }
    delete existing;
  }
  if (tools == nullptr) {
    title_tools_host_->setMinimumWidth(0);
    return;
  }
  tools->setParent(title_tools_host_);
  tools->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
  auto* layout = new QHBoxLayout(title_tools_host_);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  layout->addWidget(tools);
  title_tools_host_->adjustSize();
  const int tools_w = qMax(title_tools_host_->sizeHint().width(),
                           tools->sizeHint().width());
  title_tools_host_->setMinimumWidth(tools_w);
  // margins + icon + elided title slack + close
  constexpr int kChrome = 8 + 20 + 48 + 28 + 8;
  setMinimumWidth(qMax(minimumWidth(), tools_w + kChrome));
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
    ApplyPanelShell(child);
    applyFixedContentHeight();
    enforceFixedHeight();
  }
}

QMainWindow* PanelDockWidget::mainWindow() const {
  for (QWidget* widget = parentWidget(); widget != nullptr;
       widget = widget->parentWidget()) {
    if (auto* main_window = qobject_cast<QMainWindow*>(widget)) {
      return main_window;
    }
  }
  return nullptr;
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

void PanelDockWidget::setPanelTitle(const QString& title) {
  if (auto* glass = dynamic_cast<DockGlassTitleBar*>(title_bar_)) {
    glass->setFullTitle(title);
  } else if (title_label_ != nullptr) {
    title_label_->setText(title);
  }
  setWindowTitle(title);
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
  // Title-bar widgets ignore mouse events, so a double-click falls through to
  // QDockWidget and toggles floating. That swaps the embedded glass chrome for
  // a native top-level window (Displays / Views / Properties).
  if (watched == title_bar_ &&
      event->type() == QEvent::MouseButtonDblClick && !isFloating()) {
    return true;
  }

  // Let QDockWidget finish drag/drop (吸附) before we clear drag state —
  // ending early reparents mid-drop and can SIGSEGV / cancel snap.
  const bool qt_handled = QDockWidget::eventFilter(watched, event);

  if (watched == title_bar_) {
    switch (event->type()) {
      case QEvent::MouseButtonPress: {
        emit activated();
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (mouse->button() == Qt::LeftButton) {
          // Ignore presses on title-bar buttons / tools.
          if (QWidget* child = title_bar_->childAt(mouse->position().toPoint())) {
            if (qobject_cast<QToolButton*>(child) != nullptr ||
                child == title_tools_host_ ||
                (title_tools_host_ != nullptr &&
                 title_tools_host_->isAncestorOf(child))) {
              title_press_active_ = false;
              break;
            }
          }
          title_press_active_ = true;
          title_press_pos_ = mouse->position().toPoint();
        }
        break;
      }
      case QEvent::MouseMove: {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (!(mouse->buttons() & Qt::LeftButton)) {
          break;
        }
        // Nested MainPanelHost often fails to start Qt's built-in dock drag from
        // a custom title bar — undock under the cursor once the drag threshold
        // is crossed so the panel can be moved / snapped.
        // Only use the manual float+move path when the dock has no allowed areas
        // (center MainPanelHost). Flexible / sidebar docks keep AllowedAreas so
        // QDockWidget::eventFilter can drive native dock drag and 吸附.
        const bool use_manual_undock =
            allowedAreas() == Qt::NoDockWidgetArea;
        if (title_press_active_ && !title_drag_active_) {
          const QPoint delta = mouse->position().toPoint() - title_press_pos_;
          if (delta.manhattanLength() >= QApplication::startDragDistance()) {
            title_drag_active_ = true;
            emit titleDragStarted();
            if (use_manual_undock && !isFloating()) {
              // Center panels intentionally omit DockWidgetFloatable so the
              // QSplitter handle stays interactive — re-enable only for undock.
              setFeatures(features() | DockWidgetFloatable);
              const QPoint grab = title_press_pos_;
              setFloating(true);
              move(mouse->globalPosition().toPoint() - grab);
            }
          }
        } else if (title_drag_active_ && use_manual_undock && isFloating()) {
          move(mouse->globalPosition().toPoint() - title_press_pos_);
        }
        break;
      }
      case QEvent::MouseButtonRelease: {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (mouse->button() == Qt::LeftButton) {
          title_press_active_ = false;
          // Defer so Qt can finish docking/snap before retile / sync runs.
          QTimer::singleShot(0, this, &PanelDockWidget::endTitleDrag);
        }
        break;
      }
      default:
        break;
    }
  }
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
  return qt_handled;
}

void PanelDockWidget::changeEvent(QEvent* event) {
  // Do not end title drag on ParentChange/WindowStateChange — Qt reparents the
  // dock while dragging, and finishing early tears the dock down (SIGSEGV) and
  // cancels snap targets.
  QDockWidget::changeEvent(event);
}

void PanelDockWidget::endTitleDrag() {
  title_press_active_ = false;
  if (!title_drag_active_) {
    return;
  }
  title_drag_active_ = false;
  emit titleDragFinished();
}

void PanelDockWidget::closeEvent(QCloseEvent* event) {
  endTitleDrag();
  QDockWidget::closeEvent(event);
  emit closed();
}

void PanelDockWidget::onChildDestroyed(QObject* /*child*/) { deleteLater(); }

}  // namespace autoviz
