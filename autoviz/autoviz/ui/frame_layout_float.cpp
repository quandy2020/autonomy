/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_layout.hpp"
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/common/selection.hpp"
#include <algorithm>
#include <QApplication>
#include <QFrame>
#include <QGuiApplication>
#include <QScrollArea>
#include <QSet>
#include <QStackedWidget>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QDateTime>
#include <QDialog>
#include <QDir>
#include <QDockWidget>
#include <QMainWindow>
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QPainter>
#include <QPalette>
#include <QPen>
#include <QPointer>
#include <QResizeEvent>
#include <QScreen>
#include <QSettings>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSizeGrip>
#include <QStatusBar>
#include <QSizePolicy>
#include <QTimer>
#include <QCursor>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QVariant>
#include <QTabWidget>
#include <QToolBar>
#include <QToolButton>
#include <QGridLayout>
#include <QQuaternion>
#include <QWindow>
#include <QVBoxLayout>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/tool_manager.hpp"
#include "autoviz/common/interaction/message_queue.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/common/view_state_io.hpp"
#include "autoviz/ui/viewport_panel.hpp"

#include <functional>
#include <memory>
#include <optional>
#include "autoviz/ui/panel/add_dialog.hpp"
#include "autoviz/ui/app/settings_dialog.hpp"
#include "autoviz/ui/app/preferences.hpp"
#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/app/translation.hpp"
#include "autoviz/ui/panel/catalog.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/content_only_float.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/panel/title_tools.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/displays/panel.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/image/image_panel.hpp"
#include "autoviz/ui/panel_host.hpp"
#include "autoviz/ui/panel/role.hpp"
#include "autoviz/ui/inspector/property_panel.hpp"
#include "autoviz/ui/dialog/import_record.hpp"
#include "autoviz/ui/dialog/record_open.hpp"
#include "autoviz/ui/plot/io/plot_config_io.hpp"
#include "autoviz/ui/publish/publish_config_io.hpp"
#include "autoviz/ui/image/image_config_io.hpp"
#include "autoviz/ui/plot/panel/plot_panel.hpp"
#include "autoviz/ui/teleop/teleop_panel.hpp"
#include "autoviz/ui/navigation/navigation_panel.hpp"
#include "autoviz/ui/raw/panel.hpp"
#include "autoviz/ui/channels/channels_panel.hpp"
#include "autoviz/ui/inspector/selection_panel.hpp"
#include "autoviz/ui/inspector/tool_properties_panel.hpp"
#include "autoviz/ui/tf_tree/panel.hpp"
#include "autoviz/ui/publish/publish_panel.hpp"
#include "autoviz/ui/map/map_panel.hpp"
#include "autoviz/ui/channel_graph/channel_graph_panel.hpp"
#include "autoviz/ui/service/service_panel.hpp"
#include "autoviz/ui/time/panel.hpp"
#include "autoviz/ui/panel/name_map.hpp"
#include "autoviz/ui/viewport_hud.hpp"
#include "autoviz/ui/views/panel.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"


namespace autoviz {

namespace {

constexpr char kFloatShellProp[] = "floatShellWidget";
constexpr char kFloatShellGlSync[] = "floatShellGlSync";
constexpr char kFloatShellWrap[] = "intentionalFloatShellWrap";
constexpr int kFloatShellEdgePx = 6;

enum class FloatResizeEdge : quint8 {
  kNone = 0,
  kLeft = 1,
  kRight = 2,
  kTop = 4,
  kBottom = 8,
  kTopLeft = kTop | kLeft,
  kTopRight = kTop | kRight,
  kBottomLeft = kBottom | kLeft,
  kBottomRight = kBottom | kRight,
};

FloatResizeEdge HitTestFloatEdge(const QWidget* host, const QPoint& local) {
  if (host == nullptr) {
    return FloatResizeEdge::kNone;
  }
  const int w = host->width();
  const int h = host->height();
  quint8 edge = 0;
  if (local.x() <= kFloatShellEdgePx) {
    edge |= static_cast<quint8>(FloatResizeEdge::kLeft);
  }
  if (local.x() >= w - kFloatShellEdgePx) {
    edge |= static_cast<quint8>(FloatResizeEdge::kRight);
  }
  if (local.y() <= kFloatShellEdgePx) {
    edge |= static_cast<quint8>(FloatResizeEdge::kTop);
  }
  if (local.y() >= h - kFloatShellEdgePx) {
    edge |= static_cast<quint8>(FloatResizeEdge::kBottom);
  }
  return static_cast<FloatResizeEdge>(edge);
}

Qt::CursorShape CursorForFloatEdge(FloatResizeEdge edge) {
  switch (edge) {
    case FloatResizeEdge::kLeft:
    case FloatResizeEdge::kRight:
      return Qt::SizeHorCursor;
    case FloatResizeEdge::kTop:
    case FloatResizeEdge::kBottom:
      return Qt::SizeVerCursor;
    case FloatResizeEdge::kTopLeft:
    case FloatResizeEdge::kBottomRight:
      return Qt::SizeFDiagCursor;
    case FloatResizeEdge::kTopRight:
    case FloatResizeEdge::kBottomLeft:
      return Qt::SizeBDiagCursor;
    default:
      return Qt::ArrowCursor;
  }
}

Qt::Edges QtEdgesForFloatEdge(FloatResizeEdge edge) {
  Qt::Edges edges;
  const quint8 bits = static_cast<quint8>(edge);
  if (bits & static_cast<quint8>(FloatResizeEdge::kLeft)) {
    edges |= Qt::LeftEdge;
  }
  if (bits & static_cast<quint8>(FloatResizeEdge::kRight)) {
    edges |= Qt::RightEdge;
  }
  if (bits & static_cast<quint8>(FloatResizeEdge::kTop)) {
    edges |= Qt::TopEdge;
  }
  if (bits & static_cast<quint8>(FloatResizeEdge::kBottom)) {
    edges |= Qt::BottomEdge;
  }
  return edges;
}

/**
 * Keeps the external GLX/HWND surface in sync when the frameless float shell
 * is dragged/resized, and owns edge-resize for the shell (native GL covers a
 * dock-local QSizeGrip).
 */
class FloatShellGlSync : public QObject {
 public:
  explicit FloatShellGlSync(QWidget* shell, QWidget* wrap, PanelDockWidget* dock)
      : QObject(shell), shell_(shell), wrap_(wrap), dock_(dock) {
    setObjectName(QString::fromLatin1(kFloatShellGlSync));
    settle_.setSingleShot(true);
    settle_.setInterval(32);
    QObject::connect(&settle_, &QTimer::timeout, this, [this]() { endMove(); });
    if (shell_ != nullptr) {
      shell_->setMouseTracking(true);
      shell_->installEventFilter(this);
    }
    if (wrap_ != nullptr) {
      wrap_->setMouseTracking(true);
      wrap_->installEventFilter(this);
    }
    if (dock_ != nullptr) {
      dock_->setMouseTracking(true);
      dock_->installEventFilter(this);
    }
  }

  bool eventFilter(QObject* watched, QEvent* event) override {
    if (shell_ != nullptr &&
        (watched == shell_ || watched == wrap_ || watched == dock_) &&
        (event->type() == QEvent::Move || event->type() == QEvent::Resize)) {
      if (watched == shell_) {
        beginMove();
        settle_.start();
      }
    }

    if (shell_ == nullptr ||
        (watched != shell_ && watched != wrap_ && watched != dock_)) {
      return QObject::eventFilter(watched, event);
    }

    switch (event->type()) {
      case QEvent::MouseMove: {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (resize_edge_ != FloatResizeEdge::kNone) {
          applyManualResize(mouse->globalPosition().toPoint());
          return true;
        }
        const QPoint local =
            shell_->mapFromGlobal(mouse->globalPosition().toPoint());
        const FloatResizeEdge edge = HitTestFloatEdge(shell_, local);
        if (edge != FloatResizeEdge::kNone) {
          shell_->setCursor(CursorForFloatEdge(edge));
          if (wrap_ != nullptr) {
            wrap_->setCursor(CursorForFloatEdge(edge));
          }
        } else {
          shell_->unsetCursor();
          if (wrap_ != nullptr) {
            wrap_->unsetCursor();
          }
        }
        break;
      }
      case QEvent::MouseButtonPress: {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (mouse->button() != Qt::LeftButton) {
          break;
        }
        const QPoint local =
            shell_->mapFromGlobal(mouse->globalPosition().toPoint());
        const FloatResizeEdge edge = HitTestFloatEdge(shell_, local);
        if (edge == FloatResizeEdge::kNone) {
          break;
        }
        // Prefer compositor/system resize when the platform supports it.
        if (QWindow* win = shell_->windowHandle()) {
          if (win->startSystemResize(QtEdgesForFloatEdge(edge))) {
            return true;
          }
        }
        resize_edge_ = edge;
        resize_origin_global_ = mouse->globalPosition().toPoint();
        resize_origin_geom_ = shell_->geometry();
        beginMove();
        return true;
      }
      case QEvent::MouseButtonRelease: {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (mouse->button() == Qt::LeftButton &&
            resize_edge_ != FloatResizeEdge::kNone) {
          resize_edge_ = FloatResizeEdge::kNone;
          settle_.start();
          return true;
        }
        break;
      }
      case QEvent::Leave:
        if (resize_edge_ == FloatResizeEdge::kNone) {
          shell_->unsetCursor();
          if (wrap_ != nullptr) {
            wrap_->unsetCursor();
          }
        }
        break;
      default:
        break;
    }
    return QObject::eventFilter(watched, event);
  }

 private:
  void forEachOgre(const std::function<void(rendering::OgreRenderWindow*)>& fn) {
    if (dock_ == nullptr || !fn) {
      return;
    }
    const auto windows = dock_->findChildren<rendering::OgreRenderWindow*>();
    for (rendering::OgreRenderWindow* window : windows) {
      if (window != nullptr) {
        fn(window);
      }
    }
  }

  void beginMove() {
    if (moving_) {
      return;
    }
    moving_ = true;
    forEachOgre([](rendering::OgreRenderWindow* window) {
      window->hideNativeSurface();
    });
  }

  void endMove() {
    if (!moving_) {
      return;
    }
    moving_ = false;
    forEachOgre([](rendering::OgreRenderWindow* window) {
      window->showNativeSurface();
      window->notifyParentWindowMoved();
      window->update();
    });
  }

  void applyManualResize(const QPoint& global) {
    if (shell_ == nullptr) {
      return;
    }
    QRect geom = resize_origin_geom_;
    const QPoint delta = global - resize_origin_global_;
    const quint8 edge = static_cast<quint8>(resize_edge_);
    if (edge & static_cast<quint8>(FloatResizeEdge::kLeft)) {
      geom.setLeft(resize_origin_geom_.left() + delta.x());
    }
    if (edge & static_cast<quint8>(FloatResizeEdge::kRight)) {
      geom.setRight(resize_origin_geom_.right() + delta.x());
    }
    if (edge & static_cast<quint8>(FloatResizeEdge::kTop)) {
      geom.setTop(resize_origin_geom_.top() + delta.y());
    }
    if (edge & static_cast<quint8>(FloatResizeEdge::kBottom)) {
      geom.setBottom(resize_origin_geom_.bottom() + delta.y());
    }
    constexpr int kMinW = 320;
    constexpr int kMinH = 240;
    if (geom.width() < kMinW) {
      if (edge & static_cast<quint8>(FloatResizeEdge::kLeft)) {
        geom.setLeft(geom.right() - kMinW + 1);
      } else {
        geom.setWidth(kMinW);
      }
    }
    if (geom.height() < kMinH) {
      if (edge & static_cast<quint8>(FloatResizeEdge::kTop)) {
        geom.setTop(geom.bottom() - kMinH + 1);
      } else {
        geom.setHeight(kMinH);
      }
    }
    shell_->setGeometry(geom);
  }

  QPointer<QWidget> shell_;
  QPointer<QWidget> wrap_;
  QPointer<PanelDockWidget> dock_;
  QTimer settle_;
  bool moving_ = false;
  FloatResizeEdge resize_edge_ = FloatResizeEdge::kNone;
  QPoint resize_origin_global_;
  QRect resize_origin_geom_;
};

QWidget* FloatShellWrapFor(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return nullptr;
  }
  QWidget* shell = dock->property(kFloatShellProp).value<QWidget*>();
  if (shell == nullptr) {
    return nullptr;
  }
  if (auto* mw = qobject_cast<QMainWindow*>(shell)) {
    if (QWidget* central = mw->centralWidget()) {
      if (central->objectName() == QLatin1String(kFloatShellWrap)) {
        return central;
      }
    }
  }
  return nullptr;
}

}  // namespace

void FrameLayout::attachIntentionalFloatShell(PanelDockWidget* dock,
                                              const QRect& geom) {
  if (dock == nullptr) {
    return;
  }
  detachIntentionalFloatShell(dock);

  // Parent to the frame as a Tool window so the shell stays above Autoviz
  // panels without mosaic 吸附 and without desktop-wide Always-on-Top.
  auto* shell = new QMainWindow(frame_);
  shell->setObjectName(QStringLiteral("intentionalFloatShell_%1")
                           .arg(dock->objectName()));
  const QString title =
      dock->windowTitle().isEmpty() ? dock->objectName() : dock->windowTitle();
  shell->setWindowTitle(title);
  shell->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
  shell->setAttribute(Qt::WA_DeleteOnClose, false);
  shell->setAttribute(Qt::WA_OpaquePaintEvent, true);
  shell->setAttribute(Qt::WA_TranslucentBackground, false);
  shell->setAutoFillBackground(true);
  {
    QPalette palette = shell->palette();
    palette.setColor(QPalette::Window, QColor(18, 18, 22));
    shell->setPalette(palette);
  }
  shell->setMinimumSize(320, 240);

  {
    const QSignalBlocker blocker(dock);
    if (dock->isFloating()) {
      dock->setFloating(false);
    }
    dock->setFeatures(QDockWidget::DockWidgetClosable);
    dock->setAllowedAreas(Qt::NoDockWidgetArea);
    dock->setParent(nullptr);
  }

  // Thin Qt margins around the dock so edge-resize / size grip stay outside
  // the native Ogre surface (which otherwise eats all corner hits).
  auto* wrap = new QWidget();
  wrap->setObjectName(QString::fromLatin1(kFloatShellWrap));
  wrap->setAutoFillBackground(true);
  {
    QPalette palette = wrap->palette();
    palette.setColor(QPalette::Window, QColor(18, 18, 22));
    wrap->setPalette(palette);
  }
  auto* lay = new QVBoxLayout(wrap);
  // Right/bottom leave room for QSizeGrip outside the native GL surface.
  lay->setContentsMargins(kFloatShellEdgePx, 0, 16, 16);
  lay->setSpacing(0);
  lay->addWidget(dock);
  shell->setCentralWidget(wrap);
  dock->setProperty(kFloatShellProp, QVariant::fromValue(static_cast<QWidget*>(shell)));
  dock->show();

  // Parent to shell so it dies with the shell; syncs GLX + edge resize.
  new FloatShellGlSync(shell, wrap, dock);

  QRect placed = geom;
  if (!placed.isValid() || placed.width() < 64 || placed.height() < 64) {
    const QRect area = frame_ != nullptr ? frame_->geometry() : QRect(0, 0, 1280, 800);
    placed = QRect(QPoint(0, 0),
                   QSize(std::max(area.width() * 2 / 3, 640),
                         std::max(area.height() * 2 / 3, 480)));
    placed.moveCenter(area.center());
  }
  shell->setGeometry(placed);
  shell->show();
  shell->raise();
  shell->activateWindow();
  (void)shell->winId();

  // Initial map: force GLX to the shell's screen position.
  const auto windows = dock->findChildren<rendering::OgreRenderWindow*>();
  for (rendering::OgreRenderWindow* window : windows) {
    if (window == nullptr) {
      continue;
    }
    window->showNativeSurface();
    window->notifyParentWindowMoved();
    window->update();
  }
}

void FrameLayout::detachIntentionalFloatShell(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  QWidget* shell = dock->property(kFloatShellProp).value<QWidget*>();
  dock->setProperty(kFloatShellProp, QVariant());
  if (shell == nullptr) {
    return;
  }
  if (auto* mw = qobject_cast<QMainWindow*>(shell)) {
    QWidget* central = mw->takeCentralWidget();
    if (central == dock) {
      // Legacy path: dock was the central widget directly.
    } else if (central != nullptr) {
      if (dock->parentWidget() == central ||
          (central != nullptr && central->isAncestorOf(dock))) {
        const QSignalBlocker blocker(dock);
        dock->setParent(nullptr);
      }
      central->deleteLater();
    }
  } else if (dock->parentWidget() == shell) {
    dock->setParent(nullptr);
  }
  shell->hide();
  shell->deleteLater();
}

bool FrameLayout::tryRedockIntentionalFloat(PanelDockWidget* dock,
                                            const QPoint& global_pos) {
  if (dock == nullptr || main_panel_host_ == nullptr || !isMainPanel(dock) ||
      !dock->property("intentionalFloat").toBool()) {
    return false;
  }

  QDockWidget* target = nullptr;
  Qt::Edge edge = Qt::LeftEdge;
  const bool has_edge =
      main_panel_host_->hitTestDrop(global_pos, dock, &target, &edge);
  const bool over_host = main_panel_host_->rect().contains(
      main_panel_host_->mapFromGlobal(global_pos));
  if (!has_edge && !over_host) {
    main_panel_host_->setDropPreview(global_pos, dock, false);
    return false;
  }

  // Capture shell geometry before teardown in case redock fails.
  QRect shell_geom;
  if (QWidget* shell = dock->property(kFloatShellProp).value<QWidget*>()) {
    shell_geom = shell->geometry();
  }

  if (IsContentOnlyFloat(dock)) {
    SetContentOnlyFloatChrome(dock, false);
    if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
      image->setContentOnlyMode(false);
    }
  }
  setIntentionalFloatChrome(dock, false);
  detachIntentionalFloatShell(dock);
  {
    const QSignalBlocker blocker(dock);
    dock->setFeatures(dock->features() | QDockWidget::DockWidgetFloatable);
    dock->setFloating(true);
  }
  dock->show();

  const bool accepted = finishMainPanelDrag(dock, global_pos);
  if (!accepted) {
    // Cursor was over host but drop failed — restore free-float shell.
    if (!shell_geom.isValid()) {
      shell_geom = QRect(dock->mapToGlobal(QPoint(0, 0)), dock->size());
    }
    attachIntentionalFloatShell(dock, shell_geom);
    setIntentionalFloatChrome(dock, true);
  }
  return accepted;
}

void FrameLayout::setIntentionalFloatChrome(PanelDockWidget* dock,
                                            bool enabled) {
  if (dock == nullptr) {
    return;
  }
  dock->setProperty("intentionalFloat", enabled);

  static const QString kGripName = QStringLiteral("intentionalFloatGrip");
  static const QString kPlacerName =
      QStringLiteral("intentionalFloatGripPlacer");

  auto clear_grip_on = [](QWidget* host) {
    if (host == nullptr) {
      return;
    }
    if (auto* existing = host->findChild<QSizeGrip*>(kGripName,
                                                    Qt::FindDirectChildrenOnly)) {
      existing->hide();
      existing->deleteLater();
    }
    if (auto* placer =
            host->findChild<QObject*>(kPlacerName, Qt::FindDirectChildrenOnly)) {
      host->removeEventFilter(placer);
      placer->deleteLater();
    }
  };
  clear_grip_on(dock);
  clear_grip_on(FloatShellWrapFor(dock));

  if (!enabled) {
    dock->setMinimumSize(0, 0);
    dock->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    return;
  }

  // Shell-hosted floats must not be Floatable (would fight the outer window).
  // Standalone free-float keeps Floatable so Qt does not force-redock.
  QWidget* shell = dock->property(kFloatShellProp).value<QWidget*>();
  const bool in_shell = shell != nullptr;
  dock->setFeatures(in_shell ? QDockWidget::DockWidgetClosable
                             : (QDockWidget::DockWidgetClosable |
                                QDockWidget::DockWidgetFloatable));
  dock->setAllowedAreas(Qt::NoDockWidgetArea);
  dock->setMinimumSize(320, 240);
  dock->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
  if (QWidget* content = dock->widget()) {
    content->setMinimumSize(0, 0);
    content->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    content->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  // Prefer shell wrap as grip host so the handle sits in the Qt margin outside
  // the native GL surface.
  QWidget* grip_host = FloatShellWrapFor(dock);
  if (grip_host == nullptr) {
    grip_host = dock;
  }
  auto* grip = new QSizeGrip(grip_host);
  grip->setObjectName(kGripName);
  grip->setFixedSize(16, 16);
  grip->show();
  grip->raise();

  class GripPlacer : public QObject {
   public:
    GripPlacer(QWidget* host, QSizeGrip* grip_widget)
        : QObject(host), host_(host), grip_(grip_widget) {
      setObjectName(QStringLiteral("intentionalFloatGripPlacer"));
      place();
    }

    bool eventFilter(QObject* watched, QEvent* event) override {
      if (watched == host_ &&
          (event->type() == QEvent::Resize || event->type() == QEvent::Show)) {
        place();
      }
      return QObject::eventFilter(watched, event);
    }

   private:
    void place() {
      if (host_ == nullptr || grip_ == nullptr) {
        return;
      }
      grip_->move(host_->width() - grip_->width() - 1,
                  host_->height() - grip_->height() - 1);
      grip_->raise();
    }

    QWidget* host_ = nullptr;
    QSizeGrip* grip_ = nullptr;
  };

  auto* placer = new GripPlacer(grip_host, grip);
  grip_host->installEventFilter(placer);
}

void FrameLayout::setContentOnlyFloat(PanelDockWidget* dock, bool enabled,
                                      const QString& title) {
  if (dock == nullptr || !dock->property("intentionalFloat").toBool()) {
    return;
  }
  const bool is_viewport =
      frame_->panels_->panelTypeId(dock) == QLatin1String("ViewportDock");

  // Give the hover bar a Qt strip above native GL when hosted in the shell wrap.
  if (QWidget* wrap = FloatShellWrapFor(dock)) {
    if (auto* lay = qobject_cast<QVBoxLayout*>(wrap->layout())) {
      if (enabled) {
        lay->setContentsMargins(kFloatShellEdgePx, 32, 16, 16);
      } else {
        lay->setContentsMargins(kFloatShellEdgePx, 0, 16, 16);
      }
    }
  }

  if (!enabled) {
    SetContentOnlyFloatChrome(dock, false);
    // Restore size grip / features for normal free-float.
    setIntentionalFloatChrome(dock, true);
    if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
      image->setContentOnlyMode(false);
    }
    if (is_viewport) {
      frame_->viewport_->setViewportContentOnlyButtonChecked(dock, false);
    }
    return;
  }

  const QString default_title =
      is_viewport ? QStringLiteral("3D View") : QStringLiteral("Image");
  const QString bar_title =
      title.isEmpty()
          ? (dock->windowTitle().isEmpty() ? default_title : dock->windowTitle())
          : title;
  const QPointer<PanelDockWidget> dock_guard(dock);
  SetContentOnlyFloatChrome(
      dock, true,
      [this, dock_guard]() {
        if (dock_guard) {
          setContentOnlyFloat(dock_guard.data(), false);
        }
      },
      [this, dock_guard]() {
        if (!dock_guard) {
          return;
        }
        setContentOnlyFloat(dock_guard.data(), false);
        toggleFloatMainPanel(dock_guard.data());
      },
      bar_title);
  if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
    image->setContentOnlyMode(true);
  }
  if (is_viewport) {
    frame_->viewport_->setViewportContentOnlyButtonChecked(dock, true);
  }
}

void FrameLayout::toggleContentOnlyFloat(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  setContentOnlyFloat(dock, !IsContentOnlyFloat(dock));
}

void FrameLayout::toggleFloatMainPanel(PanelDockWidget* dock) {
  if (dock == nullptr || main_panel_host_ == nullptr || !isMainPanel(dock)) {
    return;
  }
  last_active_dock_ = dock;

  if (dock->isFloating() || dock->property("intentionalFloat").toBool()) {
    const QString key = dock->objectName();
    const QByteArray mosaic = pre_float_mosaic_layout_.take(key);
    const bool was_suppressing = suppress_center_tile_;
    suppress_center_tile_ = true;

    if (IsContentOnlyFloat(dock)) {
      // Restores title chrome, viewport toolbar, and content-only button.
      setContentOnlyFloat(dock, false);
    }
    setIntentionalFloatChrome(dock, false);
    detachIntentionalFloatShell(dock);
    {
      const QSignalBlocker blocker(dock);
      if (dock->isFloating()) {
        dock->setFloating(false);
      }
      if (frame_->dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
        frame_->removeDockWidget(dock);
      }
    }

    const bool is_viewport_redock =
        frame_->panels_->panelTypeId(dock) == QLatin1String("ViewportDock");
    bool restored_mosaic = false;
    if (!mosaic.isEmpty()) {
      center_manual_layout_ = true;
      if (is_viewport_redock) {
        frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
          frame_->viewport_->parkRenderWindowInEntry(entry);
        });
      }
      restored_mosaic = main_panel_host_->restoreMosaicLayout(
          mosaic, [this](const QString& object_name) -> QDockWidget* {
            for (PanelDockWidget* candidate : orderedDockWidgets()) {
              if (candidate != nullptr && candidate->objectName() == object_name) {
                return candidate;
              }
            }
            return frame_->findChild<PanelDockWidget*>(object_name);
          });
      if (is_viewport_redock) {
        frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
          frame_->viewport_->reinstallRenderWindowInEntry(entry);
        });
      }
    }

    if (!restored_mosaic) {
      configureMainPanelDock(dock);
      if (!main_panel_host_->hostsPanel(dock)) {
        main_panel_host_->addPanel(dock);
      }
      dock->show();
      main_panel_host_->scrubEmptyPanes();
      syncCenterLayout();
      if (is_viewport_redock) {
        frame_->viewport_->ensureViewportPanelReady(dock);
        frame_->viewport_->setActiveViewportDock(dock);
        frame_->viewport_->requestViewportUpdate();
      }
    } else {
      configureMainPanelDock(dock);
      main_panel_host_->scrubEmptyPanes();
      main_panel_host_->syncHorizontalDockLayout();
    }

    suppress_center_tile_ = was_suppressing;
    dock->raise();
    if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
      image->setFloatButtonChecked(false);
    }
    if (is_viewport_redock) {
      frame_->viewport_->setViewportFloatButtonChecked(dock, false);
    }
    return;
  }

  // Floating while mosaic-expanded would lose sibling restore state.
  if (expanded_main_panel_dock_ != nullptr) {
    restoreExpandedMainPanel();
  }

  pre_float_mosaic_layout_.insert(dock->objectName(),
                                  main_panel_host_->saveMosaicLayout());

  const QPoint top_left = dock->mapToGlobal(QPoint(0, 0));
  const QSize size = dock->size();

  const bool is_viewport =
      frame_->panels_->panelTypeId(dock) == QLatin1String("ViewportDock");

  // Mark free-float BEFORE removePanel so visibilityChanged / tileCenterPanels
  // cannot re-attach or treat the undock as a Panels-menu hide.
  const bool was_manual = center_manual_layout_;
  const bool was_suppressing = suppress_center_tile_;
  center_manual_layout_ = true;
  suppress_center_tile_ = true;
  dock->setProperty("intentionalFloat", true);
  dock->setProperty("floatTransition", true);

  // Keep Ogre/GLX inside the dock. Host the dock in a real top-level
  // QMainWindow shell — QDockWidget::setFloating on mosaic-only panels never
  // maps a visible window.
  if (main_panel_host_->hostsPanel(dock)) {
    main_panel_host_->removePanel(dock);
  }
  main_panel_host_->scrubEmptyPanes();

  QRect floated_geom(top_left,
                     QSize(std::max(size.width(), 480),
                           std::max(size.height(), 360)));
  attachIntentionalFloatShell(dock, floated_geom);
  setIntentionalFloatChrome(dock, true);
  dock->setProperty("floatTransition", false);

  if (is_viewport) {
    const QPointer<PanelDockWidget> dock_guard(dock);
    auto refresh_floated = [this, dock_guard]() {
      if (!dock_guard || !dock_guard->property("intentionalFloat").toBool()) {
        return;
      }
      if (QWidget* shell = dock_guard->property("floatShellWidget").value<QWidget*>()) {
        shell->show();
        shell->raise();
        shell->activateWindow();
      }
      dock_guard->show();
      frame_->viewport_->ensureViewportPanelReady(dock_guard.data());
      frame_->viewport_->setActiveViewportDock(dock_guard.data());
      frame_->viewport_->requestViewportUpdate();
    };
    QTimer::singleShot(0, frame_, refresh_floated);
  }

  syncCenterLayout();
  suppress_center_tile_ = was_suppressing;
  center_manual_layout_ = was_manual;

  if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
    image->setFloatButtonChecked(true);
    if (image->config().content_only_display) {
      // Defer past setFloating/show/title-bar settle — swapping the title bar
      // widget in the same stack as undock has SIGSEGV'd under Qt dock layout.
      const QPointer<PanelDockWidget> dock_guard(dock);
      const QString title = image->config().title.trimmed().isEmpty()
                                ? image->config().image_channel
                                : image->config().title.trimmed();
      QTimer::singleShot(0, frame_, [this, dock_guard, title]() {
        if (!dock_guard || !dock_guard->property("intentionalFloat").toBool()) {
          return;
        }
        setContentOnlyFloat(dock_guard.data(), true, title);
      });
    }
  }
  if (is_viewport) {
    frame_->viewport_->setViewportFloatButtonChecked(dock, true);
  }
}

void FrameLayout::expandPanelDock(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  last_active_dock_ = dock;

  if (!isMainPanel(dock) || main_panel_host_ == nullptr) {
    if (!dock->isFloating()) {
      dock->setFloating(true);
      const QRect area = frame_->geometry();
      dock->resize(area.width() * 3 / 4, area.height() * 3 / 4);
      dock->move(area.center() - dock->rect().center());
    } else {
      dock->showMaximized();
    }
    dock->raise();
    return;
  }

  if (expanded_main_panel_dock_ == dock) {
    restoreExpandedMainPanel();
    dock->raise();
    return;
  }

  if (expanded_main_panel_dock_ != nullptr) {
    restoreExpandedMainPanel();
  }

  // Prefer requestedVisible() over isVisible(): during prior expand/restore
  // churn a dock may be briefly invisible while still part of the layout set.
  pre_expand_visible_main_panel_docks_.clear();
  for (PanelDockWidget* candidate : orderedDockWidgets()) {
    if (candidate != nullptr && isMainPanel(candidate) &&
        !candidate->property("panelDisposed").toBool() &&
        !candidate->isFloating() && candidate->requestedVisible()) {
      pre_expand_visible_main_panel_docks_.push_back(candidate);
    }
  }
  // Snapshot mosaic while all leaves are still hosted (exact restore later).
  pre_expand_mosaic_layout_ = main_panel_host_->saveMosaicLayout();

  // Suppress visibilityChanged side-effects while hiding siblings so they are
  // not permanently detached via removePanel before tileCenterPanels runs.
  const bool was_suppressing = suppress_center_tile_;
  suppress_center_tile_ = true;
  expanded_main_panel_dock_ = dock;
  ensureMainPanelDockAttached(dock);
  for (PanelDockWidget* candidate : orderedDockWidgets()) {
    if (candidate == nullptr || candidate == dock || !isMainPanel(candidate) ||
        candidate->isFloating()) {
      continue;
    }
    candidate->hide();
  }
  dock->show();
  dock->raise();
  suppress_center_tile_ = was_suppressing;
  syncMainPanelExpandUi(dock);
  tileCenterPanels();
}

void FrameLayout::syncMainPanelExpandUi(PanelDockWidget* expanded_dock) {
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr || !isMainPanel(dock)) {
      continue;
    }
    if (auto* plot = qobject_cast<plot::PlotPanel*>(dock->widget())) {
      plot->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
      image->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* tf = qobject_cast<TfTreePanel*>(dock->widget())) {
      tf->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* publish = qobject_cast<publish_panel::PublishPanel*>(dock->widget())) {
      publish->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* map_panel = qobject_cast<map::MapPanel*>(dock->widget())) {
      map_panel->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* channel_graph =
            qobject_cast<channel_graph::ChannelGraphPanel*>(dock->widget())) {
      channel_graph->setExpandButtonChecked(dock == expanded_dock);
    }
  }
  frame_->viewport_->forEachViewportPanel([expanded_dock](ViewportPanelEntry& entry) {
    if (entry.expand_button == nullptr) {
      return;
    }
    entry.expand_button->blockSignals(true);
    entry.expand_button->setChecked(entry.dock == expanded_dock);
    entry.expand_button->blockSignals(false);
  });
}

void FrameLayout::wireMainPanelExpandTracking(PanelDockWidget* dock) {
  if (dock == nullptr || !isMainPanel(dock) ||
      dock->property("mainPanelExpandTracked").toBool()) {
    return;
  }
  dock->setProperty("mainPanelExpandTracked", true);
  QObject::connect(dock, &PanelDockWidget::closed, frame_, [this, dock]() {
    if (expanded_main_panel_dock_ != dock) {
      return;
    }
    // Defer out of closeEvent — restoreExpandedMainPanel may tileCenterPanels
    // and park/reinstall Ogre while the closing dock is still mid-teardown.
    const QPointer<PanelDockWidget> guard(dock);
    QTimer::singleShot(0, frame_, [this, guard]() {
      if (guard && expanded_main_panel_dock_ == guard.data()) {
        restoreExpandedMainPanel();
      }
    });
  });
}


}  // namespace autoviz
