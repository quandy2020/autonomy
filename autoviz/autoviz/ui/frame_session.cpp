/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_session.hpp"
#include "autoviz/ui/frame_session_internal.hpp"
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/common/selection.hpp"
#include <algorithm>
#include <cstdio>
#include <unordered_map>
#include <unordered_set>
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
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QElapsedTimer>
#include <QEvent>
#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QIODevice>
#include <QTextStream>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QUrl>
#include <QPainter>
#include <QPen>
#include <QPointer>
#include <QResizeEvent>
#include <QScreen>
#include <QSettings>
#include <QWindow>
#include <QShortcut>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QSize>
#include <QSizePolicy>
#include <QTimer>
#include <QCursor>
#include <QVariant>
#include <QTabWidget>
#include <QToolBar>
#include <QToolButton>
#include <QGridLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QQuaternion>
#include <QVBoxLayout>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/tool_manager.hpp"
#include "autoviz/common/interaction/message_queue.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/render_system.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/common/view_state_io.hpp"
#include "autoviz/ui/panel/add_dialog.hpp"
#include "autoviz/ui/app/settings_dialog.hpp"
#include "autoviz/ui/app/preferences.hpp"
#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/app/translation.hpp"
#include "autoviz/ui/panel/catalog.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
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
#include "autoviz/ui/record/record_panel.hpp"
#include "autoviz/ui/evidence/evidence_panel.hpp"
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
#include "autoviz/ui/record_drop_overlay.hpp"
#include "autoviz/ui/theme/tokens.hpp"

#include <QAbstractAnimation>
#include <QEasingCurve>
#include <QGraphicsOpacityEffect>
#include <QPropertyAnimation>

namespace autoviz {

using frame_session_detail::kForcePaintAfterOccludedTicks;
using frame_session_detail::CollectMosaicLeafNames;
using frame_session_detail::IsViewportDockObjectName;
using frame_session_detail::SanitizedIdleDivisor;

namespace {

/** Opaque full-frame plate — hides dock/GL churn during Open Config. */
class LayoutTransitionCover : public QWidget {
 public:
  explicit LayoutTransitionCover(QWidget* parent) : QWidget(parent) {
    setObjectName(QStringLiteral("AutovizLayoutTransitionCover"));
    setAttribute(Qt::WA_TransparentForMouseEvents, false);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setAttribute(Qt::WA_NoSystemBackground, false);
    setAutoFillBackground(false);
    effect_ = new QGraphicsOpacityEffect(this);
    effect_->setOpacity(1.0);
    setGraphicsEffect(effect_);
    hide();
  }

  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.fillRect(rect(), theme::Shell().panel_surface);
  }

  QGraphicsOpacityEffect* effect() const { return effect_; }

 private:
  QGraphicsOpacityEffect* effect_ = nullptr;
};

}  // namespace

FrameSession::FrameSession(VisualizationFrame* frame)
    : frame_(frame),
      idle_render_divisor_(
          SanitizedIdleDivisor(LoadAppUiPreferences().idle_render_divisor)) {}

void FrameSession::updateWindowTitle() {
  QString title = QStringLiteral("Autoviz");
  if (!config_path_.isEmpty()) {
    title += QStringLiteral(" - %1").arg(QFileInfo(config_path_).fileName());
  }
  if (config_modified_) {
    title += QLatin1Char('*');
  }
  frame_->setWindowTitle(title);
}

void FrameSession::markConfigModified() {
  if (suppress_config_modified_ || config_modified_) {
    return;
  }
  config_modified_ = true;
  updateWindowTitle();
}

void FrameSession::clearConfigModified() {
  config_modified_ = false;
  updateWindowTitle();
}

void FrameSession::connectConfigModifiedSignals() {
  QObject::connect(frame_->panels_->displays_panel_, &DisplaysPanel::displaysChanged, frame_, &VisualizationFrame::markConfigModified);
  QObject::connect(frame_->panels_->displays_panel_, &DisplaysPanel::fixedFrameChanged, frame_, &VisualizationFrame::markConfigModified);
  QObject::connect(frame_->panels_->displays_panel_, &DisplaysPanel::backgroundColorChanged, frame_, &VisualizationFrame::markConfigModified);

  if (frame_->panels_->views_panel_ != nullptr) {
    QObject::connect(frame_->panels_->views_panel_, &ViewsPanel::viewsChanged, frame_, &VisualizationFrame::markConfigModified);
    QObject::connect(frame_->panels_->views_panel_, &ViewsPanel::viewChanged, frame_, &VisualizationFrame::markConfigModified);
  }
  if (frame_->panels_->tool_properties_panel_ != nullptr) {
    QObject::connect(frame_->panels_->tool_properties_panel_, &ToolPropertiesPanel::propertiesChanged, frame_, &VisualizationFrame::markConfigModified);
  }

  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    QObject::connect(dock, &QDockWidget::dockLocationChanged, frame_, &VisualizationFrame::markConfigModified);
    QObject::connect(dock, &QDockWidget::topLevelChanged, frame_, &VisualizationFrame::markConfigModified);
    QObject::connect(dock, &PanelDockWidget::closed, frame_, &VisualizationFrame::markConfigModified);
  }
}

void FrameSession::updateSelectionPanel(
    const std::vector<common::SelectionEntry>& entries) {
  if (frame_->panels_->selection_panel_ != nullptr) {
    frame_->panels_->selection_panel_->setSelections(entries);
  }
}

void FrameSession::onScreenshot() {
  ViewportPanelEntry* entry = frame_->viewport_->activeViewportEntry();
  if (entry == nullptr || entry->widget == nullptr) {
    return;
  }
  const QPixmap pixmap = entry->widget->grab();
  const QString path = QFileDialog::getSaveFileName(
      frame_, frame_->tr("Save Image"),
      QStringLiteral("autoviz_%1.png")
          .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_hhmmss"))),
      frame_->tr("PNG Image (*.png)"));
  if (!path.isEmpty()) {
    pixmap.save(path);
  }
}

void FrameSession::setFullScreen(bool full_screen) {
  const auto state = frame_->windowState();
  if (full_screen == state.testFlag(Qt::WindowFullScreen)) {
    return;
  }
  if (frame_->chrome_->fullscreen_action_ != nullptr) {
    frame_->chrome_->fullscreen_action_->setChecked(full_screen);
  }
  if (full_screen) {
    frame_->chrome_->toolbar_visible_ = frame_->chrome_->tool_bar_ == nullptr || frame_->chrome_->tool_bar_->isVisible();
  }
  emit frame_->fullScreenChange(full_screen);
  frame_->menuBar()->setVisible(!full_screen);
  if (frame_->chrome_->tool_bar_ != nullptr) {
    frame_->chrome_->tool_bar_->setVisible(!full_screen && frame_->chrome_->toolbar_visible_);
  }
  frame_->statusBar()->setVisible(!full_screen);
  if (full_screen) {
    frame_->setWindowState(state | Qt::WindowFullScreen);
  } else {
    frame_->setWindowState(state & ~Qt::WindowFullScreen);
  }
  frame_->show();
}

void FrameSession::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape && frame_->isFullScreen()) {
    setFullScreen(false);
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_R && !event->modifiers().testFlag(Qt::ControlModifier) &&
      !event->modifiers().testFlag(Qt::AltModifier)) {
    frame_->onReset();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Z && frame_->panels_->views_panel_ != nullptr &&
      !event->modifiers().testFlag(Qt::ControlModifier) &&
      !event->modifiers().testFlag(Qt::AltModifier)) {
    frame_->panels_->views_panel_->zeroView();
    frame_->viewport_->requestViewportUpdate();
    event->accept();
    return;
  }
  if (frame_->manager_->tools().forwardActiveToolKeyPress(event)) {
    event->accept();
    return;
  }
  if (!event->modifiers()) {
    if (frame_->manager_->tools().handleShortcutKey(event->key())) {
      frame_->chrome_->syncActiveToolUi();
      event->accept();
      return;
    }
  }
}

void FrameSession::activateRestoredViewportAndView() {
  PanelDockWidget* visible_viewport = nullptr;
  QList<PanelDockWidget*> visible_viewports;
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || !dock->requestedVisible() ||
        frame_->panels_->panelTypeId(dock) != QLatin1String("ViewportDock")) {
      continue;
    }
    visible_viewports.push_back(dock);
    if (visible_viewport == nullptr) {
      visible_viewport = dock;
    }
  }

  const bool host_has_viewport = [&]() {
    if (frame_->layout_->main_panel_host_ == nullptr) {
      return false;
    }
    for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
      if (dock == nullptr ||
          frame_->panels_->panelTypeId(dock) !=
              QLatin1String("ViewportDock")) {
        continue;
      }
      if (frame_->layout_->main_panel_host_->hostsPanel(dock)) {
        return true;
      }
    }
    return false;
  }();

  // Center column must host a 3D View by default. Cover: closed-all, mosaic
  // that parked ViewportDock, or restore that left only Image/Plot in the host.
  if ((visible_viewport == nullptr || !host_has_viewport) &&
      frame_->viewport_->viewport_dock_ != nullptr) {
    PanelDockWidget* primary = frame_->viewport_->viewport_dock_;
    frame_->layout_->center_manual_layout_ = false;
    frame_->layout_->suppress_center_tile_ = false;
    frame_->layout_->ensureMainPanelDockAttached(primary);
    primary->show();
    if (QAction* toggle = primary->toggleViewAction()) {
      toggle->blockSignals(true);
      toggle->setChecked(true);
      toggle->blockSignals(false);
    }
    frame_->layout_->last_active_dock_ = primary;
    if (!host_has_viewport) {
      // Empty / Image-only center → retile so ViewportDock is in the mosaic.
      frame_->layout_->tileCenterPanels();
    } else {
      frame_->layout_->scheduleTileCenterPanels();
    }
    visible_viewport = primary;
    if (!visible_viewports.contains(primary)) {
      visible_viewports.push_back(primary);
    }
  }
  // Dual-pane sessions create ViewportDock_2 with create_render_window=false.
  // Ready EVERY visible 3D View — ensuring only the first left the sibling
  // without GL (Qt-gray hole) while the primary GLX often covered the right.
  for (PanelDockWidget* dock : visible_viewports) {
    frame_->viewport_->ensureViewportPanelReady(dock);
  }
  if (visible_viewport != nullptr) {
    frame_->viewport_->setActiveViewportDock(visible_viewport);
  }
  applyCurrentView();
}

bool FrameSession::startupCloseAllowed() const {
  // ~5s covers Ogre first map + maximize timers; user Ctrl+C still quits via
  // UserRequestedQuit() regardless of this gate.
  constexpr qint64 kGraceMs = 5000;
  return startup_close_grace_.isValid() &&
         startup_close_grace_.elapsed() >= kGraceMs;
}

void FrameSession::applyStartupWindowState() {
  if (!startup_close_grace_.isValid()) {
    startup_close_grace_.start();
  }
  const AppUiPreferences prefs = LoadAppUiPreferences();
  if (!prefs.start_maximized) {
    if (!frame_->isVisible()) {
      frame_->show();
    }
  } else {
    // Docker/X11: showMaximized() often sets isMaximized() without resizing.
    // Avoid the old maximize↔geometry toggle loop (visible size flicker).
    // Size the window to availableGeometry while still hidden, show once,
    // and only nudge geometry later if the WM left us undersized — never
    // clear Maximized just to setGeometry (that shrinks then grows).
    const auto screen_for = [this]() -> QScreen* {
      if (QScreen* s = frame_->screen()) {
        return s;
      }
      if (frame_->windowHandle() != nullptr) {
        if (QScreen* s = frame_->windowHandle()->screen()) {
          return s;
        }
      }
      return QGuiApplication::primaryScreen();
    };
    const auto fills_screen = [](const QWidget* w, const QRect& area) {
      if (w == nullptr || area.isEmpty()) {
        return false;
      }
      return w->width() >= static_cast<int>(area.width() * 0.95) &&
             w->height() >= static_cast<int>(area.height() * 0.95);
    };

    QScreen* screen = screen_for();
    const QRect area =
        screen != nullptr ? screen->availableGeometry() : QRect();

    Qt::WindowStates st = frame_->windowState();
    st &= ~(Qt::WindowMinimized | Qt::WindowFullScreen);

    if (!frame_->isVisible()) {
      if (!area.isEmpty()) {
        // Final size before first map — setupUi already primed this; refresh
        // in case the screen changed during loadConfig.
        frame_->setGeometry(area);
      }
      frame_->setWindowState(st | Qt::WindowMaximized);
      frame_->show();
      frame_->raise();
    }

    // One deferred correction if the WM mapped us small despite the prime.
    // Do not re-toggle Maximized (that was the flash); only push geometry.
    QTimer::singleShot(0, frame_, [this, screen_for, fills_screen]() {
      if (!LoadAppUiPreferences().start_maximized || frame_ == nullptr) {
        return;
      }
      QScreen* s = screen_for();
      if (s == nullptr) {
        return;
      }
      const QRect a = s->availableGeometry();
      if (a.isEmpty() || fills_screen(frame_, a)) {
        if (!frame_->isMaximized()) {
          frame_->setWindowState(frame_->windowState() | Qt::WindowMaximized);
        }
        return;
      }
      // Keep Maximized bit; setGeometry while maximized is honored on many
      // X11 WMs when the mapped size was wrong. Fallback: replace state with
      // a full-area normal window (still no shrink→grow cycle).
      frame_->setGeometry(a);
      if (!fills_screen(frame_, a)) {
        frame_->setWindowState(Qt::WindowNoState);
        frame_->setGeometry(a);
      } else if (!frame_->isMaximized()) {
        frame_->setWindowState(frame_->windowState() | Qt::WindowMaximized);
      }
    });
  }

  // Show Displays tree shortly after the window maps (avoids blank panel from
  // paint-suppress racing Ogre first paint).
  QTimer::singleShot(900, frame_, [this]() {
    if (frame_->panels_ != nullptr &&
        frame_->panels_->displays_panel_ != nullptr) {
      frame_->panels_->displays_panel_->armTreePainting();
    }
  });

  // One settle pass after first map: resize sidebars + ensure 3D mosaic, then
  // reveal the native GL surface once. Splitting these across 0/50ms timers
  // made the center 3D View flash (create → resize → park/reinstall).
  QTimer::singleShot(0, frame_, [this]() {
    if (frame_->panels_->displays_dock_ != nullptr) {
      frame_->resizeDocks({frame_->panels_->displays_dock_}, {320},
                          Qt::Horizontal);
    }
    if (frame_->panels_->views_dock_ != nullptr) {
      frame_->resizeDocks({frame_->panels_->views_dock_}, {280},
                          Qt::Horizontal);
    }

    if (frame_->layout_->main_panel_host_ != nullptr) {
      bool host_has_viewport = false;
      for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
        if (dock == nullptr ||
            frame_->panels_->panelTypeId(dock) !=
                QLatin1String("ViewportDock") ||
            !dock->requestedVisible()) {
          continue;
        }
        if (frame_->layout_->main_panel_host_->hostsPanel(dock)) {
          host_has_viewport = true;
          frame_->viewport_->ensureViewportPanelReady(dock);
        }
      }
      if (!host_has_viewport) {
        frame_->layout_->applyMainPanelDefaultLayout();
      }
    }

    scheduleViewportRevealAfterLayoutSettle();

    // Optional CI/smoke: Split Right the active 3D View after layout settles.
    const int split_ms = qEnvironmentVariableIntValue("AUTOVIZ_SMOKE_SPLIT_MS");
    if (split_ms > 0 && frame_->viewport_->viewport_dock_ != nullptr) {
      QTimer::singleShot(split_ms, frame_, [this]() {
        if (frame_->viewport_->viewport_dock_ == nullptr) {
          return;
        }
        frame_->layout_->onSplitActiveDock(frame_->viewport_->viewport_dock_,
                                           Qt::Horizontal);
        // Dump mosaic state after deferred Split finish settles.
        QTimer::singleShot(250, frame_, [this]() {
          QFile diag(QStringLiteral("/tmp/autoviz_split_diag.txt"));
          if (!diag.open(QIODevice::WriteOnly | QIODevice::Truncate |
                         QIODevice::Text)) {
            return;
          }
          QTextStream out(&diag);
          int hosted_count = 0;
          for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
            if (dock == nullptr || frame_->layout_->main_panel_host_ == nullptr ||
                !frame_->layout_->main_panel_host_->hostsPanel(dock)) {
              continue;
            }
            ++hosted_count;
            out << "dock name=" << dock->objectName()
                << " title=" << dock->windowTitle()
                << " visible=" << (dock->isVisible() ? 1 : 0)
                << " geo=" << dock->width() << 'x' << dock->height()
                << " at=" << dock->x() << ',' << dock->y() << '\n';
          }
          out << "hosted=" << hosted_count << '\n';
          int vp_count = 0;
          frame_->viewport_->forEachViewportPanel(
              [&](ViewportPanelEntry& entry) {
                ++vp_count;
                out << "viewport name=" << entry.object_name
                    << " widget=" << (entry.widget != nullptr ? 1 : 0)
                    << " ogre=" << (entry.ogre_viewport != nullptr ? 1 : 0);
                if (entry.host != nullptr) {
                  out << " host=" << entry.host->width() << 'x'
                      << entry.host->height();
                }
                if (entry.widget != nullptr) {
                  out << " gl=" << entry.widget->width() << 'x'
                      << entry.widget->height()
                      << " gl_vis=" << (entry.widget->isVisible() ? 1 : 0);
                }
                out << '\n';
              });
          out << "viewport_entries=" << vp_count << '\n';
        });
      });
    }
  });
}

void FrameSession::showLayoutTransitionCover() {
  if (frame_ == nullptr) {
    return;
  }
  if (layout_transition_cover_.isNull()) {
    layout_transition_cover_ = new LayoutTransitionCover(frame_);
  }
  auto* cover =
      static_cast<LayoutTransitionCover*>(layout_transition_cover_.data());
  if (cover == nullptr) {
    return;
  }
  if (cover->effect() != nullptr) {
    cover->effect()->setOpacity(1.0);
  }
  layoutLayoutTransitionCover();
  cover->show();
  cover->raise();
  cover->update();
}

void FrameSession::dismissLayoutTransitionCover() {
  if (layout_transition_cover_.isNull()) {
    return;
  }
  auto* cover =
      static_cast<LayoutTransitionCover*>(layout_transition_cover_.data());
  if (cover == nullptr || !cover->isVisible()) {
    if (layout_transition_cover_ != nullptr) {
      layout_transition_cover_->hide();
    }
    return;
  }
  auto* effect = cover->effect();
  if (effect == nullptr) {
    cover->hide();
    return;
  }
  auto* fade = new QPropertyAnimation(effect, "opacity", cover);
  fade->setDuration(160);
  fade->setStartValue(effect->opacity());
  fade->setEndValue(0.0);
  fade->setEasingCurve(QEasingCurve::OutCubic);
  QObject::connect(fade, &QPropertyAnimation::finished, cover, [cover]() {
    cover->hide();
    if (cover->effect() != nullptr) {
      cover->effect()->setOpacity(1.0);
    }
  });
  fade->start(QAbstractAnimation::DeleteWhenStopped);
}

void FrameSession::layoutLayoutTransitionCover() {
  if (layout_transition_cover_.isNull() || frame_ == nullptr) {
    return;
  }
  layout_transition_cover_->setGeometry(frame_->rect());
  layout_transition_cover_->raise();
}

void FrameSession::scheduleViewportRevealAfterLayoutSettle() {
  // Keep GLX destroyed through mosaic/WM configure; release once hosts are
  // size-stable. Cover (if any) stays up until remount finishes.
  QTimer::singleShot(0, frame_, [this]() {
    if (frame_ == nullptr || frame_->viewport_ == nullptr) {
      return;
    }
    frame_->viewport_->setViewportRevealHeld(true);
    layoutLayoutTransitionCover();
    if (frame_->layout_ != nullptr &&
        frame_->layout_->main_panel_host_ != nullptr) {
      for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
        if (dock == nullptr ||
            frame_->panels_->panelTypeId(dock) !=
                QLatin1String("ViewportDock") ||
            !dock->requestedVisible()) {
          continue;
        }
        if (frame_->layout_->main_panel_host_->hostsPanel(dock)) {
          frame_->viewport_->ensureViewportPanelReady(dock);
        }
      }
    }
    frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
      frame_->viewport_->parkRenderWindowInEntry(entry);
      if (entry.host != nullptr) {
        entry.host->setStyleSheet(QStringLiteral(
            "QWidget#AutovizViewportHost { background-color: #303030; }"));
      }
    });

    if (frame_->layout_ != nullptr &&
        frame_->layout_->main_panel_host_ != nullptr) {
      frame_->layout_->main_panel_host_->syncHorizontalDockLayout();
      frame_->layout_->main_panel_host_->updateGeometry();
    }
    frame_->updateGeometry();

    // Require two consecutive matching host sizes before remount — avoids
    // binding GLX to a mid-layout width (map tearing across dual panes).
    auto last_sizes = std::make_shared<QHash<QString, QSize>>();
    auto stable_hits = std::make_shared<int>(0);

    auto try_release = std::make_shared<std::function<void(int)>>();
    *try_release = [this, try_release, last_sizes, stable_hits](int attempt) {
      if (frame_ == nullptr || frame_->viewport_ == nullptr) {
        dismissLayoutTransitionCover();
        return;
      }
      if (frame_->layout_ != nullptr &&
          frame_->layout_->center_dock_drag_count_ > 0) {
        QTimer::singleShot(40, frame_, [try_release, attempt]() {
          (*try_release)(attempt + 1);
        });
        return;
      }

      bool pending = false;
      bool size_changed = false;
      frame_->viewport_->forEachViewportPanel(
          [&pending, &size_changed, last_sizes](ViewportPanelEntry& entry) {
            if (entry.dock == nullptr || !entry.dock->requestedVisible()) {
              return;
            }
            if (entry.host == nullptr || entry.host->width() < 40 ||
                entry.host->height() < 40) {
              pending = true;
              return;
            }
            const QString key = entry.dock->objectName();
            const QSize now(entry.host->width(), entry.host->height());
            const auto it = last_sizes->constFind(key);
            if (it == last_sizes->constEnd() || it.value() != now) {
              size_changed = true;
              (*last_sizes)[key] = now;
            }
          });

      if (pending || size_changed) {
        *stable_hits = 0;
      } else {
        ++(*stable_hits);
      }

      if ((pending || *stable_hits < 2) && attempt < 16) {
        QTimer::singleShot(32, frame_, [try_release, attempt]() {
          (*try_release)(attempt + 1);
        });
        return;
      }

      // Remount under the cover, then fade the cover after first presents.
      frame_->viewport_->setViewportRevealHeld(false);
      QTimer::singleShot(120, frame_, [this]() {
        dismissLayoutTransitionCover();
        if (frame_ != nullptr) {
          frame_->update();
        }
      });
    };

    QTimer::singleShot(48, frame_, [try_release]() { (*try_release)(0); });
  });
}

void FrameSession::onFixedFrameChanged(const QString& /*frame*/) {
  frame_->viewport_->syncToolContext();
  frame_->chrome_->updateStatusBar();
}

void FrameSession::onAboutToQuit() {
  if (quit_teardown_done_) {
    return;
  }
  quit_teardown_done_ = true;
  // Stop timers / playback, then release display GPU objects while Ogre
  // windows still exist. Do NOT manager_->shutdown() here (Autolink/docks).
  render_timer_.stop();
  refresh_timer_.stop();
  // Unparent GLX surfaces before any later hideChildren / dock walk.
  safeguardNativeViewportsBeforeHide();

  // ~VisualizationFrame destroys FrameViewport/FrameLayout members BEFORE the
  // QWidget child walk. Dock destroyed/visibilityChanged lambdas capture those
  // collaborators via frame_ context — disconnect every dock→frame_ link and
  // silence docks before any hide/destroy walk.
  const QList<QDockWidget*> docks = frame_->findChildren<QDockWidget*>();
  for (QDockWidget* dock : docks) {
    if (dock == nullptr) {
      continue;
    }
    QObject::disconnect(dock, nullptr, frame_, nullptr);
    dock->blockSignals(true);
  }

  if (frame_->manager_ != nullptr) {
    frame_->manager_->playback().stop();
    frame_->manager_->detachDisplaysFromScene();
  }
  frame_->viewport_->forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->hideNativeSurface();
      if (auto* host = entry.ogre_viewport->ogreSceneHost()) {
        host->clear();
      }
    }
  });
  // Sync-destroy before ~VisualizationFrame walks dock children — deferred
  // deleteLater leaves GLX widgets for QObject teardown and SIGSEGVs.
  frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
    frame_->viewport_->destroyRenderWindowInEntry(entry, /*synchronous=*/true);
  });
  frame_->viewport_->clearForQuit();

  // Drop Ogre::Root / GLX headless bootstrap while the X11 display is still
  // alive, so process exit does not leave a live GL stack for driver unload.
  rendering::RenderSystem::destroyInstance();

  QSettings settings;
  settings.setValue(QStringLiteral("recent_configs"), recent_configs_);
}

void FrameSession::leaveEvent(QEvent* event) {
  // RViz2 clears the tool status when the cursor leaves the window.
  frame_->chrome_->status_hint_.clear();
  if (frame_->chrome_->status_label_ != nullptr) {
    frame_->chrome_->status_label_->clear();
  }
}

void FrameSession::onReset() {
  if (frame_->manager_ != nullptr) {
    frame_->manager_->resetTime();
  }
  // Persistent Ogre geometry is not rebuilt every frame; clear hosts now so
  // emptied Displays do not leave stale meshes/point clouds on screen.
  frame_->viewport_->forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport != nullptr) {
      if (auto* host = entry.ogre_viewport->ogreSceneHost()) {
        host->clear();
      }
    }
  });
  if (frame_->panels_->time_panel_ != nullptr) {
    frame_->panels_->time_panel_->syncAfterReset();
  }
  frame_->viewport_->requestViewportUpdate();
}

void FrameSession::resizeEvent(QResizeEvent* event) {
  Q_UNUSED(event);
  // Never retile on resize — that would destroy a directed mosaic.
  // Always scale-fill existing splitter ratios so 3D View + Map stay edge-to
  // edge when the window grows (manual layout used to skip this and left a gap).
  if (frame_->layout_->main_panel_host_ != nullptr &&
      !frame_->layout_->center_tiling_ &&
      !frame_->layout_->suppress_center_tile_) {
    frame_->layout_->main_panel_host_->syncHorizontalDockLayout();
  }
  layoutRecordDropOverlay();
  layoutLayoutTransitionCover();
}

void FrameSession::applyBackgroundColor(const QColor& color) {
  const QColor resolved = color.isValid() ? color : QColor(48, 48, 48);
  frame_->viewport_->forEachViewportPanel([resolved](ViewportPanelEntry& entry) {
    if (entry.host != nullptr) {
      // Stable plate while GL reveal is held — avoids a white flash in the
      // center 3D column during startup maximize / dock resize.
      QPalette pal = entry.host->palette();
      pal.setColor(QPalette::Window, resolved);
      pal.setColor(QPalette::Base, resolved);
      entry.host->setAutoFillBackground(true);
      entry.host->setPalette(pal);
    }
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->setBackgroundColor(resolved);
    }
  });
  frame_->viewport_->requestViewportUpdate();
}

void FrameSession::onResetDefaultLayout() {
  frame_->layout_->applyDefaultDockLayout();
  frame_->manager_->setToolbarTools({});
  frame_->manager_->tools().setActiveTool("Interact");
  frame_->chrome_->rebuildToolbar();
  frame_->chrome_->syncActiveToolUi();
  markConfigModified();
}

void FrameSession::applyShortcutPreferences(
    const QHash<QString, QKeySequence>& shortcuts) {
  const auto apply = [&](QAction* action, const QString& id,
                         const QKeySequence& fallback) {
    if (action == nullptr) {
      return;
    }
    const QKeySequence sequence =
        shortcuts.contains(id) ? shortcuts.value(id) : fallback;
    action->setShortcut(sequence);
  };

  apply(frame_->chrome_->open_config_action_, QStringLiteral("file.open"), QKeySequence::Open);
  apply(frame_->chrome_->open_record_action_, QStringLiteral("file.open_record"),
        QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_O));
  apply(frame_->chrome_->save_config_action_, QStringLiteral("file.save"), QKeySequence::Save);
  apply(frame_->chrome_->save_config_as_action_, QStringLiteral("file.save_as"),
        QKeySequence::SaveAs);
  apply(frame_->chrome_->quit_action_, QStringLiteral("file.quit"), QKeySequence::Quit);
  if (frame_->chrome_->add_panel_action_ != nullptr) {
    frame_->chrome_->add_panel_action_->setShortcut(QKeySequence());
  }
  if (frame_->chrome_->fullscreen_action_ != nullptr) {
    frame_->chrome_->fullscreen_action_->setShortcut(QKeySequence());
  }
  frame_->chrome_->setupToolShortcuts();
}

void FrameSession::applyUiPreferences(const AppSettingsResult& settings,
                                            const AppUiPreferences& previous_ui) {
  AppUiPreferences ui_preferences;
  ui_preferences.language_code = settings.language_code;
  ui_preferences.theme_mode = settings.theme_mode.isEmpty()
                                  ? QStringLiteral("light")
                                  : settings.theme_mode;
  ui_preferences.accent_color = settings.accent_color;
  ui_preferences.start_maximized = settings.start_maximized;
  ui_preferences.viewport_hud_visible = previous_ui.viewport_hud_visible;
  ui_preferences.idle_render_divisor = settings.idle_render_divisor;
  ui_preferences.shortcuts = settings.shortcuts;
  SaveAppUiPreferences(ui_preferences);
  setIdleRenderDivisor(settings.idle_render_divisor);

  if (ui_preferences.start_maximized) {
    frame_->showMaximized();
  }

  if (QApplication* app = qobject_cast<QApplication*>(QApplication::instance())) {
    ApplyUiTheme(ui_preferences.theme_mode, ui_preferences.accent_color, *app);
    // ApplyUiThemeMode already RefreshAppThemeChrome; also force menu/tool QSS.
    if (frame_->chrome_ != nullptr) {
      frame_->chrome_->refreshShellChrome();
    }
    if (ui_preferences.language_code != previous_ui.language_code) {
      InstallAppTranslations(*app, ui_preferences.language_code);
    }
  }
  applyShortcutPreferences(ui_preferences.shortcuts);
}

void FrameSession::onAppSettings() {
  AppSettingsDialog dialog(frame_->manager_.get(), frame_);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }

  const AppUiPreferences previous_ui = LoadAppUiPreferences();
  const AppSettingsResult settings = dialog.resultValues();
  applyUiPreferences(settings, previous_ui);

  if (!settings.fixed_frame.empty()) {
    frame_->manager_->setFixedFrame(settings.fixed_frame);
    frame_->onFixedFrameChanged(QString::fromStdString(settings.fixed_frame));
  }

  if (!settings.transformer_id.empty()) {
    frame_->manager_->transformationManager().setTransformer(settings.transformer_id);
  }

  if (!settings.render_backend.empty() &&
      settings.render_backend != frame_->manager_->renderBackendName()) {
    frame_->manager_->setRenderBackendName(settings.render_backend);
    frame_->viewport_->applyRenderBackend(QString::fromStdString(settings.render_backend));
    syncRenderBackendMenu(QString::fromStdString(settings.render_backend));
  }

  frame_->manager_->setTargetFrameRate(settings.frame_rate);
  frame_->viewport_->applyTargetFrameRate(settings.frame_rate);

  frame_->manager_->setBackgroundColor(settings.background_color);
  frame_->applyBackgroundColor(common::ParseColorProperty(settings.background_color,
                                                  QColor(48, 48, 48)));

  frame_->manager_->setTimeSyncMode(settings.time_sync_mode);
  frame_->manager_->setTimePaused(settings.time_paused);
  frame_->manager_->setPlotSettingsVisible(settings.plot_settings_visible);
  frame_->layout_->showPropertyInspector(settings.plot_settings_visible);

  frame_->manager_->setDockHideState(settings.hide_left_dock, settings.hide_right_dock);
  frame_->layout_->hideLeftDock(settings.hide_left_dock);
  frame_->layout_->hideRightDock(settings.hide_right_dock);
  frame_->chrome_->syncToolbarLayoutControls();

  if (frame_->panels_->displays_panel_ != nullptr) {
    frame_->panels_->displays_panel_->refresh();
  }
  frame_->viewport_->requestViewportUpdate();
  markConfigModified();
}

void FrameSession::onHelpAbout() {
  QString about_text =
      frame_->tr("This is Autoviz.\n\nCompiled against Qt version %1.")
          .arg(QLatin1String(QT_VERSION_STR));
  about_text += frame_->tr("\nViewport: Ogre 1.x (required).");
  QMessageBox::about(QApplication::activeWindow(), frame_->tr("About"), about_text);
}

void FrameSession::onToggleFullscreen() {
  setFullScreen(!frame_->windowState().testFlag(Qt::WindowFullScreen));
}

void FrameSession::onBackendOgre() {
  frame_->manager_->setRenderBackendName("Ogre");
  frame_->viewport_->applyRenderBackend(QStringLiteral("Ogre"));
  syncRenderBackendMenu(QStringLiteral("Ogre"));
  markConfigModified();
}

void FrameSession::syncRenderBackendMenu(const QString& /*name*/) {
  if (frame_->chrome_->backend_ogre_action_ != nullptr) {
    frame_->chrome_->backend_ogre_action_->setChecked(true);
    frame_->chrome_->backend_ogre_action_->setEnabled(true);
  }
}

void FrameSession::safeguardNativeViewportsBeforeHide() {
  if (quit_teardown_done_ || frame_ == nullptr || frame_->viewport_ == nullptr) {
    return;
  }
  frame_->viewport_->forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.widget == nullptr || entry.detached_for_window_hide) {
      return;
    }
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->hideNativeSurface();
    }
    QWidget* w = entry.widget;
    w->setFocusProxy(nullptr);
    if (QWidget* fw = QApplication::focusWidget()) {
      if (fw == w || w->isAncestorOf(fw)) {
        fw->clearFocus();
      }
    }
    w->clearFocus();
    if (entry.layout != nullptr) {
      entry.layout->removeWidget(w);
    }
    // Out of the QWidget tree — hideChildren will not visit this surface.
    w->setParent(nullptr);
    w->hide();
    entry.detached_for_window_hide = true;
  });
}

void FrameSession::restoreNativeViewportsAfterShow() {
  if (quit_teardown_done_ || frame_ == nullptr || frame_->viewport_ == nullptr) {
    return;
  }
  frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
    if (!entry.detached_for_window_hide || entry.widget == nullptr) {
      return;
    }
    entry.detached_for_window_hide = false;
    frame_->viewport_->reinstallRenderWindowInEntry(entry);
  });
}

void FrameSession::changeEvent(QEvent* event) {
  if (event != nullptr && event->type() == QEvent::WindowStateChange) {
    // Belt-and-suspenders: some WMs change state before hide() is visible on
    // the call stack; detach as soon as we learn we are minimized.
    if (frame_ != nullptr && frame_->isMinimized()) {
      safeguardNativeViewportsBeforeHide();
    } else if (frame_ != nullptr && !frame_->isMinimized() &&
               frame_->isVisible()) {
      restoreNativeViewportsAfterShow();
    }
  }
  if (event != nullptr && (event->type() == QEvent::WindowStateChange ||
                           event->type() == QEvent::Hide ||
                           event->type() == QEvent::Show)) {
    syncOffscreenPause();
  }
}

}  // namespace autoviz
