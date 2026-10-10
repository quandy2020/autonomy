/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_viewport.hpp"
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
#include <QPen>
#include <QPointer>
#include <QResizeEvent>
#include <QScreen>
#include <QSettings>
#include <QShortcut>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QSizePolicy>
#include <QTimer>
#include <QCursor>
#include <QVariant>
#include <QTabWidget>
#include <QToolBar>
#include <QToolButton>
#include <QGridLayout>
#include <QQuaternion>
#include <QVBoxLayout>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/tool_manager.hpp"
#include "autoviz/integration/message_queue.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
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
#include "autoviz/ui/plot/plot_config_io.hpp"
#include "autoviz/ui/publish/publish_config_io.hpp"
#include "autoviz/ui/image/image_config_io.hpp"
#include "autoviz/ui/plot/plot_panel.hpp"
#include "autoviz/ui/teleop/teleop_panel.hpp"
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
#include "autoviz/ui/viewport_toolbar.hpp"
#include "autoviz/ui/viewport_hud.hpp"
#include "autoviz/ui/views/panel.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/ui/viewport_panel.hpp"

namespace autoviz {

FrameViewport::FrameViewport(VisualizationFrame* frame) : frame_(frame) {}

ViewportPanelEntry*
FrameViewport::viewportEntryForDock(PanelDockWidget* dock) {
  auto it = viewport_panels_.find(dock);
  return it == viewport_panels_.end() ? nullptr : &(*it);
}

const ViewportPanelEntry*
FrameViewport::viewportEntryForDock(PanelDockWidget* dock) const {
  auto it = viewport_panels_.constFind(dock);
  return it == viewport_panels_.cend() ? nullptr : &(*it);
}

ViewportPanelEntry*
FrameViewport::activeViewportEntry() {
  if (active_viewport_dock_ != nullptr) {
    if (ViewportPanelEntry* entry = viewportEntryForDock(active_viewport_dock_)) {
      return entry;
    }
    // Dock was destroyed without removeViewportPanel — drop the stale key.
    active_viewport_dock_ = nullptr;
  }
  if (viewport_dock_ != nullptr) {
    if (ViewportPanelEntry* entry = viewportEntryForDock(viewport_dock_)) {
      return entry;
    }
    viewport_dock_ = nullptr;
  }
  return viewport_panels_.isEmpty() ? nullptr : &(*viewport_panels_.begin());
}

void FrameViewport::setActiveViewportDock(PanelDockWidget* dock) {
  if (dock == nullptr || !viewport_panels_.contains(dock)) {
    return;
  }
  active_viewport_dock_ = dock;
  frame_->layout_->last_active_dock_ = dock;
  if (ViewportPanelEntry* entry = viewportEntryForDock(dock)) {
    // Keep main toolbar / global tool aligned with frame_ split's local tool.
    if (frame_->manager_ != nullptr &&
        frame_->manager_->tools().activeToolId() != entry->local_tool_id) {
      frame_->manager_->tools().setActiveTool(entry->local_tool_id);
      frame_->chrome_->syncToolbarToActiveTool();
    }
  }
  if (frame_->panels_->views_panel_ != nullptr) {
    frame_->panels_->views_panel_->setViewController(activeViewController());
  }
  frame_->layout_->raiseLeftSidebarDisplays();
  syncViewportTitleBarTools();
  syncToolContext();
}

void FrameViewport::forEachViewportPanel(
    const std::function<void(ViewportPanelEntry&)>& fn) {
  for (auto it = viewport_panels_.begin(); it != viewport_panels_.end(); ++it) {
    fn(*it);
  }
}

void FrameViewport::destroyRenderWindowInEntry(ViewportPanelEntry& entry,
                                               bool synchronous) {
  if (entry.widget != nullptr) {
    entry.widget->hide();
    // Reparent off the dock before touching the layout — host may already be
    // destroying (PanelDockWidget::onChildDestroyed → closed → remove).
    entry.widget->setParent(nullptr);
    if (entry.layout != nullptr) {
      entry.layout->removeWidget(entry.widget);
    }
    if (synchronous) {
      delete entry.widget;
    } else {
      entry.widget->deleteLater();
    }
    entry.widget = nullptr;
    entry.ogre_viewport = nullptr;
  }
}

void FrameViewport::parkRenderWindowInEntry(ViewportPanelEntry& entry) {
  if (entry.widget == nullptr) {
    return;
  }
  if (entry.layout != nullptr) {
    entry.layout->removeWidget(entry.widget);
  }
  // Park on the frame (not a QMainWindow dock area) so mosaic reparent of
  // the empty ViewportDock does not touch the native GLX window.
  entry.widget->setParent(frame_);
  entry.widget->hide();
}

void FrameViewport::reinstallRenderWindowInEntry(ViewportPanelEntry& entry) {
  if (entry.widget == nullptr || entry.layout == nullptr) {
    return;
  }
  entry.layout->addWidget(entry.widget, 0, 0);
  if (entry.floating_toolbar != nullptr) {
    entry.layout->addWidget(entry.floating_toolbar, 0, 0,
                            Qt::AlignRight | Qt::AlignTop);
    entry.floating_toolbar->raise();
  }
  if (entry.hud_overlay != nullptr) {
    entry.layout->addWidget(entry.hud_overlay, 0, 0,
                            Qt::AlignLeft | Qt::AlignTop);
  }
  if (entry.dock != nullptr && entry.dock->isVisible()) {
    if (entry.host != nullptr) {
      entry.host->updateGeometry();
      // Pin the WA_NativeWindow to the post-split host rect before mapping.
      // Otherwise the pre-split full-width GLX surface covers the sibling pane
      // and the UI looks like "only one 3D View".
      if (entry.host->width() >= 2 && entry.host->height() >= 2) {
        entry.widget->setGeometry(entry.host->rect());
      }
    }
    entry.widget->show();
    entry.widget->raise();
    entry.widget->updateGeometry();
    (void)entry.widget->winId();
    if (entry.ogre_viewport != nullptr) {
      // Undo hideNativeSurface / park so the GLX window paints again.
      entry.ogre_viewport->showNativeSurface();
    }
    // Paint only — do not syncToolContext here. Split/mosaic reinstall runs
    // while sibling docks may still be mid-reparent; requestViewportUpdate()
    // would call objectName() on a disposed dock and SIGSEGV.
    entry.widget->update();
  }
}

void FrameViewport::createRenderWindowInEntry(ViewportPanelEntry& entry,
                                                   const QString& /*backend*/) {
  rendering::GpuCapabilities::instance().ensureProbed();
  destroyRenderWindowInEntry(entry);
  entry.saved_3d_view_state.reset();

  // Parent to the dock host (not the QMainWindow). A visible WA_NativeWindow
  // child of the frame flashes a blank top-level GLX surface that covers the
  // Split sibling — the white empty pane after Split Right.
  QWidget* parent =
      entry.host != nullptr ? entry.host : static_cast<QWidget*>(frame_);
  entry.ogre_viewport = new rendering::OgreRenderWindow(parent);
  entry.ogre_viewport->hide();
  entry.ogre_viewport->setSceneOverlay(&frame_->manager_->sceneOverlay());
  entry.ogre_viewport->setToolManager(&frame_->manager_->tools());
  entry.widget = entry.ogre_viewport;

  if (entry.widget != nullptr && entry.layout != nullptr) {
    entry.layout->addWidget(entry.widget, 0, 0);
    if (entry.floating_toolbar != nullptr) {
      entry.layout->addWidget(entry.floating_toolbar, 0, 0,
                              Qt::AlignRight | Qt::AlignTop);
      entry.floating_toolbar->raise();
      entry.floating_toolbar->show();
    }
    if (entry.hud_overlay != nullptr) {
      entry.layout->addWidget(entry.hud_overlay, 0, 0,
                              Qt::AlignLeft | Qt::AlignTop);
      if (entry.hud_overlay->isHudVisible()) {
        entry.hud_overlay->raise();
        entry.hud_overlay->show();
      } else {
        entry.hud_overlay->hide();
      }
    }
  }
  applyViewportEntryRenderSettings(entry);
  connectViewportInteractionsForEntry(entry);

  // Reveal only when the host has a real size. Split creates the dock before
  // QSplitter assigns pane geometry — initializing at 0×0 leaves a blank /
  // missing sibling 3D View.
  const bool sized = entry.host != nullptr && entry.host->width() >= 2 &&
                     entry.host->height() >= 2;
  if (entry.dock != nullptr && entry.dock->requestedVisible() &&
      entry.widget != nullptr && sized) {
    entry.widget->setGeometry(entry.host->rect());
    entry.widget->show();
    entry.widget->raise();
    entry.widget->updateGeometry();
    (void)entry.widget->winId();
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->showNativeSurface();
    }
    entry.widget->update();
  }
}

void FrameViewport::applyViewportEntryRenderSettings(
    ViewportPanelEntry& entry) {
  PanelDockWidget* dock = entry.dock;
  const std::string viewport_key =
      dock != nullptr ? dock->objectName().toStdString() : std::string();
  const auto activate = [this, dock]() {
    setActiveViewportDock(dock);
  };
  if (entry.ogre_viewport != nullptr) {
    entry.ogre_viewport->setBackgroundColor(
        common::ParseColorProperty(frame_->manager_->backgroundColor(),
                                   QColor(48, 48, 48)));
    entry.ogre_viewport->viewController().setFrameManager(&frame_->manager_->frameManager());
    entry.ogre_viewport->setToolManager(&frame_->manager_->tools());
    entry.ogre_viewport->setViewportToolId(entry.local_tool_id);
    entry.ogre_viewport->setViewportActivationCallback(activate);
  }
}

void FrameViewport::connectViewportInteractionsForEntry(
    ViewportPanelEntry& entry) {
  // Drag updates must stay cheap: rebuilding the Views type combo / property
  // visibility on every mouse move makes Orbit/FPS interaction hitch.
  const auto sync_views_live = [this]() {
    if (frame_->panels_->views_panel_ != nullptr) {
      frame_->panels_->views_panel_->syncCameraValuesFromController();
    }
  };
  const auto sync_views_end = [this]() {
    if (frame_->panels_->views_panel_ != nullptr) {
      frame_->panels_->views_panel_->refreshFromController();
    }
  };
  if (entry.ogre_viewport != nullptr) {
    QObject::connect(entry.ogre_viewport,
                     &rendering::OgreRenderWindow::viewDragUpdated, frame_,
                     sync_views_live);
    QObject::connect(entry.ogre_viewport,
                     &rendering::OgreRenderWindow::viewDragEnded, frame_,
                     sync_views_end);
    QObject::connect(entry.ogre_viewport,
                     &rendering::OgreRenderWindow::toolShortcutTriggered, frame_,
                     [this]() { frame_->chrome_->syncActiveToolUi(); });
  }
}

void FrameViewport::registerPrimaryViewportPanel() {
  if (viewport_dock_ == nullptr || viewport_panels_.contains(viewport_dock_)) {
    return;
  }
  ViewportPanelEntry entry;
  entry.dock = viewport_dock_;
  entry.object_name = viewport_dock_->objectName();
  entry.host = viewport_dock_->widget();
  if (entry.host != nullptr) {
    entry.layout = qobject_cast<QGridLayout*>(entry.host->layout());
  }
  viewport_panels_.insert(viewport_dock_, entry);
  active_viewport_dock_ = viewport_dock_;
}

void FrameViewport::removeViewportPanel(PanelDockWidget* dock) {
  if (dock == nullptr || !viewport_panels_.contains(dock)) {
    return;
  }
  ViewportPanelEntry entry = viewport_panels_.take(dock);
  destroyRenderWindowInEntry(entry);

  if (active_viewport_dock_ == dock) {
    active_viewport_dock_ = nullptr;
    if (!viewport_panels_.isEmpty()) {
      active_viewport_dock_ = viewport_panels_.begin().key();
    } else if (viewport_dock_ != nullptr && viewport_dock_ != dock &&
               viewport_panels_.contains(viewport_dock_)) {
      active_viewport_dock_ = viewport_dock_;
    }
    if (frame_->panels_->views_panel_ != nullptr) {
      frame_->panels_->views_panel_->setViewController(activeViewController());
    }
  }
  // Keep viewport_dock_ on the canonical ViewportDock even when closed so the
  // Panels menu can reopen it. Never promote Split duplicates (ViewportDock_2).
  if (dock == viewport_dock_ &&
      dock->objectName() != QLatin1String("ViewportDock")) {
    if (auto* canonical = frame_->findChild<PanelDockWidget*>(
            QStringLiteral("ViewportDock"))) {
      viewport_dock_ = canonical;
    } else {
      viewport_dock_ = nullptr;
    }
  }
}

void FrameViewport::wireViewportPanel(ViewportPanelEntry& entry) {
  if (entry.dock == nullptr) {
    return;
  }
  if (entry.object_name.isEmpty()) {
    entry.object_name = entry.dock->objectName();
  }
  QObject::connect(entry.dock, &PanelDockWidget::activated, frame_,
                   [this, dock = entry.dock]() { setActiveViewportDock(dock); });
  if (entry.dock->property("viewportDestroyedWired").toBool()) {
    return;
  }
  entry.dock->setProperty("viewportDestroyedWired", true);
  // Safety net: onChildDestroyed / unexpected teardown must not leave a
  // dangling key in viewport_panels_ for syncToolContext to dereference.
  QObject::connect(entry.dock, &QObject::destroyed, frame_,
                   [this, dock = entry.dock]() {
                     if (!viewport_panels_.contains(dock)) {
                       return;
                     }
                     ViewportPanelEntry taken = viewport_panels_.take(dock);
                     if (taken.widget != nullptr) {
                       taken.widget->hide();
                       taken.widget->setParent(nullptr);
                       taken.widget->deleteLater();
                       taken.widget = nullptr;
                       taken.ogre_viewport = nullptr;
                     }
                     if (active_viewport_dock_ == dock) {
                       active_viewport_dock_ =
                           viewport_panels_.isEmpty()
                               ? nullptr
                               : viewport_panels_.begin().key();
                     }
                     if (viewport_dock_ == dock) {
                       // Prefer the canonical dock over a Split duplicate.
                       viewport_dock_ = frame_->findChild<PanelDockWidget*>(
                           QStringLiteral("ViewportDock"));
                       if (viewport_dock_ == nullptr) {
                         viewport_dock_ = active_viewport_dock_;
                       }
                     }
                   });
}

void FrameViewport::ensureViewportPanelReady(PanelDockWidget* dock) {
  if (dock == nullptr || frame_->panels_->panelTypeId(dock) != QLatin1String("ViewportDock")) {
    return;
  }

  if (!viewport_panels_.contains(dock)) {
    ViewportPanelEntry entry;
    entry.dock = dock;
    entry.object_name = dock->objectName();
    entry.host = dock->widget();
    if (entry.host != nullptr) {
      entry.layout = qobject_cast<QGridLayout*>(entry.host->layout());
      const QList<ViewportFloatingToolbar*> toolbars =
          entry.host->findChildren<ViewportFloatingToolbar*>();
      if (!toolbars.isEmpty()) {
        entry.floating_toolbar = toolbars.first();
      }
      const QList<ViewportHudOverlay*> huds =
          entry.host->findChildren<ViewportHudOverlay*>();
      if (!huds.isEmpty()) {
        entry.hud_overlay = huds.first();
      }
    }
    viewport_panels_.insert(dock, entry);
    wireViewportPanel(viewport_panels_[dock]);
    installViewportTitleBarToolsForEntry(viewport_panels_[dock]);
    if (active_viewport_dock_ == nullptr) {
      active_viewport_dock_ = dock;
    }
  }

  // Reopen after "close all 3D Views" must restore the canonical pointer.
  if (dock->objectName() == QLatin1String("ViewportDock")) {
    viewport_dock_ = dock;
  }
  if (active_viewport_dock_ == nullptr) {
    active_viewport_dock_ = dock;
  }

  ViewportPanelEntry& entry = viewport_panels_[dock];
  if (entry.host == nullptr) {
    entry.host = dock->widget();
  }
  if (entry.host != nullptr && entry.layout == nullptr) {
    entry.layout = qobject_cast<QGridLayout*>(entry.host->layout());
  }
  if (entry.widget == nullptr) {
    const QString backend = QString::fromStdString(frame_->manager_->renderBackendName());
    createRenderWindowInEntry(entry, backend);
    if (entry.floating_toolbar == nullptr) {
      installViewportFloatingToolbar(entry);
    }
    if (entry.hud_overlay == nullptr) {
      installViewportHudOverlay(entry);
    }
  } else {
    applyViewportEntryRenderSettings(entry);
    // Mosaic tile parks the GL surface on the frame. Reopen / attach paths that
    // skip a full retile must reinstall it or the 3D View stays blank (toolbar
    // only). Only reinstall when actually parked — transient !isVisible during
    // drag/resize must not reparent the GLX window (interaction hitch).
    const bool parked =
        entry.host != nullptr && entry.widget->parentWidget() != entry.host;
    if (parked) {
      reinstallRenderWindowInEntry(entry);
    } else if (dock->isVisible() && entry.widget->parentWidget() == entry.host) {
      const bool sized = entry.host != nullptr && entry.host->width() >= 2 &&
                         entry.host->height() >= 2;
      if (sized) {
        entry.widget->setGeometry(entry.host->rect());
        if (!entry.widget->isVisible()) {
          entry.widget->show();
        }
        entry.widget->raise();
        (void)entry.widget->winId();
        if (entry.ogre_viewport != nullptr) {
          entry.ogre_viewport->showNativeSurface();
        }
        entry.widget->update();
      }
    }
  }

  if (frame_->panels_->views_panel_ != nullptr && active_viewport_dock_ == dock) {
    frame_->panels_->views_panel_->setViewController(entry.viewController());
  }
}

PanelDockWidget* FrameViewport::createViewportPanelDock(
    const QString& object_name, bool create_render_window) {
  const QString dock_name =
      object_name.isEmpty() ? frame_->panels_->uniquePanelObjectName(QStringLiteral("ViewportDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("3D View"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("ViewportDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Panel3D")));

  ViewportPanelEntry entry;
  entry.dock = dock;
  entry.object_name = dock_name;
  entry.host = new QWidget(dock);
  entry.host->setObjectName(QString::fromLatin1(AppThemeIds::kViewportHost));
  // Keep mins small so repeated Split down (4+ stacked panes) can fit.
  entry.host->setMinimumSize(80, 60);
  entry.layout = new QGridLayout(entry.host);
  entry.layout->setContentsMargins(0, 0, 0, 0);
  entry.layout->setSpacing(0);
  dock->setContentWidget(entry.host);

  // Session restore must not create native Ogre windows before the mosaic is
  // applied — reparenting a live GLX window during loadConfig SIGSEGVs in Qt
  // style/palette code on the next activate.
  if (create_render_window) {
    const QString backend =
        QString::fromStdString(frame_->manager_->renderBackendName());
    createRenderWindowInEntry(entry, backend);
    installViewportFloatingToolbar(entry);
    installViewportHudOverlay(entry);
  }

  frame_->layout_->configureMainPanelDock(dock);
  frame_->panels_->registerPanelDock(dock);
  wireViewportPanel(entry);
  installViewportTitleBarToolsForEntry(entry);

  viewport_panels_.insert(dock, entry);
  return dock;
}

void FrameViewport::createViewport(const QString& backend) {
  forEachViewportPanel([this, backend](ViewportPanelEntry& entry) {
    createRenderWindowInEntry(entry, backend);
  });
  if (frame_->panels_->views_panel_ != nullptr) {
    frame_->panels_->views_panel_->setViewController(activeViewController());
  }
  connectViewportInteractions();
  frame_->chrome_->setupToolShortcuts();
}

void FrameViewport::connectViewportInteractions() {
  // Per-viewport signal wiring is handled in connectViewportInteractionsForEntry().
}

void FrameViewport::applyTargetFrameRate(int fps) {
  const int clamped = std::clamp(fps, 1, 120);
  const int interval_ms = std::max(1, 1000 / clamped);
  frame_->session_->render_timer_.setTimerType(Qt::PreciseTimer);
  frame_->session_->render_timer_.setInterval(interval_ms);
  if (!frame_->session_->app_inactive_ && !frame_->session_->render_timer_.isActive()) {
    frame_->session_->render_timer_.start(interval_ms);
  }
}

rendering::ViewController* FrameViewport::activeViewController() {
  if (ViewportPanelEntry* entry = activeViewportEntry()) {
    return entry->viewController();
  }
  return nullptr;
}

void FrameViewport::requestViewportUpdate() {
  // While frame_->manager_->update() is running, displays often call request_redraw from
  // processMessage. Nested syncToolContext + frame_->update() storms the UI thread —
  // especially after returning from the background with a message backlog.
  // The render timer owns update cadence; here we only schedule a paint.
  if (frame_->manager_ != nullptr && frame_->manager_->isUpdating()) {
    forEachViewportPanel([](ViewportPanelEntry& entry) {
      if (entry.widget != nullptr) {
        entry.widget->update();
      }
    });
    return;
  }
  if (frame_->session_->app_inactive_) {
    return;
  }
  syncToolContext();
  if (frame_->manager_ != nullptr) {
    frame_->manager_->update();
  }
  forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.widget != nullptr) {
      entry.widget->update();
    }
  });
}

void FrameViewport::viewportTick(float delta_seconds) {
  syncToolContext();
  forEachViewportPanel([this, delta_seconds](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport == nullptr) {
      return;
    }
    // Hidden 3D docks must not keep painting their native Ogre surface —
    // otherwise Change panel leaves a GL window covering the replacement.
    if (entry.dock != nullptr && !entry.dock->isVisible()) {
      entry.ogre_viewport->hideNativeSurface();
      return;
    }
    // Parked on the frame during mosaic/split reparent — never paint GLX there.
    if (entry.widget != nullptr && entry.widget->parentWidget() == frame_) {
      return;
    }
    entry.ogre_viewport->tick(delta_seconds);
  });
}

void FrameViewport::syncToolContext() {
  ViewportPanelEntry* active_entry = activeViewportEntry();
  if (frame_->manager_ != nullptr) {
    frame_->manager_->displayContext().ogre_scene_host =
        active_entry != nullptr && active_entry->ogre_viewport != nullptr
            ? active_entry->ogre_viewport->ogreSceneHost()
            : nullptr;
  }
  common::ToolContext context;
  context.view_controller = activeViewController();
  context.scene_overlay = &frame_->manager_->sceneOverlay();
  if (active_entry != nullptr) {
    // Never call QObject::objectName on a possibly-disposed dock pointer.
    if (!active_entry->object_name.isEmpty()) {
      context.viewport_key = active_entry->object_name.toStdString();
    } else if (active_entry->dock != nullptr &&
               viewport_panels_.contains(active_entry->dock)) {
      active_entry->object_name = active_entry->dock->objectName();
      context.viewport_key = active_entry->object_name.toStdString();
    }
  }
  if (active_entry != nullptr && active_entry->widget != nullptr) {
    context.viewport_width = active_entry->widget->width();
    context.viewport_height = active_entry->widget->height();
  }
  if (frame_->manager_ != nullptr) {
    auto& display_context = frame_->manager_->displayContext();
    display_context.view_controller = context.view_controller;
    display_context.viewport_width = context.viewport_width;
    display_context.viewport_height = context.viewport_height;
    if (context.view_controller != nullptr && context.viewport_width > 0 &&
        context.viewport_height > 0) {
      const float aspect =
          static_cast<float>(context.viewport_width) /
          static_cast<float>(std::max(1, context.viewport_height));
      display_context.view_matrix = context.view_controller->viewMatrix();
      display_context.projection_matrix =
          context.view_controller->projectionMatrix(aspect);
      display_context.has_view_matrices = true;
    } else {
      display_context.has_view_matrices = false;
    }
  }
  context.gpu_picking_enabled =
      rendering::GpuCapabilities::instance().hasHardwareGpu();
  if (context.gpu_picking_enabled && active_entry != nullptr) {
    if (active_entry->ogre_viewport != nullptr) {
      rendering::OgreRenderWindow* viewport = active_entry->ogre_viewport;
      context.gpu_depth_pick = [viewport](int x, int y, QVector3D* world) {
        if (viewport == nullptr || world == nullptr) {
          return false;
        }
        return viewport->readDepthPick(
            x, y, viewport->viewController().viewMatrix(),
            viewport->viewController().projectionMatrix(
                static_cast<float>(viewport->width()) /
                static_cast<float>(std::max(1, viewport->height()))),
            world);
      };
      context.gpu_pick_id_read = [viewport](int x, int y) {
        if (viewport == nullptr) {
          return common::kInvalidPickHandle;
        }
        return viewport->readPickHandleAt(x, y);
      };
    }
  }
  context.request_redraw = [this]() { requestViewportUpdate(); };
  context.sync_ogre_host = [this]() {
    if (ViewportPanelEntry* entry = activeViewportEntry()) {
      if (frame_->manager_ != nullptr && entry->ogre_viewport != nullptr) {
        frame_->manager_->displayContext().ogre_scene_host =
            entry->ogre_viewport->ogreSceneHost();
      }
    }
  };
  context.set_status = [this](const QString& text) {
    frame_->chrome_->status_hint_ = text.trimmed();
    frame_->chrome_->updateStatusBar();
  };
  context.autolink_node = frame_->manager_->autolinkNode();
  context.fixed_frame = frame_->manager_->fixedFrame();
  context.scene_overlay = &frame_->manager_->sceneOverlay();
  context.display_context = &frame_->manager_->displayContext();
  context.selection_manager = &frame_->manager_->selectionManager();
  context.pick_registry = &frame_->manager_->pickRegistry();
  context.handler_manager = &frame_->manager_->handlerManager();
  context.interactive_markers = &frame_->manager_->interactiveMarkerRegistry();
  context.selections_changed = [this](
                                    const std::vector<common::SelectionEntry>&
                                        entries) { frame_->session_->updateSelectionPanel(entries); };
  context.revert_to_default_tool = [this]() {
    frame_->chrome_->applyActiveTool(frame_->manager_->tools().defaultToolId());
  };
  frame_->manager_->tools().setContext(std::move(context));
  forEachViewportPanel([this](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->setToolManager(&frame_->manager_->tools());
    }
  });
}

void FrameViewport::updateViewportCursor() {
  forEachViewportPanel([this](ViewportPanelEntry& entry) {
    common::Tool* tool = frame_->manager_->tools().toolById(entry.local_tool_id);
    if (tool != nullptr) {
      tool->setCursor(IconLoader::toolCursor(
          QString::fromStdString(entry.local_tool_id)));
    }
    const QCursor cursor =
        tool != nullptr ? tool->cursor() : IconLoader::defaultCursor();
    if (entry.host != nullptr) {
      entry.host->setCursor(cursor);
    }
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->setToolCursor(cursor);
    }
  });
}

void FrameViewport::applyViewController(const QString& name) {
  rendering::ViewController* controller = activeViewController();
  if (controller == nullptr) {
    return;
  }
  controller->setTypeByName(name);
  if (name == QLatin1String("FPS")) {
    if (ViewportPanelEntry* entry = activeViewportEntry()) {
      if (entry->widget != nullptr) {
        entry->widget->setFocus();
      }
    }
  }
  // Avoid frame_->manager_->update() here: during startup callbacks frame_ runs on the
  // UI thread before the splash finishes and can stall on TF / message queues.
  if (frame_->panels_->views_panel_ != nullptr) {
    frame_->panels_->views_panel_->refreshFromController();
  }
  syncViewportTitleBarTools();
  forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.widget != nullptr) {
      entry.widget->update();
    }
  });
}

void FrameViewport::applyRenderBackend(const QString& /*name*/) {
  rendering::GpuCapabilities::instance().ensureProbed();
  if (frame_->manager_->renderBackendName() != "Ogre") {
    frame_->manager_->setRenderBackendName("Ogre");
  }
  if (!rendering::GpuCapabilities::instance().hasHardwareGpu() &&
      frame_->chrome_->status_label_ != nullptr) {
    frame_->chrome_->status_label_->setText(
        frame_->tr("No hardware GPU detected; Ogre may use a software path."));
  }
  createViewport(QStringLiteral("Ogre"));
  forEachViewportPanel([this](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->setToolManager(&frame_->manager_->tools());
    }
  });
  applyViewController(QString::fromStdString(frame_->manager_->viewControllerName()));
  syncToolContext();
  updateViewportCursor();
  // Widget repaint only — do not drain display message queues here (startup).
  forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.widget != nullptr) {
      entry.widget->update();
    }
  });
}

void FrameViewport::syncViewportTitleBarToolsForEntry(
    const ViewportPanelEntry& entry) {
  if (entry.expand_button != nullptr) {
    entry.expand_button->blockSignals(true);
    entry.expand_button->setChecked(frame_->layout_->expanded_main_panel_dock_ == entry.dock);
    entry.expand_button->blockSignals(false);
  }
  if (entry.settings_button != nullptr && frame_->panels_->views_dock_ != nullptr) {
    entry.settings_button->blockSignals(true);
    const Qt::DockWidgetArea views_area = frame_->dockWidgetArea(frame_->panels_->views_dock_);
    const bool views_in_sidebar =
        views_area == Qt::LeftDockWidgetArea ||
        views_area == Qt::RightDockWidgetArea;
    entry.settings_button->setChecked(views_in_sidebar && frame_->panels_->views_dock_->isVisible());
    entry.settings_button->blockSignals(false);
  }
  syncViewportFloatingToolbarForEntry(entry);
}

void FrameViewport::syncViewportFloatingToolbarForEntry(
    const ViewportPanelEntry& entry) {
  if (entry.floating_toolbar == nullptr) {
    return;
  }
  entry.floating_toolbar->setInspectChecked(entry.local_tool_id == "Select");
  entry.floating_toolbar->setMeasureChecked(entry.local_tool_id == "Measure");
  if (rendering::ViewController* controller = entry.viewController()) {
    const auto type = controller->type();
    entry.floating_toolbar->set2dCameraChecked(
        type == rendering::ViewControllerType::kTopDownOrtho);
    const QString frame_label = controller->targetFrameDisplay();
    entry.floating_toolbar->setRecenterToolTip(
        frame_->tr("Re-center On %1").arg(frame_label));
  }
  const bool hud_visible =
      entry.hud_overlay != nullptr && entry.hud_overlay->isHudVisible();
  entry.floating_toolbar->setHudChecked(hud_visible);
}

void FrameViewport::pushViewportLocalTool(ViewportPanelEntry& entry) {
  const std::string viewport_key =
      entry.dock != nullptr ? entry.dock->objectName().toStdString()
                            : std::string();
  if (entry.ogre_viewport != nullptr) {
    entry.ogre_viewport->setViewportToolId(entry.local_tool_id);
  }
  syncViewportFloatingToolbarForEntry(entry);
  if (common::Tool* tool = frame_->manager_->tools().toolById(entry.local_tool_id)) {
    tool->setCursor(IconLoader::toolCursor(
        QString::fromStdString(entry.local_tool_id)));
    const QCursor cursor = tool->cursor();
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->setToolCursor(cursor);
    }
  }
}

void FrameViewport::setViewportLocalTool(PanelDockWidget* dock,
                                              const std::string& tool_id) {
  ViewportPanelEntry* entry = viewportEntryForDock(dock);
  if (entry == nullptr || frame_->manager_->tools().toolById(tool_id) == nullptr) {
    return;
  }
  const std::string previous_tool = entry->local_tool_id;
  const std::string viewport_key =
      dock != nullptr ? dock->objectName().toStdString() : std::string();
  entry->local_tool_id = tool_id;
  pushViewportLocalTool(*entry);
  if (previous_tool == "Measure" && tool_id != "Measure") {
    // clearViewportSession must hit frame_ dock's host, not the active one.
    if (entry->ogre_viewport != nullptr) {
      frame_->manager_->displayContext().ogre_scene_host =
          entry->ogre_viewport->ogreSceneHost();
    }
    frame_->manager_->tools().clearToolViewportSession("Measure", viewport_key);
  }
  if (dock == active_viewport_dock_) {
    frame_->manager_->tools().setActiveTool(tool_id);
    frame_->chrome_->syncToolbarToActiveTool();
    syncToolContext();
    if (frame_->panels_->tool_properties_panel_ != nullptr) {
      frame_->panels_->tool_properties_panel_->refresh();
    }
    frame_->chrome_->updateStatusBar();
  }
  requestViewportUpdate();
}

void FrameViewport::toggleViewportLocalTool(PanelDockWidget* dock,
                                                 const std::string& tool_id) {
  ViewportPanelEntry* entry = viewportEntryForDock(dock);
  if (entry == nullptr) {
    return;
  }
  const std::string next =
      entry->local_tool_id == tool_id
          ? frame_->manager_->tools().defaultToolId()
          : tool_id;
  setViewportLocalTool(dock, next.empty() ? "Interact" : next);
}

void FrameViewport::onViewportToggle2dCamera(PanelDockWidget* dock) {
  ViewportPanelEntry* entry = viewportEntryForDock(dock);
  if (entry == nullptr) {
    return;
  }
  setActiveViewportDock(dock);
  rendering::ViewController* controller = entry->viewController();
  if (controller == nullptr) {
    return;
  }
  const auto type = controller->type();
  const bool in_2d = type == rendering::ViewControllerType::kTopDownOrtho;
  if (in_2d) {
    if (entry->saved_3d_view_state.has_value()) {
      controller->setState(*entry->saved_3d_view_state);
      entry->saved_3d_view_state.reset();
    } else {
      controller->setTypeByName(QStringLiteral("Orbit"));
    }
  } else {
    // TopDownOrtho::setType overwrites yaw/pitch — save full 3D state first.
    entry->saved_3d_view_state = controller->state();
    controller->setTypeByName(QStringLiteral("TopDownOrtho"));
  }
  if (frame_->panels_->views_panel_ != nullptr && dock == active_viewport_dock_) {
    frame_->panels_->views_panel_->refreshFromController();
  }
  syncViewportFloatingToolbarForEntry(*entry);
  requestViewportUpdate();
  frame_->session_->markConfigModified();
}

void FrameViewport::onViewportRecenterOnFrame(PanelDockWidget* dock) {
  ViewportPanelEntry* entry = viewportEntryForDock(dock);
  if (entry == nullptr) {
    return;
  }
  setActiveViewportDock(dock);
  rendering::ViewController* controller = entry->viewController();
  if (controller == nullptr) {
    return;
  }
  // Same as Views panel "Zero": reset yaw/pitch/distance/focal point to defaults
  // in the Target Frame. Only setTarget(0,0,0) is a no-op after orbit-only moves.
  controller->reset();
  if (frame_->panels_->views_panel_ != nullptr && dock == active_viewport_dock_) {
    frame_->panels_->views_panel_->refreshFromController();
  }
  syncViewportFloatingToolbarForEntry(*entry);
  requestViewportUpdate();
  frame_->session_->markConfigModified();
}

void FrameViewport::installViewportFloatingToolbar(ViewportPanelEntry& entry) {
  if (entry.host == nullptr || entry.floating_toolbar != nullptr) {
    return;
  }
  entry.floating_toolbar = new ViewportFloatingToolbar(entry.host);
  PanelDockWidget* dock = entry.dock;
  ViewportFloatingToolbarCallbacks callbacks;
  callbacks.on_inspect = [this, dock]() {
    setActiveViewportDock(dock);
    toggleViewportLocalTool(dock, "Select");
  };
  callbacks.on_toggle_2d_camera = [this, dock]() {
    onViewportToggle2dCamera(dock);
  };
  callbacks.on_measure = [this, dock]() {
    setActiveViewportDock(dock);
    toggleViewportLocalTool(dock, "Measure");
  };
  callbacks.on_recenter_frame = [this, dock]() {
    onViewportRecenterOnFrame(dock);
  };
  callbacks.on_toggle_hud = [this](bool visible) {
    forEachViewportPanel([visible](ViewportPanelEntry& panel) {
      if (panel.hud_overlay != nullptr) {
        panel.hud_overlay->setHudVisible(visible);
      }
    });
    AppUiPreferences preferences = LoadAppUiPreferences();
    preferences.viewport_hud_visible = visible;
    SaveAppUiPreferences(preferences);
    forEachViewportPanel([this](ViewportPanelEntry& panel) {
      syncViewportFloatingToolbarForEntry(panel);
    });
  };
  entry.floating_toolbar->setCallbacks(std::move(callbacks));
  if (entry.layout != nullptr) {
    entry.layout->addWidget(entry.floating_toolbar, 0, 0,
                            Qt::AlignRight | Qt::AlignTop);
    entry.layout->setContentsMargins(8, 8, 8, 8);
  }
  entry.floating_toolbar->raise();
  entry.floating_toolbar->show();
  syncViewportFloatingToolbarForEntry(entry);
}

void FrameViewport::installViewportHudOverlay(ViewportPanelEntry& entry) {
  if (entry.host == nullptr || entry.hud_overlay != nullptr) {
    return;
  }
  entry.hud_overlay = new ViewportHudOverlay(frame_->manager_.get(), entry.host);
  const bool visible = LoadAppUiPreferences().viewport_hud_visible;
  entry.hud_overlay->setHudVisible(visible);
  if (entry.layout != nullptr) {
    entry.layout->addWidget(entry.hud_overlay, 0, 0,
                            Qt::AlignLeft | Qt::AlignTop);
    entry.layout->setContentsMargins(8, 8, 8, 8);
  }
  if (visible) {
    entry.hud_overlay->raise();
  }
  if (entry.floating_toolbar != nullptr) {
    entry.floating_toolbar->setHudChecked(visible);
  }
}

void FrameViewport::syncViewportTitleBarTools() {
  forEachViewportPanel([this](ViewportPanelEntry& entry) {
    syncViewportTitleBarToolsForEntry(entry);
  });
}

void FrameViewport::installViewportTitleBarToolsForEntry(
    ViewportPanelEntry& entry) {
  PanelDockWidget* dock = entry.dock;
  if (dock == nullptr) {
    return;
  }

  const PanelContextMenuCallbacks callbacks = frame_->chrome_->makePanelContextMenuCallbacks(dock);
  PanelTitleBarOptions options;
  options.show_settings = true;
  options.on_settings_toggled = [this, dock](bool checked) {
    if (frame_->panels_->views_dock_ == nullptr || frame_->panels_->views_panel_ == nullptr) {
      return;
    }
    setActiveViewportDock(dock);
    if (checked) {
      frame_->layout_->ensureSidebarDockAttached(frame_->panels_->views_dock_);
      if (rendering::ViewController* controller = activeViewController()) {
        frame_->panels_->views_panel_->setViewController(controller);
        frame_->panels_->views_panel_->refreshFromController();
      }
      frame_->panels_->views_dock_->show();
      frame_->panels_->views_dock_->raise();
      frame_->layout_->last_active_dock_ = frame_->panels_->views_dock_;
    } else {
      frame_->panels_->views_dock_->hide();
      frame_->layout_->last_active_dock_ = dock;
    }
    syncViewportTitleBarTools();
  };
  options.show_expand = true;
  options.expand_checkable = true;
  options.on_expand = [this, dock]() { frame_->layout_->expandPanelDock(dock); };

  const PanelTitleBarTools tools =
      CreatePanelTitleBarTools(dock, callbacks, options);
  entry.settings_button = tools.settings_button;
  entry.expand_button = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
  syncViewportTitleBarToolsForEntry(entry);
}

}  // namespace autoviz
