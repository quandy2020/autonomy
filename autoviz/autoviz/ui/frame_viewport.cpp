/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_viewport.hpp"
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/common/selection.hpp"
#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <vector>
#include <cstddef>
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
#include "autoviz/common/interaction/message_queue.hpp"
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
    // Displays panel edits only this 3D View's display tree.
    if (frame_->manager_ != nullptr && entry->visibility_bit != 0) {
      frame_->manager_->setActiveViewportBit(entry->visibility_bit);
      if (frame_->panels_->displays_panel_ != nullptr) {
        frame_->panels_->displays_panel_->refresh();
      }
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
  if (entry.widget == nullptr) {
    return;
  }
  // ViewsPanel holds a raw pointer into OgreRenderWindow::viewController().
  // Detach before delete — otherwise onRefreshTick UAFs in targetFrameDisplay
  // after mosaic drag recreate (QString::fromUtf8 SEGV).
  if (frame_ != nullptr && frame_->panels_ != nullptr &&
      frame_->panels_->views_panel_ != nullptr &&
      entry.ogre_viewport != nullptr) {
    frame_->panels_->views_panel_->detachViewControllerIf(
        &entry.ogre_viewport->viewController());
  }
  QWidget* w = entry.widget;
  entry.widget = nullptr;
  entry.ogre_viewport = nullptr;

  // hide() → hide_helper → focusNextPrevChild → focusProxy SEGV'd when the
  // WA_NativeWindow Ogre child still had a dangling focus chain during quit
  // (CloseEvent right after RenderSystem ready under Docker/X11).
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
  w->hide();
  // Synchronous delete keeps the parent — setParent(nullptr) on a live
  // WA_NativeWindow maps a top-level X window (see-through mosaic-drag hole).
  if (synchronous) {
    delete w;
  } else {
    w->setParent(nullptr);
    w->hide();
    w->deleteLater();
  }
}

void FrameViewport::clearForQuit() {
  viewport_panels_.clear();
  active_viewport_dock_ = nullptr;
  viewport_dock_ = nullptr;
}

void FrameViewport::parkRenderWindowInEntry(ViewportPanelEntry& entry) {
  // Title-drag: unmap GLX but keep the charcoal QWidget plate visible in the
  // hosted leaf. Hiding/destroying WA_NativeWindow punches an X11 hole so the
  // IDE shows through "3D View". Do not delete OgreRenderWindow (SceneHost UAF).
  if (entry.hud_overlay != nullptr) {
    if (entry.layout != nullptr) {
      entry.layout->removeWidget(entry.hud_overlay);
    }
    entry.hud_overlay->setAttribute(Qt::WA_AlwaysStackOnTop, false);
    entry.hud_overlay->setHudVisible(false);
    entry.hud_overlay->hide();
  }
  if (entry.ogre_viewport != nullptr) {
    entry.ogre_viewport->releaseNativeWindowForPark();
  } else if (entry.widget != nullptr) {
    entry.widget->show();
    entry.widget->update();
  }
  // Settle must recreate at the post-snap pane size — stale GLX geometry is
  // the black/purple/grey 黑边 after mosaic 吸附.
  entry.needs_gl_recreate = true;
  if (entry.host != nullptr) {
    entry.host->setAttribute(Qt::WA_OpaquePaintEvent, true);
    entry.host->setAutoFillBackground(true);
    entry.host->setStyleSheet(
        QStringLiteral("QWidget#AutovizViewportHost { background-color: #303030; }"));
    entry.host->show();
    entry.host->update();
  }
  if (entry.widget != nullptr && entry.layout != nullptr &&
      entry.widget->parentWidget() != entry.host) {
    entry.layout->addWidget(entry.widget, 0, 0);
  }
  if (entry.widget != nullptr && entry.host != nullptr &&
      entry.host->width() >= 2 && entry.host->height() >= 2) {
    entry.widget->setGeometry(entry.host->rect());
    entry.widget->show();
    entry.widget->raise();
    entry.widget->update();
  }
}

void FrameViewport::reinstallRenderWindowInEntry(ViewportPanelEntry& entry) {
  // Park sets needs_gl_recreate — recreate only after reveal is released.
  // Doing it under reveal_held_ (loadConfig / mosaic restore) flashes a
  // transparent GLX child while docks are still shuffling.
  if (!reveal_held_ && entry.needs_gl_recreate && entry.host != nullptr &&
      entry.host->width() >= 2 && entry.host->height() >= 2) {
    // Skip while mosaic drag / splitter mutate — host size is still wrong and
    // a premature recreate locks in black bars until the next full restart.
    if (frame_ != nullptr && frame_->layout_ != nullptr) {
      if (frame_->layout_->isCenterDockDragging()) {
        return;
      }
      if (frame_->layout_->main_panel_host_ != nullptr &&
          frame_->layout_->main_panel_host_->isMutatingMosaic()) {
        return;
      }
    }
    recreateRenderSurfaceInEntry(entry);
    return;
  }
  if (entry.widget == nullptr || entry.layout == nullptr) {
    return;
  }
  // Legacy park used setParent(frame_); put the widget back if needed.
  if (entry.widget->parentWidget() != entry.host) {
    entry.layout->addWidget(entry.widget, 0, 0);
  }
  // Never remount while the leaf is mid mosaic title-drag (parking lot).
  if (frame_ != nullptr && frame_->layout_ != nullptr &&
      frame_->layout_->isCenterDockDragging()) {
    const bool hosted =
        frame_->layout_->main_panel_host_ != nullptr &&
        entry.dock != nullptr &&
        frame_->layout_->main_panel_host_->hostsPanel(entry.dock);
    if (!hosted) {
      return;
    }
  }
  // Free-float / intentionalFloat docks may still be mapping (isVisible false
  // briefly). Prefer hosted / floating so parking mid-drag cannot remount GLX.
  const bool hosted =
      frame_ != nullptr && frame_->layout_ != nullptr &&
      frame_->layout_->main_panel_host_ != nullptr && entry.dock != nullptr &&
      frame_->layout_->main_panel_host_->hostsPanel(entry.dock);
  const bool should_show =
      entry.dock != nullptr &&
      (entry.dock->isVisible() || hosted || entry.dock->isFloating() ||
       entry.dock->property("intentionalFloat").toBool() ||
       (entry.dock->requestedVisible() && !entry.needs_gl_recreate));
  if (!should_show) {
    return;
  }
  if (entry.host != nullptr) {
    entry.host->show();
    entry.host->updateGeometry();
    // Pin the WA_NativeWindow to the post-split / floated host rect before
    // mapping. Otherwise a stale full-width GLX surface covers siblings, or a
    // 0×0 host after undock leaves the float looking empty.
    if (entry.host->width() >= 2 && entry.host->height() >= 2) {
      entry.widget->setGeometry(entry.host->rect());
    } else if (entry.dock != nullptr && entry.dock->width() >= 2 &&
               entry.dock->height() >= 2) {
      entry.host->resize(entry.dock->size());
      entry.widget->setGeometry(entry.host->rect());
    }
  }
  entry.widget->updateGeometry();
  if (!reveal_held_) {
    (void)entry.widget->winId();
    if (entry.ogre_viewport != nullptr) {
      // coverNativeHole rebinds then shows — never bare QWidget::show().
      entry.ogre_viewport->showNativeSurface();
      entry.ogre_viewport->notifyParentWindowMoved();
    }
  } else if (entry.ogre_viewport != nullptr) {
    entry.ogre_viewport->hideNativeSurface();
  }
}

void FrameViewport::recreateRenderSurfaceInEntry(ViewportPanelEntry& entry) {
  if (entry.host == nullptr || entry.layout == nullptr) {
    return;
  }
  std::optional<rendering::ViewState> camera;
  if (rendering::ViewController* controller = entry.viewController()) {
    camera = controller->state();
  }
  const std::shared_ptr<rendering::SceneOverlay> overlay = entry.scene_overlay;
  const std::string tool = entry.local_tool_id;

  destroyRenderWindowInEntry(entry, /*synchronous=*/true);
  entry.scene_overlay =
      overlay != nullptr ? overlay
                         : std::make_shared<rendering::SceneOverlay>();
  entry.local_tool_id = tool;
  createRenderWindowInEntry(entry, QStringLiteral("Ogre"));
  if (camera.has_value() && entry.ogre_viewport != nullptr) {
    entry.ogre_viewport->viewController().setState(*camera);
  }
  entry.needs_gl_recreate = false;
  if (entry.host != nullptr) {
    entry.host->setStyleSheet(
        QStringLiteral("QWidget#AutovizViewportHost { background-color: #303030; }"));
    if (entry.host->width() >= 2 && entry.host->height() >= 2 &&
        entry.widget != nullptr) {
      entry.widget->setGeometry(entry.host->rect());
    }
  }
  if (!reveal_held_ && entry.ogre_viewport != nullptr) {
    entry.ogre_viewport->showNativeSurface();
    entry.ogre_viewport->notifyParentWindowMoved();
  }
  // Rebind ViewsPanel to the (possibly new) active controller after recreate.
  if (frame_ != nullptr && frame_->panels_ != nullptr &&
      frame_->panels_->views_panel_ != nullptr) {
    frame_->panels_->views_panel_->setViewController(activeViewController());
  }
  // New OgreSceneHost is empty until displays redraw into it.
  requestViewportUpdate();
}

void FrameViewport::scheduleSettleReinstallAll(int delay_ms,
                                               bool close_broken_duplicates) {
  ++settle_reinstall_epoch_;
  // delay_ms < 0: cancel in-flight settle only (e.g. new mosaic drag started).
  if (delay_ms < 0 || frame_ == nullptr) {
    return;
  }
  const int epoch = settle_reinstall_epoch_;
  const int wait_ms = delay_ms;

  const auto try_reveal = [this, epoch]() -> bool {
    if (epoch != settle_reinstall_epoch_ || frame_ == nullptr || reveal_held_) {
      return true;
    }
    bool pending = false;
    forEachViewportPanel([this, &pending](ViewportPanelEntry& entry) {
      if (entry.dock == nullptr) {
        return;
      }
      const bool hosted =
          frame_->layout_ != nullptr &&
          frame_->layout_->main_panel_host_ != nullptr &&
          frame_->layout_->main_panel_host_->hostsPanel(entry.dock);
      const bool want_visible =
          entry.dock->isVisible() || entry.dock->isFloating() ||
          entry.dock->requestedVisible() ||
          entry.dock->property("intentionalFloat").toBool() || hosted;
      if (!want_visible) {
        return;
      }
      if (entry.host == nullptr) {
        ensureViewportPanelReady(entry.dock);
      }
      if (entry.host == nullptr) {
        pending = true;
        return;
      }
      if (entry.host->width() < 2 || entry.host->height() < 2) {
        pending = true;
        return;
      }

      entry.host->updateGeometry();
      if (entry.dock != nullptr) {
        entry.dock->updateGeometry();
      }

      const bool size_mismatch =
          entry.widget != nullptr &&
          (qAbs(entry.widget->width() - entry.host->width()) > 2 ||
           qAbs(entry.widget->height() - entry.host->height()) > 2);

      // Dragged panes / stale GLX: destroy + new QWidget at final pane size.
      // Size mismatch after 吸附 is the usual black/grey 黑边 source.
      if (entry.needs_gl_recreate || size_mismatch ||
          entry.ogre_viewport == nullptr || entry.widget == nullptr) {
        entry.needs_gl_recreate = true;
        recreateRenderSurfaceInEntry(entry);
      } else {
        if (entry.widget->parentWidget() != entry.host) {
          reinstallRenderWindowInEntry(entry);
        }
        entry.widget->setGeometry(entry.host->rect());
        (void)entry.widget->winId();
        entry.ogre_viewport->showNativeSurface();
        entry.ogre_viewport->notifyParentWindowMoved();
      }

      if (entry.ogre_viewport == nullptr || entry.widget == nullptr ||
          !entry.widget->isVisible()) {
        pending = true;
        return;
      }
      // Drop the charcoal plate once GL is live — avoids a second flash.
      if (entry.host != nullptr) {
        entry.host->setStyleSheet(QString());
      }
      if (entry.hud_overlay != nullptr && entry.hud_overlay->isVisible()) {
        entry.hud_overlay->raise();
      }
    });
    requestViewportUpdate();
    return !pending;
  };

  QTimer::singleShot(wait_ms, frame_, [this, epoch, try_reveal,
                                       close_broken_duplicates]() {
    if (epoch != settle_reinstall_epoch_) {
      return;
    }
    // Freeze paints while remounting so dual-pane GLX does not tear mid-bind.
    QWidget* mosaic = (frame_->layout_ != nullptr)
                          ? static_cast<QWidget*>(frame_->layout_->main_panel_host_)
                          : nullptr;
    if (mosaic != nullptr) {
      mosaic->setUpdatesEnabled(false);
    }
    const bool done = try_reveal();
    if (mosaic != nullptr) {
      mosaic->setUpdatesEnabled(true);
    }
    if (done) {
      if (close_broken_duplicates) {
        closeTransparentViewportDuplicates();
      }
      return;
    }
    // Sequential retries only — the old parallel {32,80,160,320} barrage
    // remounted GLX several times and caused loadConfig tearing/flicker.
    const std::vector<int> gaps = {40, 80, 160};
    auto chain = std::make_shared<std::function<void(size_t)>>();
    *chain = [this, epoch, try_reveal, close_broken_duplicates, gaps,
              chain, mosaic](size_t index) {
      if (epoch != settle_reinstall_epoch_ || frame_ == nullptr) {
        return;
      }
      if (mosaic != nullptr) {
        mosaic->setUpdatesEnabled(false);
      }
      const bool ok = try_reveal();
      if (mosaic != nullptr) {
        mosaic->setUpdatesEnabled(true);
      }
      if (ok) {
        if (close_broken_duplicates) {
          closeTransparentViewportDuplicates();
        }
        return;
      }
      if (index >= gaps.size()) {
        if (close_broken_duplicates) {
          closeTransparentViewportDuplicates();
        }
        return;
      }
      QTimer::singleShot(gaps[index], frame_, [chain, index]() {
        (*chain)(index + 1);
      });
    };
    QTimer::singleShot(gaps[0], frame_, [chain]() { (*chain)(1); });
  });
}

void FrameViewport::closeTransparentViewportDuplicates() {
  if (frame_ == nullptr) {
    return;
  }
  // Only close floating / intentionalFloat ghosts with no live GL. Never close
  // hosted mosaic panes — intentional Split right / dual-pane snap must keep
  // both (Foxglove multi-3D model).
  std::vector<PanelDockWidget*> ghosts;
  forEachViewportPanel([this, &ghosts](ViewportPanelEntry& entry) {
    if (entry.dock == nullptr) {
      return;
    }
    const bool floating =
        entry.dock->isFloating() ||
        entry.dock->property("intentionalFloat").toBool();
    if (!floating) {
      return;
    }
    const bool hosted =
        frame_->layout_ != nullptr &&
        frame_->layout_->main_panel_host_ != nullptr &&
        frame_->layout_->main_panel_host_->hostsPanel(entry.dock);
    if (hosted) {
      return;
    }
    const bool live_gl = entry.ogre_viewport != nullptr &&
                         entry.ogre_viewport->hasLiveGlSurface() &&
                         entry.widget != nullptr && entry.widget->isVisible();
    if (!live_gl) {
      if (entry.ogre_viewport != nullptr) {
        entry.ogre_viewport->hideNativeSurface();
      }
      ghosts.push_back(entry.dock);
    }
  });
  for (PanelDockWidget* dock : ghosts) {
    if (dock != nullptr) {
      dock->close();
    }
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
  // Per-entry overlay — do not share the manager global overlay across splits.
  if (entry.scene_overlay == nullptr) {
    entry.scene_overlay = std::make_shared<rendering::SceneOverlay>();
  }
  entry.ogre_viewport->setSceneOverlay(entry.scene_overlay.get());
  entry.ogre_viewport->setToolManager(&frame_->manager_->tools());
  entry.widget = entry.ogre_viewport;

  if (entry.widget != nullptr && entry.layout != nullptr) {
    entry.layout->addWidget(entry.widget, 0, 0);
  }
  // Never remount the glass HUD — AlwaysStackOnTop floats over other apps
  // during mosaic drag. installViewportHudOverlay() only tears leftovers down.
  if (entry.hud_overlay != nullptr) {
    installViewportHudOverlay(entry);
  }
  applyViewportEntryRenderSettings(entry);
  connectViewportInteractionsForEntry(entry);

  // Keep the Qt widget in the layout for geometry, but do not map the native
  // GLX surface until startup settle (reveal_held_) and the host is sized.
  const bool sized = entry.host != nullptr && entry.host->width() >= 2 &&
                     entry.host->height() >= 2;
  const bool mosaic_dragging =
      frame_ != nullptr && frame_->layout_ != nullptr &&
      frame_->layout_->isCenterDockDragging();
  const bool hosted =
      frame_ != nullptr && frame_->layout_ != nullptr &&
      frame_->layout_->main_panel_host_ != nullptr && entry.dock != nullptr &&
      frame_->layout_->main_panel_host_->hostsPanel(entry.dock);
  const bool want_map =
      entry.dock != nullptr && sized && entry.widget != nullptr &&
      (hosted || entry.dock->isVisible() || entry.dock->isFloating() ||
       entry.dock->property("intentionalFloat").toBool()) &&
      !(mosaic_dragging && !hosted);
  if (want_map) {
    entry.widget->setGeometry(entry.host->rect());
    entry.widget->updateGeometry();
    if (!reveal_held_) {
      (void)entry.widget->winId();
      if (entry.ogre_viewport != nullptr) {
        entry.ogre_viewport->showNativeSurface();
      }
    } else if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->hideNativeSurface();
    }
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
  // Primary was registered without createViewportPanelDock — still need a
  // visibility bit or drawViewport() skips the pane (no Grid / displays).
  if (frame_->manager_ != nullptr) {
    entry.visibility_bit = frame_->manager_->allocViewportBit();
    if (entry.visibility_bit != 0) {
      frame_->manager_->claimOrphanDisplaysForViewport(entry.visibility_bit);
      frame_->manager_->setActiveViewportBit(entry.visibility_bit);
    }
  }
  viewport_panels_.insert(viewport_dock_, entry);
  active_viewport_dock_ = viewport_dock_;
}

void FrameViewport::removeViewportPanel(PanelDockWidget* dock) {
  if (dock == nullptr || !viewport_panels_.contains(dock)) {
    return;
  }
  ViewportPanelEntry entry = viewport_panels_.take(dock);
  if (frame_->manager_ != nullptr && entry.visibility_bit != 0) {
    frame_->manager_->detachDisplaysFromViewport(entry.visibility_bit);
    frame_->manager_->freeViewportBit(entry.visibility_bit);
    entry.visibility_bit = 0;
  }
  destroyRenderWindowInEntry(entry);

  if (active_viewport_dock_ == dock) {
    active_viewport_dock_ = nullptr;
    if (!viewport_panels_.isEmpty()) {
      active_viewport_dock_ = viewport_panels_.begin().key();
    } else if (viewport_dock_ != nullptr && viewport_dock_ != dock &&
               viewport_panels_.contains(viewport_dock_)) {
      active_viewport_dock_ = viewport_dock_;
    }
    if (ViewportPanelEntry* next = activeViewportEntry()) {
      if (frame_->manager_ != nullptr && next->visibility_bit != 0) {
        frame_->manager_->setActiveViewportBit(next->visibility_bit);
      }
    }
    if (frame_->panels_->views_panel_ != nullptr) {
      frame_->panels_->views_panel_->setViewController(activeViewController());
    }
    if (frame_->panels_->displays_panel_ != nullptr) {
      frame_->panels_->displays_panel_->refresh();
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
                     if (frame_ == nullptr || frame_->session_ == nullptr ||
                         frame_->session_->quit_teardown_done_) {
                       return;
                     }
                     if (!viewport_panels_.contains(dock)) {
                       return;
                     }
                     ViewportPanelEntry taken = viewport_panels_.take(dock);
                     if (frame_->manager_ != nullptr &&
                         taken.visibility_bit != 0) {
                       frame_->manager_->detachDisplaysFromViewport(
                           taken.visibility_bit);
                       frame_->manager_->freeViewportBit(taken.visibility_bit);
                       taken.visibility_bit = 0;
                     }
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
                       if (ViewportPanelEntry* next = activeViewportEntry()) {
                         if (frame_->manager_ != nullptr &&
                             next->visibility_bit != 0) {
                           frame_->manager_->setActiveViewportBit(
                               next->visibility_bit);
                         }
                       }
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
  // Session reopen / primary path may still lack a bit after register.
  if (entry.visibility_bit == 0 && frame_->manager_ != nullptr) {
    entry.visibility_bit = frame_->manager_->allocViewportBit();
    if (entry.visibility_bit != 0) {
      const bool first =
          std::count_if(viewport_panels_.cbegin(), viewport_panels_.cend(),
                        [](const auto& item) {
                          return item.visibility_bit != 0;
                        }) <= 1;
      if (first) {
        frame_->manager_->claimOrphanDisplaysForViewport(entry.visibility_bit);
        frame_->manager_->setActiveViewportBit(entry.visibility_bit);
      } else {
        frame_->manager_->seedViewportDisplaysFromPeer(entry.visibility_bit);
      }
    }
  }
  if (entry.widget == nullptr) {
    const QString backend = QString::fromStdString(frame_->manager_->renderBackendName());
    createRenderWindowInEntry(entry, backend);
    if (entry.hud_overlay == nullptr) {
      installViewportHudOverlay(entry);
    }
  } else {
    applyViewportEntryRenderSettings(entry);
    // Park sets needs_gl_recreate without reparenting — honor that flag so
    // reopen / split deferred ensure does not leave a blank charcoal pane.
    const bool parked =
        entry.needs_gl_recreate ||
        (entry.host != nullptr && entry.widget->parentWidget() != entry.host);
    if (parked) {
      reinstallRenderWindowInEntry(entry);
    } else if (dock->isVisible() && entry.widget->parentWidget() == entry.host) {
      const bool sized = entry.host != nullptr && entry.host->width() >= 2 &&
                         entry.host->height() >= 2;
      if (sized) {
        entry.widget->setGeometry(entry.host->rect());
        if (!reveal_held_) {
          (void)entry.widget->winId();
          if (entry.ogre_viewport != nullptr) {
            entry.ogre_viewport->showNativeSurface();
          }
        } else if (entry.ogre_viewport != nullptr) {
          entry.ogre_viewport->hideNativeSurface();
        }
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
  // Opaque plate under the WA_PaintOnScreen GL child. If GLX is down/hidden,
  // this plate must fill the dock — otherwise the pane is a transparent hole.
  entry.host->setAttribute(Qt::WA_OpaquePaintEvent, true);
  entry.host->setAttribute(Qt::WA_NoSystemBackground, false);
  entry.host->setAutoFillBackground(true);
  {
    const QColor plate =
        frame_->manager_ != nullptr
            ? common::ParseColorProperty(frame_->manager_->backgroundColor(),
                                         QColor(48, 48, 48))
            : QColor(48, 48, 48);
    QPalette pal = entry.host->palette();
    pal.setColor(QPalette::Window, plate);
    pal.setColor(QPalette::Base, plate);
    entry.host->setPalette(pal);
    // Stylesheet beats theme polish that otherwise leaves a light dock plate.
    entry.host->setStyleSheet(
        QStringLiteral("QWidget#AutovizViewportHost { background-color: %1; }")
            .arg(plate.name()));
  }
  // Keep mins small so repeated Split down (4+ stacked panes) can fit.
  entry.host->setMinimumSize(80, 60);
  entry.layout = new QGridLayout(entry.host);
  entry.layout->setContentsMargins(0, 0, 0, 0);
  entry.layout->setSpacing(0);
  dock->setContentWidget(entry.host);

  // Independent Display ownership for this pane. First view claims session
  // orphans; interactive Split starts empty. Session restore of extra 3D
  // Views (create_render_window=false) clones the primary tree so dual-pane
  // layouts keep the same cool nav displays.
  if (frame_->manager_ != nullptr) {
    entry.visibility_bit = frame_->manager_->allocViewportBit();
    const bool first_viewport = viewport_panels_.isEmpty();
    if (first_viewport && entry.visibility_bit != 0) {
      frame_->manager_->claimOrphanDisplaysForViewport(entry.visibility_bit);
      frame_->manager_->setActiveViewportBit(entry.visibility_bit);
    } else if (!create_render_window && entry.visibility_bit != 0) {
      frame_->manager_->seedViewportDisplaysFromPeer(entry.visibility_bit);
    }
  }

  // Session restore must not create native Ogre windows before the mosaic is
  // applied — reparenting a live GLX window during loadConfig SIGSEGVs in Qt
  // style/palette code on the next activate.
  if (create_render_window) {
    const QString backend =
        QString::fromStdString(frame_->manager_->renderBackendName());
    createRenderWindowInEntry(entry, backend);
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
    const ViewportPanelEntry* active_entry = activeViewportEntry();
    forEachViewportPanel([this, active_entry](ViewportPanelEntry& entry) {
      if (entry.visibility_bit == 0 || entry.scene_overlay == nullptr) {
        return;
      }
      const bool is_active =
          active_entry != nullptr && entry.dock == active_entry->dock;
      rendering::OgreSceneHost* host =
          entry.ogre_viewport != nullptr ? entry.ogre_viewport->ogreSceneHost()
                                         : nullptr;
      frame_->manager_->drawViewport(entry.visibility_bit, *entry.scene_overlay,
                                     host, is_active);
    });
  }
  forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.widget != nullptr) {
      entry.widget->update();
    }
  });
}

void FrameViewport::setViewportRevealHeld(bool held) {
  auto park_opaque = [this](bool force_paint) {
    forEachViewportPanel([force_paint](ViewportPanelEntry& entry) {
      entry.needs_gl_recreate = true;
      if (entry.ogre_viewport != nullptr) {
        entry.ogre_viewport->hideNativeSurface();
      }
      if (entry.host != nullptr) {
        entry.host->setStyleSheet(QStringLiteral(
            "QWidget#AutovizViewportHost { background-color: #303030; }"));
        // Avoid host->update() storms during loadConfig re-enter — paints
        // while docks reshuffle are the main UI flicker source.
        if (force_paint) {
          entry.host->update();
        }
      }
    });
  };

  if (reveal_held_ == held) {
    // loadConfig / mosaic churn may re-enter while already held — re-park so
    // a stray showNativeSurface cannot flash a hole mid-restore.
    if (held) {
      park_opaque(/*force_paint=*/false);
    }
    return;
  }
  reveal_held_ = held;
  if (!reveal_held_) {
    // Do not bare-map GLX here — that flashes X11 holes during loadConfig.
    // Settle recreates each parked pane at final geometry, then presents.
    park_opaque(/*force_paint=*/false);
    scheduleSettleReinstallAll(/*delay_ms=*/32,
                               /*close_broken_duplicates=*/false);
  } else {
    park_opaque(/*force_paint=*/false);
  }
}

void FrameViewport::revealViewportSurfaces() {
  if (reveal_held_) {
    return;
  }
  forEachViewportPanel([this](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport == nullptr || entry.widget == nullptr ||
        entry.host == nullptr || entry.dock == nullptr) {
      return;
    }
    if (!entry.dock->requestedVisible() || !entry.dock->isVisible()) {
      return;
    }
    if (entry.widget->parentWidget() != entry.host) {
      return;
    }
    if (entry.host->width() < 2 || entry.host->height() < 2) {
      return;
    }
    entry.widget->setGeometry(entry.host->rect());
    (void)entry.widget->winId();
    entry.ogre_viewport->showNativeSurface();
    // External GLX ignores Qt layout until windowMovedOrResized — dual panes
    // otherwise keep a stale full-width surface over the sibling.
    entry.ogre_viewport->notifyParentWindowMoved();
    if (entry.hud_overlay != nullptr && entry.hud_overlay->isVisible()) {
      entry.hud_overlay->raise();
    }
  });
}

void FrameViewport::viewportTick(float delta_seconds) {
  syncToolContext();
  forEachViewportPanel([this, delta_seconds](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport == nullptr) {
      return;
    }
    // Startup settle: do not paint until the window/layout stop resizing.
    if (reveal_held_) {
      return;
    }
    // Parked on the frame during mosaic/split reparent — never paint GLX there.
    if (entry.widget != nullptr && entry.widget->parentWidget() == frame_) {
      return;
    }
    // Mosaic title-drag: only the dragged dock stays parked. Sibling panes
    // keep live GL so the mosaic looks unchanged aside from the drop half.
    const bool mosaic_dragging =
        frame_->layout_ != nullptr && frame_->layout_->isCenterDockDragging();
    const bool hosted =
        frame_->layout_ != nullptr &&
        frame_->layout_->main_panel_host_ != nullptr &&
        entry.dock != nullptr &&
        frame_->layout_->main_panel_host_->hostsPanel(entry.dock);
    if (mosaic_dragging) {
      PanelDockWidget* dragged =
          frame_->layout_ != nullptr ? frame_->layout_->mosaicDragDock()
                                     : nullptr;
      if (entry.dock == dragged) {
        entry.ogre_viewport->hideNativeSurface();
        if (entry.widget != nullptr && !entry.widget->isVisible()) {
          entry.widget->show();
          entry.widget->update();
        }
        return;
      }
      // Fall through — remount/resize sibling viewports normally.
    }
    // Hidden 3D docks must not keep painting their native Ogre surface —
    // otherwise Change panel leaves a GL window covering the replacement.
    // After mosaic snap, isVisible() can flicker false while the dock is
    // already hosted — do not unmap in that race (blank pane). Do NOT treat
    // requestedVisible alone as keep_mapped (parking mid-drag).
    if (entry.dock != nullptr && !entry.dock->isVisible()) {
      const bool keep_mapped =
          entry.dock->isFloating() ||
          entry.dock->property("intentionalFloat").toBool() || hosted;
      if (!keep_mapped) {
        entry.ogre_viewport->hideNativeSurface();
        return;
      }
    }
    // Keep each WA_NativeWindow clipped to its host. Splitter moves often
    // leave one GLX at the pre-split full width covering the other pane —
    // or a mapped Ogre child with background_pixel=0 (see-through hole).
    // Destroy the child before resize; paint/tick recreates on next present.
    if (entry.widget != nullptr && entry.host != nullptr &&
        entry.widget->parentWidget() == entry.host &&
        entry.host->width() >= 2 && entry.host->height() >= 2) {
      const QRect host_rect = entry.host->rect();
      if (entry.widget->geometry() != host_rect) {
        entry.widget->setGeometry(host_rect);
        entry.ogre_viewport->invalidateGlSurface();
        entry.ogre_viewport->notifyParentWindowMoved();
      }
    }
    // tick() owns show/hide: never force-show a PaintOnScreen widget here —
    // that reopened X11 see-through holes when present failed.
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
  if (active_entry != nullptr && active_entry->scene_overlay != nullptr) {
    context.scene_overlay = active_entry->scene_overlay.get();
  } else if (frame_->manager_ != nullptr) {
    context.scene_overlay = &frame_->manager_->sceneOverlay();
  }
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
  // Keep the active entry overlay set above — do not reset to the shared
  // manager overlay (that would make tools/draw inherit the wrong pane).
  if (context.scene_overlay == nullptr) {
    context.scene_overlay = &frame_->manager_->sceneOverlay();
  }
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
    if (entry.host == nullptr) {
      return;
    }
    common::Tool* tool = frame_->manager_->tools().toolById(entry.local_tool_id);
    // Shape cursors only on the host widget. Custom pixmap cursors have SEGV'd
    // in QCursor copy during the first mouse move after RenderSystem ready
    // (heap already soft-corrupted); keep pixmap cursors off the QWidget path.
    QCursor cursor = IconLoader::defaultCursor();
    if (entry.local_tool_id == "Interact") {
      cursor = QCursor(Qt::PointingHandCursor);
    } else if (entry.local_tool_id == "Select" ||
               entry.local_tool_id == "Measure" ||
               entry.local_tool_id == "PublishPoint") {
      cursor = QCursor(Qt::CrossCursor);
    }
    if (tool != nullptr) {
      tool->setCursor(cursor);
    }
    entry.host->setCursor(cursor);
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
  if (entry.float_button != nullptr && entry.dock != nullptr) {
    const bool floating =
        entry.dock->isFloating() ||
        entry.dock->property("intentionalFloat").toBool();
    entry.float_button->blockSignals(true);
    entry.float_button->setChecked(floating);
    entry.float_button->setToolTip(floating ? frame_->tr("Dock panel")
                                            : frame_->tr("Float panel"));
    entry.float_button->blockSignals(false);
    if (entry.content_only_button != nullptr) {
      entry.content_only_button->setEnabled(floating);
      if (!floating) {
        entry.content_only_button->blockSignals(true);
        entry.content_only_button->setChecked(false);
        entry.content_only_button->blockSignals(false);
      }
    }
  }
}

void FrameViewport::setViewportFloatButtonChecked(PanelDockWidget* dock,
                                                  bool checked) {
  ViewportPanelEntry* entry = viewportEntryForDock(dock);
  if (entry == nullptr || entry->float_button == nullptr) {
    return;
  }
  entry->float_button->blockSignals(true);
  entry->float_button->setChecked(checked);
  entry->float_button->setToolTip(checked ? frame_->tr("Dock panel")
                                          : frame_->tr("Float panel"));
  entry->float_button->blockSignals(false);
  if (entry->content_only_button != nullptr) {
    entry->content_only_button->setEnabled(checked);
    if (!checked) {
      entry->content_only_button->blockSignals(true);
      entry->content_only_button->setChecked(false);
      entry->content_only_button->blockSignals(false);
    }
  }
}

void FrameViewport::setViewportContentOnlyButtonChecked(PanelDockWidget* dock,
                                                        bool enabled) {
  ViewportPanelEntry* entry = viewportEntryForDock(dock);
  if (entry == nullptr || entry->content_only_button == nullptr) {
    return;
  }
  entry->content_only_button->blockSignals(true);
  entry->content_only_button->setChecked(enabled);
  entry->content_only_button->setToolTip(
      enabled ? frame_->tr("Show title bar")
              : frame_->tr("Content-only (hide chrome)"));
  entry->content_only_button->blockSignals(false);
}

void FrameViewport::pushViewportLocalTool(ViewportPanelEntry& entry) {
  const std::string viewport_key =
      entry.dock != nullptr ? entry.dock->objectName().toStdString()
                            : std::string();
  if (entry.ogre_viewport != nullptr) {
    entry.ogre_viewport->setViewportToolId(entry.local_tool_id);
  }
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





void FrameViewport::installViewportHudOverlay(ViewportPanelEntry& entry) {
  // Glass SPEED/ANG/ODOM dials (WA_AlwaysStackOnTop + translucent) float as a
  // see-through strip over the desktop during mosaic drag. Flat theme keeps
  // the 3D plate clean — tear down any leftover overlay and do not reinstall.
  if (entry.hud_overlay != nullptr) {
    if (entry.layout != nullptr) {
      entry.layout->removeWidget(entry.hud_overlay);
    }
    entry.hud_overlay->setAttribute(Qt::WA_AlwaysStackOnTop, false);
    entry.hud_overlay->setHudVisible(false);
    entry.hud_overlay->hide();
    entry.hud_overlay->deleteLater();
    entry.hud_overlay = nullptr;
  }
  if (entry.host != nullptr) {
    const QList<ViewportHudOverlay*> orphans =
        entry.host->findChildren<ViewportHudOverlay*>();
    for (ViewportHudOverlay* hud : orphans) {
      if (hud == nullptr) {
        continue;
      }
      hud->setAttribute(Qt::WA_AlwaysStackOnTop, false);
      hud->setHudVisible(false);
      hud->hide();
      hud->deleteLater();
    }
  }
  if (entry.layout != nullptr) {
    entry.layout->setContentsMargins(0, 0, 0, 0);
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

  // Free-float + content-only (same path as Image): no mosaic snap while floating.
  entry.float_button = nullptr;
  entry.content_only_button = nullptr;
  if (tools.widget != nullptr) {
    entry.float_button = CreateTitleToolButton(
        tools.widget, IconLoader::panelTitleIcon(QStringLiteral("panel.float")),
        frame_->tr("Float panel"), true);
    const bool floating = dock->isFloating() ||
                          dock->property("intentionalFloat").toBool();
    entry.float_button->setChecked(floating);
    entry.float_button->setToolTip(floating ? frame_->tr("Dock panel")
                                            : frame_->tr("Float panel"));

    entry.content_only_button = CreateTitleToolButton(
        tools.widget,
        IconLoader::panelTitleIcon(QStringLiteral("panel.content_only")),
        frame_->tr("Content-only (hide chrome)"), true);
    entry.content_only_button->setEnabled(floating);
    entry.content_only_button->setChecked(IsContentOnlyFloat(dock));

    if (auto* layout = qobject_cast<QHBoxLayout*>(tools.widget->layout())) {
      int insert_at = layout->count();
      if (entry.expand_button != nullptr) {
        const int expand_index = layout->indexOf(entry.expand_button);
        if (expand_index >= 0) {
          insert_at = expand_index;
        }
      }
      layout->insertWidget(insert_at, entry.float_button);
      layout->insertWidget(insert_at + 1, entry.content_only_button);
    }
    QObject::connect(entry.float_button, &QToolButton::clicked, frame_,
                     [this, dock]() {
                       frame_->layout_->toggleFloatMainPanel(dock);
                     });
    QObject::connect(entry.content_only_button, &QToolButton::clicked, frame_,
                     [this, dock]() {
                       if (dock == nullptr ||
                           !dock->property("intentionalFloat").toBool()) {
                         return;
                       }
                       const QString title =
                           dock->windowTitle().isEmpty()
                               ? frame_->tr("3D View")
                               : dock->windowTitle();
                       frame_->layout_->setContentOnlyFloat(
                           dock, !IsContentOnlyFloat(dock), title);
                     });
    QObject::connect(dock, &QDockWidget::topLevelChanged, frame_,
                     [this, dock](bool floating) {
                       setViewportFloatButtonChecked(
                           dock,
                           floating ||
                               (dock != nullptr &&
                                dock->property("intentionalFloat").toBool()));
                     });
  }

  dock->setTitleBarTools(tools.widget);
  syncViewportTitleBarToolsForEntry(entry);
}

}  // namespace autoviz
