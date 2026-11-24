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


}  // namespace autoviz
