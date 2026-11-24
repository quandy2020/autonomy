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

FrameLayout::FrameLayout(VisualizationFrame* frame) : frame_(frame) {}

void FrameLayout::applyMainPanelDefaultLayout() {
  restoreExpandedMainPanel();
  if (main_panel_host_ == nullptr || frame_->viewport_->viewport_dock_ == nullptr) {
    return;
  }

  suppress_center_tile_ = false;
  center_manual_layout_ = false;

  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr || !isMainPanel(dock) || dock == frame_->viewport_->viewport_dock_) {
      continue;
    }
    dock->hide();
  }

  PanelDockWidget* viewport = frame_->viewport_->viewport_dock_;
  viewport->show();
  if (QAction* toggle = viewport->toggleViewAction()) {
    toggle->blockSignals(true);
    toggle->setChecked(true);
    toggle->blockSignals(false);
  }
  ensureMainPanelDockAttached(viewport);
  last_active_dock_ = viewport;
  tileCenterPanels();
  frame_->viewport_->ensureViewportPanelReady(viewport);
  frame_->viewport_->setActiveViewportDock(viewport);
}

void FrameLayout::configureFlexibleDock(PanelDockWidget* dock) {
  if (dock == nullptr || dock == frame_->panels_->time_dock_) {
    return;
  }
  dock->setAllowedAreas(Qt::AllDockWidgetAreas);
  dock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                    QDockWidget::DockWidgetFloatable);
}

void FrameLayout::restoreExpandedMainPanel() {
  if (expanded_main_panel_dock_ == nullptr &&
      pre_expand_visible_main_panel_docks_.isEmpty() &&
      pre_expand_mosaic_layout_.isEmpty()) {
    return;
  }
  // Block visibilityChanged → removePanel / addPanel / scheduleTile for the
  // whole restore. Early show()+handlers used to append an empty mosaic pane
  // (title-less blank column) beside the restored tree.
  const bool was_suppressing = suppress_center_tile_;
  suppress_center_tile_ = true;
  expanded_main_panel_dock_ = nullptr;

  const QList<QPointer<PanelDockWidget>> restore_docks =
      pre_expand_visible_main_panel_docks_;
  const QByteArray mosaic = pre_expand_mosaic_layout_;
  pre_expand_visible_main_panel_docks_.clear();
  pre_expand_mosaic_layout_.clear();

  bool restored_mosaic = false;
  if (!mosaic.isEmpty() && main_panel_host_ != nullptr) {
    // Prefer exact splitter tree (e.g. 3D | Image/Image) over PreferredGridSize
    // which collapses multi-instance panels that were parked out of discovery.
    center_manual_layout_ = true;
    frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
      frame_->viewport_->parkRenderWindowInEntry(entry);
    });
    restored_mosaic = main_panel_host_->restoreMosaicLayout(
        mosaic,
        [this, &restore_docks](const QString& object_name) -> QDockWidget* {
          for (const QPointer<PanelDockWidget>& dock : restore_docks) {
            if (!dock.isNull() && dock->objectName() == object_name) {
              return dock.data();
            }
          }
          for (PanelDockWidget* dock : orderedDockWidgets()) {
            if (dock != nullptr && dock->objectName() == object_name) {
              return dock;
            }
          }
          return frame_->findChild<PanelDockWidget*>(object_name);
        });
    frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
      frame_->viewport_->reinstallRenderWindowInEntry(entry);
    });
  }
  if (restored_mosaic) {
    // restoreMosaicLayout already showed placed docks; only re-show any leaf
    // that failed to resolve so requestedVisible stays consistent.
    for (const QPointer<PanelDockWidget>& dock : restore_docks) {
      if (dock.isNull() || dock->property("panelDisposed").toBool()) {
        continue;
      }
      if (!main_panel_host_->hostsPanel(dock.data())) {
        dock->show();
        ensureMainPanelDockAttached(dock.data());
      }
    }
    main_panel_host_->scrubEmptyPanes();
    main_panel_host_->syncHorizontalDockLayout();
    syncMainPanelExpandUi(nullptr);
    suppress_center_tile_ = was_suppressing;
  } else {
    for (const QPointer<PanelDockWidget>& dock : restore_docks) {
      if (dock.isNull() || !isMainPanel(dock.data()) ||
          dock->property("panelDisposed").toBool()) {
        continue;
      }
      dock->show();
    }
    suppress_center_tile_ = was_suppressing;
    syncMainPanelExpandUi(nullptr);
    center_manual_layout_ = false;
    tileCenterPanels();
  }
}

void FrameLayout::applyFoxgloveDefaultLayout() {
  if (frame_->panels_->displays_dock_ == nullptr || frame_->panels_->plot_dock_ == nullptr ||
      frame_->viewport_->viewport_dock_ == nullptr) {
    return;
  }

  if (QMainWindow* outer_host = dockHostForPanel(frame_->panels_->displays_dock_)) {
    outer_host->removeDockWidget(frame_->panels_->displays_dock_);
  }
  displays_closed_by_user_ = false;
  frame_->panels_->displays_dock_->show();
  addSidebarDock(frame_->panels_->displays_dock_, Qt::LeftDockWidgetArea);

  applyMainPanelDefaultLayout();
}

void FrameLayout::applyDefaultDockLayout() {
  restoreExpandedMainPanel();

  for (PanelDockWidget* dock : frame_->findChildren<PanelDockWidget*>()) {
    if (dock == nullptr || dock == frame_->viewport_->viewport_dock_ || dock == frame_->panels_->time_dock_ ||
        isMainPanel(dock)) {
      continue;
    }
    if (dock == frame_->panels_->displays_dock_ || dock == frame_->panels_->views_dock_ ||
        dock == frame_->panels_->properties_dock_ ||
        dock == frame_->panels_->record_dock_) {
      continue;
    }
    if (frame_->dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
      frame_->removeDockWidget(dock);
    }
    dock->hide();
  }

  if (frame_->panels_->displays_dock_ != nullptr) {
    if (frame_->dockWidgetArea(frame_->panels_->displays_dock_) != Qt::LeftDockWidgetArea) {
      frame_->removeDockWidget(frame_->panels_->displays_dock_);
      addSidebarDock(frame_->panels_->displays_dock_, Qt::LeftDockWidgetArea);
    }
    displays_closed_by_user_ = false;
    frame_->panels_->displays_dock_->show();
  }

  if (frame_->panels_->properties_dock_ != nullptr) {
    if (frame_->dockWidgetArea(frame_->panels_->properties_dock_) != Qt::LeftDockWidgetArea) {
      frame_->removeDockWidget(frame_->panels_->properties_dock_);
      addSidebarDock(frame_->panels_->properties_dock_, Qt::LeftDockWidgetArea);
    }
    frame_->panels_->properties_dock_->show();
    if (frame_->panels_->displays_dock_ != nullptr) {
      frame_->tabifyDockWidget(frame_->panels_->displays_dock_, frame_->panels_->properties_dock_);
    }
  }

  if (frame_->panels_->displays_dock_ != nullptr) {
    frame_->panels_->displays_dock_->raise();
  }
  left_sidebar_shows_properties_ = false;

  if (frame_->panels_->views_dock_ != nullptr) {
    if (frame_->dockWidgetArea(frame_->panels_->views_dock_) != Qt::RightDockWidgetArea) {
      frame_->removeDockWidget(frame_->panels_->views_dock_);
      addSidebarDock(frame_->panels_->views_dock_, Qt::RightDockWidgetArea);
    }
    frame_->panels_->views_dock_->show();
    frame_->panels_->views_dock_->raise();
  }

  // Keep Record docked on the right but closed — no preview until a record
  // is opened (File → Open Record / drop) or the user toggles Panels → Record.
  if (frame_->panels_->record_dock_ != nullptr) {
    if (frame_->dockWidgetArea(frame_->panels_->record_dock_) !=
        Qt::RightDockWidgetArea) {
      frame_->removeDockWidget(frame_->panels_->record_dock_);
      addSidebarDock(frame_->panels_->record_dock_, Qt::RightDockWidgetArea);
    }
    frame_->panels_->record_dock_->hide();
  }

  applyMainPanelDefaultLayout();
  showPropertyInspector(false);
  ensureTimeDockAtBottom();

  frame_->manager_->setDockHideState(false, false);
  hideLeftDock(false);
  hideRightDock(false);
  frame_->chrome_->syncToolbarLayoutControls();
  frame_->panels_->syncDeletePanelMenu();
  scheduleTileCenterPanels();
}

void FrameLayout::ensureTimeDockAtBottom() {
  if (frame_->panels_->time_dock_ == nullptr) {
    return;
  }
  frame_->panels_->time_dock_->setAllowedAreas(Qt::BottomDockWidgetArea);
  if (frame_->dockWidgetArea(frame_->panels_->time_dock_) != Qt::BottomDockWidgetArea) {
    const bool was_visible = frame_->panels_->time_dock_->isVisible();
    frame_->removeDockWidget(frame_->panels_->time_dock_);
    frame_->addDockWidget(Qt::BottomDockWidgetArea, frame_->panels_->time_dock_);
    if (was_visible) {
      frame_->panels_->time_dock_->show();
    } else {
      frame_->panels_->time_dock_->hide();
    }
  }
  frame_->updateGeometry();
}

void FrameLayout::hideDockImpl(Qt::DockWidgetArea area, bool hide) {
  for (PanelDockWidget* dock : frame_->findChildren<PanelDockWidget*>()) {
    if (frame_->dockWidgetArea(dock) != area) {
      continue;
    }
    dock->setCollapsed(hide);
    if (hide) {
      dock->setAllowedAreas(dock->allowedAreas() & ~area);
    } else {
      dock->setAllowedAreas(dock->allowedAreas() | area);
    }
  }
}

void FrameLayout::hideLeftDock(bool hide) {
  if (frame_->manager_ != nullptr) {
    frame_->manager_->setDockHideState(hide, frame_->manager_->hideRightDock());
  }
  hideDockImpl(Qt::LeftDockWidgetArea, hide);
  syncCenterLayout();
}

void FrameLayout::hideRightDock(bool hide) {
  if (frame_->manager_ != nullptr) {
    frame_->manager_->setDockHideState(frame_->manager_->hideLeftDock(), hide);
  }
  hideDockImpl(Qt::RightDockWidgetArea, hide);
  syncCenterLayout();
}

bool FrameLayout::sidebarAreaHasVisibleDock(Qt::DockWidgetArea area) const {
  if ((area != Qt::LeftDockWidgetArea && area != Qt::RightDockWidgetArea) ||
      frame_ == nullptr || frame_->panels_ == nullptr) {
    return false;
  }
  // Do not use findChildren — mid-teardown / retile leaves dangling QObject*
  // and SEGV's in QMetaObject::cast (mousePress → raiseLeftSidebarDisplays).
  FramePanels* panels = frame_->panels_.get();
  PanelDockWidget* known[] = {
      panels->displays_dock_,      panels->properties_dock_,
      panels->views_dock_,         panels->tool_props_dock_,
      panels->selection_dock_,     panels->tf_dock_,
      panels->channel_graph_dock_, panels->teleop_dock_,
      panels->navigation_dock_,    panels->image_dock_,
      panels->plot_dock_,
      panels->channel_dock_,       panels->channels_dock_,
      panels->record_dock_,
  };
  for (PanelDockWidget* dock : known) {
    if (dock == nullptr || !dock->isVisible() || dock->isFloating() ||
        dock == panels->time_dock_ || isMainPanel(dock)) {
      continue;
    }
    if (frame_->dockWidgetArea(dock) == area) {
      return true;
    }
  }
  return false;
}

void FrameLayout::onHideLeftDockToggled(bool hide) {
  hideLeftDock(hide);
  frame_->chrome_->syncToolbarLayoutControls();
  frame_->session_->markConfigModified();
}

void FrameLayout::onHideRightDockToggled(bool hide) {
  hideRightDock(hide);
  frame_->chrome_->syncToolbarLayoutControls();
  frame_->session_->markConfigModified();
}

void FrameLayout::onDockPanelVisibilityChange(bool visible) {
  auto* dock_widget = qobject_cast<PanelDockWidget*>(frame_->sender());
  // Always keep Panels menu checkboxes in sync — even during center tiling /
  // Split suppress, where we must not schedule another retile.
  if (dock_widget != nullptr) {
    if (QAction* toggle = dock_widget->toggleViewAction()) {
      toggle->blockSignals(true);
      toggle->setChecked(visible);
      toggle->blockSignals(false);
    }
  }
  if (center_tiling_ || center_tile_pending_ || suppress_center_tile_ ||
      center_dock_drag_count_ > 0) {
    return;
  }
  // Mid title-bar drag: Qt flickers visibility / floating. Reattach or retile
  // here removes the dock under the cursor → SIGSEGV.
  if (dock_widget != nullptr &&
      (dock_widget->isTitleDragActive() || center_dock_drag_count_ > 0)) {
    if (!visible) {
      frame_->panels_->unregisterDeletePanelAction(dock_widget);
    }
    return;
  }
  // Floating / free-float panels are intentionally undocked — do not reattach.
  if (dock_widget != nullptr &&
      (dock_widget->isFloating() ||
       dock_widget->property("intentionalFloat").toBool())) {
    if (visible) {
      frame_->panels_->registerDeletePanelAction(dock_widget);
      last_active_dock_ = dock_widget;
    } else {
      frame_->panels_->unregisterDeletePanelAction(dock_widget);
    }
    frame_->chrome_->syncToolbarLayoutControls();
    return;
  }
  if (dock_widget != nullptr) {
    if (visible) {
      frame_->panels_->registerDeletePanelAction(dock_widget);
      last_active_dock_ = dock_widget;
    } else {
      frame_->panels_->unregisterDeletePanelAction(dock_widget);
    }
  }
  if (dock_widget != nullptr && visible &&
      frame_->panels_->panelTypeId(dock_widget) == QLatin1String("ViewportDock")) {
    frame_->viewport_->ensureViewportPanelReady(dock_widget);
  }
  // A side panel shown from the Panels menu clears the matching "hide dock"
  // flag without re-entering hideLeft/RightDock (avoids toggle signal loops).
  if (dock_widget != nullptr && visible && !isMainPanel(dock_widget) &&
      dock_widget != frame_->panels_->time_dock_) {
    // Re-dock before reading area — Teleop must land on the right sidebar.
    ensureSidebarDockAttached(dock_widget);
    dock_widget->raise();
  }
  if (dock_widget != nullptr && visible && frame_->manager_ != nullptr) {
    const Qt::DockWidgetArea area = frame_->dockWidgetArea(dock_widget);
    if (area == Qt::LeftDockWidgetArea && frame_->manager_->hideLeftDock()) {
      frame_->manager_->setDockHideState(false, frame_->manager_->hideRightDock());
    }
    if (area == Qt::RightDockWidgetArea && frame_->manager_->hideRightDock()) {
      frame_->manager_->setDockHideState(frame_->manager_->hideLeftDock(), false);
    }
  }
  if (dock_widget != nullptr && isMainPanel(dock_widget)) {
    if (visible) {
      const bool needs_attach =
          main_panel_host_ == nullptr ||
          !main_panel_host_->hostsPanel(dock_widget);
      if (needs_attach) {
        // Only clear manual layout / retile when the dock is not in the mosaic.
        // Retiling an already-hosted 3D View parks Ogre every time → flicker.
        center_manual_layout_ = false;
        suppress_center_tile_ = false;
        ensureMainPanelDockAttached(dock_widget);
        if (!dock_widget->isVisible()) {
          dock_widget->show();
        }
        scheduleTileCenterPanels();
      }
    } else if (dock_widget->property("panelClosing").toBool()) {
      // Title-bar close: FramePanels::closed defers removePanel out of
      // closeEvent. Do not detach or retile here — sync mosaic teardown under
      // QDockWidget::closeEvent reparents live Ogre windows and SIGSEGVs.
    } else if (main_panel_host_ != nullptr &&
               main_panel_host_->hostsPanel(dock_widget) &&
               !main_panel_host_->isMutatingMosaic()) {
      // Hide via Panels menu: detach that leaf only (no full mosaic rebuild).
      // Skip when mosaic collapse already in progress — visibilityChanged from
      // QSplitter::replaceWidget must not re-enter removePanel (UAF / SEGV).
      // Park viewport GLX first (same hazard as Split / title-bar drag).
      if (frame_->panels_->panelTypeId(dock_widget) ==
          QLatin1String("ViewportDock")) {
        frame_->viewport_->forEachViewportPanel(
            [this](ViewportPanelEntry& entry) {
              frame_->viewport_->parkRenderWindowInEntry(entry);
            });
      }
      main_panel_host_->removePanel(dock_widget);
      main_panel_host_->syncHorizontalDockLayout();
      if (frame_->panels_->panelTypeId(dock_widget) ==
          QLatin1String("ViewportDock")) {
        frame_->viewport_->forEachViewportPanel(
            [this](ViewportPanelEntry& entry) {
              frame_->viewport_->reinstallRenderWindowInEntry(entry);
            });
      }
    } else if (!center_manual_layout_) {
      scheduleTileCenterPanels();
    }
  }
  frame_->chrome_->syncToolbarLayoutControls();
}

void FrameLayout::restoreDockHideState() {
  hideLeftDock(frame_->manager_->hideLeftDock());
  hideRightDock(frame_->manager_->hideRightDock());
  frame_->chrome_->syncToolbarLayoutControls();
}

PanelDockWidget* FrameLayout::activeDockForSplit() const {
  if (last_active_dock_ != nullptr && last_active_dock_->isVisible() &&
      last_active_dock_ != frame_->panels_->time_dock_) {
    return last_active_dock_;
  }
  if (QWidget* focused = QApplication::focusWidget()) {
    for (PanelDockWidget* dock : orderedDockWidgets()) {
      if (dock != nullptr && dock->isVisible() && dock != frame_->panels_->time_dock_ &&
          dock->isAncestorOf(focused)) {
        return dock;
      }
    }
  }
  if (frame_->viewport_->viewport_dock_ != nullptr && frame_->viewport_->viewport_dock_->isVisible()) {
    return frame_->viewport_->viewport_dock_;
  }
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock != nullptr && dock->isVisible() && dock != frame_->panels_->time_dock_) {
      return dock;
    }
  }
  return nullptr;
}

void FrameLayout::onSplitActiveDock(PanelDockWidget* source,
                                         Qt::Orientation orientation) {
  if (source == nullptr || source == frame_->panels_->time_dock_) {
    return;
  }
  if (expanded_main_panel_dock_ != nullptr) {
    restoreExpandedMainPanel();
  }
  last_active_dock_ = source;

  const bool is_viewport = frame_->panels_->panelTypeId(source) == QLatin1String("ViewportDock");
  const bool is_plot = frame_->panels_->panelTypeId(source) == QLatin1String("PlotDock");
  const bool is_image = frame_->panels_->panelTypeId(source) == QLatin1String("ImageDock");
  const bool is_teleop = frame_->panels_->panelTypeId(source) == QLatin1String("TeleopDock");
  const bool is_tf = frame_->panels_->panelTypeId(source) == QLatin1String("TfTreeDock");
  const bool is_publish = frame_->panels_->panelTypeId(source) == QLatin1String("PublishDock");
  const bool is_map = frame_->panels_->panelTypeId(source) == QLatin1String("MapDock");
  const bool is_channel_graph =
      frame_->panels_->panelTypeId(source) == QLatin1String("ChannelGraphDock");

  // Snapshot camera before park — source Ogre is detached across splitPanel.
  std::optional<rendering::ViewState> split_view_state;
  if (is_viewport) {
    if (ViewportPanelEntry* src_entry =
            frame_->viewport_->viewportEntryForDock(source)) {
      if (rendering::ViewController* src_vc = src_entry->viewController()) {
        split_view_state = src_vc->state();
      }
    }
  }

  PanelDockWidget* duplicate = frame_->panels_->duplicatePanelDock(source);
  if (duplicate == nullptr) {
    QMessageBox::information(
        frame_, frame_->tr("Split Panel"),
        frame_->tr("This panel type cannot be duplicated yet."));
    return;
  }

  QMainWindow* host = dockHostForPanel(source);
  if (host == nullptr) {
    return;
  }

  // Directed split must not be overwritten by the auto grid tiler / equalize.
  suppress_center_tile_ = true;
  center_manual_layout_ = true;
  ++center_tile_epoch_;
  center_tile_pending_ = false;

  if (source->isFloating()) {
    source->setFloating(false);
  }
  if (!main_panel_host_->hostsPanel(source) && isMainPanel(source)) {
    configureMainPanelDock(source);
    main_panel_host_->addPanel(source);
  } else if (host->dockWidgetArea(source) == Qt::NoDockWidgetArea &&
             !isMainPanel(source)) {
    host->addDockWidget(Qt::LeftDockWidgetArea, source);
  }
  source->blockSignals(true);
  source->show();
  source->blockSignals(false);
  source->raise();

  if (host->dockWidgetArea(duplicate) != Qt::NoDockWidgetArea) {
    host->removeDockWidget(duplicate);
  }
  // Drop outer-frame parenting so the center host can own the duplicate.
  if (frame_->dockWidgetArea(duplicate) != Qt::NoDockWidgetArea) {
    frame_->removeDockWidget(duplicate);
  }
  if (isMainPanel(source)) {
    configureMainPanelDock(duplicate);
    wireMainPanelExpandTracking(duplicate);
  }
  duplicate->setFloating(false);
  duplicate->blockSignals(true);
  // Park live Ogre/GLX before splitter reparent (replaceWidget / WrapDockInPane).
  // Same hazard as tileCenterPanels / session mosaic restore.
  if (is_viewport) {
    if (frame_->panels_ != nullptr &&
        frame_->panels_->views_panel_ != nullptr) {
      frame_->panels_->views_panel_->setViewController(nullptr);
    }
    frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
      frame_->viewport_->parkRenderWindowInEntry(entry);
    });
  }
  if (isMainPanel(source) && main_panel_host_ != nullptr) {
    main_panel_host_->splitPanel(source, duplicate, orientation);
  } else {
    host->splitDockWidget(source, duplicate, orientation);
    if (host->dockWidgetArea(duplicate) == Qt::NoDockWidgetArea) {
      host->addDockWidget(host->dockWidgetArea(source) == Qt::NoDockWidgetArea
                              ? Qt::LeftDockWidgetArea
                              : host->dockWidgetArea(source),
                          duplicate);
      host->splitDockWidget(source, duplicate, orientation);
    }
  }
  duplicate->show();
  duplicate->blockSignals(false);
  duplicate->raise();

  if (is_viewport) {
    // Pause paint while GLX is parked / second window is created — a concurrent
    // render tick during reparent is another common Split SIGSEGV path.
    const bool timer_was_active = frame_->session_->render_timer_.isActive();
    frame_->session_->render_timer_.stop();

    // Wait for QSplitter pane geometry, then settle-recreate both panes
    // (same pipeline as mosaic drag-snap — Foxglove: layout first, GL resize).
    const QPointer<PanelDockWidget> source_gl(source);
    const QPointer<PanelDockWidget> duplicate_gl(duplicate);
    const std::optional<rendering::ViewState> view_copy = split_view_state;
    auto finish = std::make_shared<std::function<void(int)>>();
    *finish = [this, source_gl, duplicate_gl, view_copy, timer_was_active,
               finish](int attempt) {
      if (!duplicate_gl) {
        suppress_center_tile_ = false;
        if (timer_was_active && !frame_->session_->app_inactive_) {
          frame_->session_->render_timer_.start();
        }
        return;
      }

      if (main_panel_host_ != nullptr) {
        main_panel_host_->syncHorizontalDockLayout();
      }
      duplicate_gl->show();
      duplicate_gl->raise();
      if (source_gl) {
        source_gl->show();
      }

      int host_w = duplicate_gl->width();
      int host_h = duplicate_gl->height();
      if (ViewportPanelEntry* dup_entry =
              frame_->viewport_->viewportEntryForDock(duplicate_gl.data())) {
        if (dup_entry->host != nullptr) {
          host_w = qMax(host_w, dup_entry->host->width());
          host_h = qMax(host_h, dup_entry->host->height());
        }
      }
      if ((host_w < 80 || host_h < 60) && attempt < 12) {
        QTimer::singleShot(16, frame_, [finish, attempt]() {
          (*finish)(attempt + 1);
        });
        return;
      }

      if (source_gl) {
        frame_->viewport_->ensureViewportPanelReady(source_gl.data());
      }
      frame_->viewport_->ensureViewportPanelReady(duplicate_gl.data());
      if (view_copy.has_value()) {
        if (ViewportPanelEntry* dst_entry =
                frame_->viewport_->viewportEntryForDock(duplicate_gl.data())) {
          if (rendering::ViewController* dst_vc = dst_entry->viewController()) {
            dst_vc->setState(*view_copy);
          }
        }
      }
      frame_->viewport_->setActiveViewportDock(duplicate_gl.data());
      frame_->chrome_->rebuildPanelsMenuToggles();
      suppress_center_tile_ = false;
      frame_->viewport_->scheduleSettleReinstallAll(
          48, /*close_broken_duplicates=*/false);
      frame_->viewport_->requestViewportUpdate();
      if (timer_was_active && !frame_->session_->app_inactive_) {
        frame_->session_->render_timer_.start();
      }
    };
    QTimer::singleShot(0, frame_, [finish]() { (*finish)(0); });
  }

  if (QAction* toggle = duplicate->toggleViewAction()) {
    toggle->blockSignals(true);
    toggle->setChecked(true);
    toggle->blockSignals(false);
  }
  if (QAction* toggle = source->toggleViewAction()) {
    toggle->blockSignals(true);
    toggle->setChecked(true);
    toggle->blockSignals(false);
  }

  last_active_dock_ = duplicate;

  // Resize after the dock layout commits — immediate resizeDocks often leaves
  // the new pane clipped / offset (especially Split right + QOpenGLWidget).
  const QPointer<QMainWindow> host_guard(host);
  const QPointer<PanelDockWidget> source_guard(source);
  const QPointer<PanelDockWidget> duplicate_guard(duplicate);
  QTimer::singleShot(0, frame_, [this, host_guard, source_guard, duplicate_guard,
                               orientation]() {
    if (host_guard && source_guard && duplicate_guard) {
      // Relax mins so stacked Split downs are not blocked by content mins.
      for (PanelDockWidget* dock :
           {source_guard.data(), duplicate_guard.data()}) {
        if (dock == nullptr) {
          continue;
        }
        dock->setMinimumSize(60, 40);
        if (QWidget* content = dock->widget()) {
          content->setMinimumSize(60, 40);
        }
      }

      // Center host uses QSplitter — refill after geometry commits so a stale
      // pane width from splitPanel cannot leave a gap between neighbours.
      const bool center_split =
          host_guard.data() == static_cast<QMainWindow*>(main_panel_host_) ||
          (main_panel_host_ != nullptr &&
           main_panel_host_->hostsPanel(source_guard.data()));
      if (center_split && main_panel_host_ != nullptr) {
        main_panel_host_->syncHorizontalDockLayout();
      } else if (!center_split && orientation == Qt::Vertical) {
        // Equalize the whole column that contains the split (not just the pair),
        // otherwise the 3rd/4th Split down overflows and clips neighbors.
        QList<QDockWidget*> column;
        const QRect src_geo = source_guard->geometry();
        for (QDockWidget* dock : host_guard->findChildren<QDockWidget*>()) {
          if (dock == nullptr || dock->isFloating() || !dock->isVisible()) {
            continue;
          }
          if (host_guard->dockWidgetArea(dock) == Qt::NoDockWidgetArea) {
            continue;
          }
          const QRect geo = dock->geometry();
          const int overlap = std::min(src_geo.right(), geo.right()) -
                              std::max(src_geo.left(), geo.left());
          if (overlap > src_geo.width() / 2) {
            dock->setMinimumHeight(40);
            if (QWidget* content = dock->widget()) {
              content->setMinimumHeight(40);
            }
            column.push_back(dock);
          }
        }
        std::sort(column.begin(), column.end(),
                  [](const QDockWidget* a, const QDockWidget* b) {
                    return a->y() < b->y();
                  });
        if (column.size() >= 2) {
          const int each =
              qMax(1, host_guard->height() / static_cast<int>(column.size()));
          const int pane_h = qMax(each, 40);
          QList<int> heights;
          heights.reserve(column.size());
          for (int i = 0; i < column.size(); ++i) {
            heights.push_back(pane_h);
          }
          host_guard->resizeDocks(column, heights, Qt::Vertical);
        }
      } else if (!center_split) {
        QList<QDockWidget*> row;
        const QRect src_geo = source_guard->geometry();
        for (QDockWidget* dock : host_guard->findChildren<QDockWidget*>()) {
          if (dock == nullptr || dock->isFloating() || !dock->isVisible()) {
            continue;
          }
          if (host_guard->dockWidgetArea(dock) == Qt::NoDockWidgetArea) {
            continue;
          }
          const QRect geo = dock->geometry();
          const int overlap = std::min(src_geo.bottom(), geo.bottom()) -
                              std::max(src_geo.top(), geo.top());
          if (overlap > src_geo.height() / 2) {
            dock->setMinimumWidth(60);
            if (QWidget* content = dock->widget()) {
              content->setMinimumWidth(60);
            }
            row.push_back(dock);
          }
        }
        std::sort(row.begin(), row.end(),
                  [](const QDockWidget* a, const QDockWidget* b) {
                    return a->x() < b->x();
                  });
        if (row.size() >= 2) {
          const int each =
              qMax(1, host_guard->width() / static_cast<int>(row.size()));
          const int pane_w = qMax(each, 60);
          QList<int> widths;
          widths.reserve(row.size());
          for (int i = 0; i < row.size(); ++i) {
            widths.push_back(pane_w);
          }
          host_guard->resizeDocks(row, widths, Qt::Horizontal);
        }
      }
    }
    // Viewport split keeps suppress_center_tile_ until GL settle finish above.
    const bool viewport_pair =
        frame_->panels_->panelTypeId(source_guard.data()) ==
        QLatin1String("ViewportDock");
    if (!viewport_pair) {
      suppress_center_tile_ = false;
    }
    syncCenterLayout();
    ensureTimeDockAtBottom();
    if (QStatusBar* status = frame_->statusBar()) {
      status->setVisible(true);
      status->raise();
    }
    for (PanelDockWidget* dock : {source_guard.data(), duplicate_guard.data()}) {
      if (dock == nullptr) {
        continue;
      }
      if (ViewportPanelEntry* entry =
              frame_->viewport_->viewportEntryForDock(dock)) {
        if (!viewport_pair) {
          frame_->viewport_->ensureViewportPanelReady(dock);
        }
        if (entry->host != nullptr) {
          entry->host->updateGeometry();
        }
        if (entry->widget != nullptr) {
          entry->widget->updateGeometry();
          entry->widget->update();
        }
      }
      dock->updateGeometry();
      dock->update();
    }
    if (main_panel_host_ != nullptr) {
      main_panel_host_->updateGeometry();
      main_panel_host_->update();
    }
  });

  if (is_viewport) {
    frame_->viewport_->setActiveViewportDock(duplicate);
  } else if (is_plot) {
    if (auto* panel = qobject_cast<plot::PlotPanel*>(duplicate->widget())) {
      frame_->panels_->setActivePlotPanel(panel);
    }
  } else if (is_image) {
    if (auto* panel = qobject_cast<image::ImagePanel*>(duplicate->widget())) {
      frame_->panels_->setActiveImagePanel(panel);
    }
  } else if (is_teleop) {
    if (auto* panel = qobject_cast<teleop::TeleopPanel*>(duplicate->widget())) {
      frame_->panels_->setActiveTeleopPanel(panel);
    }
  } else if (is_tf) {
    if (auto* panel = qobject_cast<TfTreePanel*>(duplicate->widget())) {
      panel->refresh();
    }
  } else if (is_channel_graph) {
    if (auto* panel =
            qobject_cast<channel_graph::ChannelGraphPanel*>(duplicate->widget())) {
      panel->refreshGraph();
    }
  } else if (is_publish) {
    if (auto* panel = qobject_cast<publish_panel::PublishPanel*>(duplicate->widget())) {
      frame_->panels_->setActivePublishPanel(panel);
    }
  } else if (is_map) {
    if (auto* panel = qobject_cast<map::MapPanel*>(duplicate->widget())) {
      frame_->panels_->setActiveMapPanel(panel);
    }
  }

  frame_->panels_->syncDeletePanelMenu();
  frame_->session_->markConfigModified();
}

QList<PanelDockWidget*> FrameLayout::orderedDockWidgets() const {
  QList<PanelDockWidget*> docks = {frame_->viewport_->viewport_dock_,   frame_->panels_->displays_dock_,
                                   frame_->panels_->record_dock_,
                                   frame_->panels_->selection_dock_,  frame_->panels_->tool_props_dock_, frame_->panels_->views_dock_,
                                   frame_->panels_->time_dock_,       frame_->panels_->channel_dock_,
                                   frame_->panels_->channels_dock_,
                                   frame_->panels_->tf_dock_, frame_->panels_->image_dock_,
                                   frame_->panels_->plot_dock_,       frame_->panels_->teleop_dock_,
                                   frame_->panels_->navigation_dock_,
                                   frame_->panels_->channel_graph_dock_};
  docks.removeAll(nullptr);
  const auto add = [&](PanelDockWidget* dock) {
    if (dock != nullptr && !docks.contains(dock)) {
      docks.push_back(dock);
    }
  };
  // Prefer mosaic + direct children over recursive findChildren — the latter
  // has SEGV'd in QMetaObject::cast when a dock QObject edge was dangling.
  if (main_panel_host_ != nullptr) {
    for (QDockWidget* dock : main_panel_host_->hostedPanels()) {
      add(qobject_cast<PanelDockWidget*>(dock));
    }
    // Parked multi-instance panels (2nd Image, Split View, …) are parented to
    // MainPanelHost::parking_ — not frame_ children — and must stay discoverable
    // so expand/restore / tileCenterPanels can bring them back.
    for (QDockWidget* dock : main_panel_host_->parkedPanels()) {
      add(qobject_cast<PanelDockWidget*>(dock));
    }
  }
  for (QObject* child : frame_->children()) {
    add(qobject_cast<PanelDockWidget*>(child));
  }
  return docks;
}

}  // namespace autoviz
