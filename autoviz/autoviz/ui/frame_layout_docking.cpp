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

void FrameLayout::setupCentralContainer() {
  main_panel_host_ = new MainPanelHost(frame_);
  main_panel_host_->setMinimumSize(0, 0);
  main_panel_host_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  frame_->setCentralWidget(main_panel_host_);
}

void FrameLayout::setupMainPanelHost() {}

QMainWindow* FrameLayout::dockHostForPanel(
    const PanelDockWidget* dock) const {
  if (dock != nullptr && isMainPanel(dock)) {
    return main_panel_host_;
  }
  return const_cast<VisualizationFrame*>(frame_);
}

bool FrameLayout::isMainPanel(const PanelDockWidget* dock) const {
  if (dock == nullptr) {
    return false;
  }
  return dock->property("panelRole").toString() == PanelRoleMain();
}

void FrameLayout::configureMainPanelDock(PanelDockWidget* dock) {
  if (dock == nullptr || dock == frame_->panels_->time_dock_) {
    return;
  }
  dock->setProperty("panelRole", PanelRoleMain());
  // Center panels live in MainPanelHost's QSplitter. Movable/Floatable make
  // QDockWidget steal mouse events near edges and block splitter resizing.
  // Title-bar undock is handled manually in PanelDockWidget.
  dock->setAllowedAreas(Qt::NoDockWidgetArea);
  dock->setFeatures(QDockWidget::DockWidgetClosable);
}

bool FrameLayout::isMainPanelPinnedToCenter(
    const PanelDockWidget* dock) const {
  if (dock == nullptr) {
    return false;
  }
  const QString type = frame_->panels_->panelTypeId(dock);
  return type == QLatin1String("ImageDock") ||
         type == QLatin1String("ImageDisplayDock") ||
         type == QLatin1String("PlotDock") ||
         type == QLatin1String("TableDock") ||
         type == QLatin1String("ChannelBrowserDock") ||
         type == QLatin1String("ChannelsDock") ||
         type == QLatin1String("TfTreeDock") ||
         type == QLatin1String("ChannelGraphDock") ||
         type == QLatin1String("EvidenceDock");
}

bool FrameLayout::isAnyMainPanelFloating() const {
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock != nullptr && isMainPanel(dock) && dock->isFloating()) {
      return true;
    }
  }
  return false;
}

bool FrameLayout::isCenterDockInteractionBlocked() const {
  // Only block while a title-bar drag is active — floating panels must remain
  // allowed after the user intentionally undocks them.
  return center_dock_drag_count_ > 0 || suppress_center_tile_ ||
         center_tiling_ || center_tile_pending_;
}

void FrameLayout::configureSidebarDock(PanelDockWidget* dock,
                                              Qt::DockWidgetArea area) {
  if (dock == nullptr || dock == frame_->panels_->time_dock_) {
    return;
  }
  dock->setProperty("panelRole", PanelRoleSidebar());
  // Teleop / Navigation: default right, but allow drag/snap to either sidebar.
  if (frame_->panels_->panelTypeId(dock) == QLatin1String("TeleopDock") ||
      frame_->panels_->panelTypeId(dock) == QLatin1String("NavigationDock")) {
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  } else {
    dock->setAllowedAreas(area);
  }
  dock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                    QDockWidget::DockWidgetFloatable);
}

void FrameLayout::addMainPanelDock(PanelDockWidget* dock,
                                          Qt::DockWidgetArea /*area*/) {
  if (dock == nullptr || main_panel_host_ == nullptr) {
    return;
  }
  configureMainPanelDock(dock);
  if (frame_->dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
    frame_->removeDockWidget(dock);
  }
  if (main_panel_host_->dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
    main_panel_host_->removeDockWidget(dock);
  }
  main_panel_host_->addPanel(dock);
  dock->setFloating(false);
  wireMainPanelExpandTracking(dock);
}

void FrameLayout::ensureMainPanelDockAttached(
    PanelDockWidget* dock, Qt::DockWidgetArea /*area*/) {
  if (dock == nullptr || main_panel_host_ == nullptr || !isMainPanel(dock)) {
    return;
  }
  // Reentrancy: configureMainPanelDock → setFeatures(no Floatable) can call
  // setFloating(false) → visibilityChanged → frame_ function. Guard the entire
  // body including the already-hosted early path.
  if (ensuring_main_panel_attach_) {
    return;
  }
  // Never reparent mid title-bar drag / intentional float.
  if (dock->isFloating() || dock->property("intentionalFloat").toBool() ||
      dock->isTitleDragActive() || center_dock_drag_count_ > 0) {
    return;
  }

  ensuring_main_panel_attach_ = true;
  const QSignalBlocker visibility_blocker(dock);

  // Already in the center splitter — only refresh features if still wrong.
  if (main_panel_host_->hostsPanel(dock)) {
    if (dock->features() != QDockWidget::DockWidgetClosable ||
        dock->allowedAreas() != Qt::NoDockWidgetArea) {
      configureMainPanelDock(dock);
    }
    ensuring_main_panel_attach_ = false;
    return;
  }

  if (frame_->dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
    frame_->removeDockWidget(dock);
  }
  if (main_panel_host_->dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
    main_panel_host_->removeDockWidget(dock);
  }
  configureMainPanelDock(dock);
  if (dock->isFloating()) {
    dock->setFloating(false);
  }
  main_panel_host_->addPanel(dock);
  if (dock->isCollapsed()) {
    dock->setCollapsed(false);
  }
  ensuring_main_panel_attach_ = false;
}

Qt::DockWidgetArea FrameLayout::defaultSidebarArea(
    const PanelDockWidget* dock) const {
  if (dock == nullptr) {
    return Qt::RightDockWidgetArea;
  }
  if (dock == frame_->panels_->displays_dock_ ||
      dock == frame_->panels_->properties_dock_) {
    return Qt::LeftDockWidgetArea;
  }
  // Record / Teleop default to the right sidebar (Views / Selection peers).
  if (dock == frame_->panels_->record_dock_ ||
      frame_->panels_->panelTypeId(dock) == QLatin1String("RecordDock") ||
      frame_->panels_->panelTypeId(dock) == QLatin1String("TeleopDock") ||
      frame_->panels_->panelTypeId(dock) == QLatin1String("NavigationDock")) {
    return Qt::RightDockWidgetArea;
  }
  return Qt::RightDockWidgetArea;
}

void FrameLayout::ensureSidebarDockAttached(PanelDockWidget* dock) {
  if (dock == nullptr || isMainPanel(dock)) {
    return;
  }
  // A dock restored from the session mosaic lives in MainPanelHost; re-docking
  // it as a sidebar destroys the restored split tree (panes vanish, the
  // default layout reappears). Mosaic ownership wins over catalog role.
  if (main_panel_host_ != nullptr && main_panel_host_->hostsPanel(dock)) {
    return;
  }
  const bool is_teleop = frame_->panels_->panelTypeId(dock) == QLatin1String("TeleopDock");
  const bool is_navigation =
      frame_->panels_->panelTypeId(dock) == QLatin1String("NavigationDock");
  Qt::DockWidgetArea area = frame_->dockWidgetArea(dock);
  // Teleop / Navigation always open on the right (Views / Selection peers).
  // Saved window state or a prior left-side session must not keep them left.
  if (is_teleop || is_navigation) {
    area = Qt::RightDockWidgetArea;
  } else if (area == Qt::NoDockWidgetArea) {
    area = defaultSidebarArea(dock);
  }
  configureSidebarDock(dock, area);
  if (is_teleop) {
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  } else {
    dock->setAllowedAreas(area);
  }
  if (dock->isFloating()) {
    dock->setFloating(false);
  }
  if (frame_->dockWidgetArea(dock) != area) {
    if (frame_->dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
      frame_->removeDockWidget(dock);
    }
    frame_->addDockWidget(area, dock);
  }
  dock->setCollapsed(false);

  if (area == Qt::LeftDockWidgetArea &&
      frame_->chrome_->toolbar_toggle_left_dock_action_ != nullptr &&
      !frame_->chrome_->toolbar_toggle_left_dock_action_->isChecked()) {
    frame_->chrome_->toolbar_toggle_left_dock_action_->setChecked(true);
  }
  if (area == Qt::RightDockWidgetArea &&
      frame_->chrome_->toolbar_toggle_right_dock_action_ != nullptr &&
      !frame_->chrome_->toolbar_toggle_right_dock_action_->isChecked()) {
    frame_->chrome_->toolbar_toggle_right_dock_action_->setChecked(true);
  }
}

void FrameLayout::syncCenterLayout() {
  if (main_panel_host_ == nullptr) {
    return;
  }
  if (QWidget* central = frame_->centralWidget()) {
    central->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    central->setMinimumSize(0, 0);
    central->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
  }
  main_panel_host_->setMinimumSize(0, 0);
  main_panel_host_->updateGeometry();
  frame_->updateGeometry();
}

void FrameLayout::scheduleTileCenterPanels() {
  // Session mosaic / directed Split own the splitter tree — never overwrite.
  if (main_panel_host_ == nullptr || isCenterDockInteractionBlocked() ||
      center_manual_layout_) {
    return;
  }
  center_tile_pending_ = true;
  const int epoch = center_tile_epoch_;
  QTimer::singleShot(0, frame_, [this, epoch]() {
    center_tile_pending_ = false;
    if (epoch != center_tile_epoch_ || center_tiling_ ||
        suppress_center_tile_ || center_dock_drag_count_ > 0 ||
        center_manual_layout_) {
      return;
    }
    tileCenterPanels();
  });
}

void FrameLayout::tileCenterPanels() {
  if (main_panel_host_ == nullptr || center_tiling_) {
    return;
  }
  center_tiling_ = true;
  center_manual_layout_ = false;

  QList<QDockWidget*> visible;
  QList<QDockWidget*> hidden;
  visible.reserve(8);
  hidden.reserve(8);

  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr || !isMainPanel(dock) ||
        dock->property("panelDisposed").toBool() || dock->isFloating() ||
        dock->property("intentionalFloat").toBool()) {
      continue;
    }
    // Prefer requestedVisible(): before the frame is shown, isVisible() is
    // always false and would park every dock → empty white center.
    if (dock->requestedVisible()) {
      visible.push_back(dock);
    } else {
      hidden.push_back(dock);
    }
  }

  // Allow an empty center column (user may close every 3D View / main panel).
  // Do not force-show ViewportDock — that made the last 3D View unclosable.

  // Skip destructive retile when the mosaic already hosts exactly the visible
  // set. Full tile parks/reinstalls Ogre and causes perpetual 3D View flicker.
  if (!visible.isEmpty()) {
    bool mosaic_ok = true;
    for (QDockWidget* dock : visible) {
      if (!main_panel_host_->hostsPanel(dock)) {
        mosaic_ok = false;
        break;
      }
    }
    if (mosaic_ok) {
      for (QDockWidget* dock : hidden) {
        if (dock != nullptr && main_panel_host_->hostsPanel(dock)) {
          mosaic_ok = false;
          break;
        }
      }
    }
    if (mosaic_ok) {
      main_panel_host_->syncHorizontalDockLayout();
      // Mosaic already correct — still reinstall any parked Ogre surface so a
      // 3D View that skipped a full retile is not left blank (toolbar only).
      for (QDockWidget* dock : visible) {
        auto* panel_dock = qobject_cast<PanelDockWidget*>(dock);
        if (panel_dock != nullptr &&
            frame_->panels_->panelTypeId(panel_dock) ==
                QLatin1String("ViewportDock")) {
          frame_->viewport_->ensureViewportPanelReady(panel_dock);
        }
      }
      for (QDockWidget* dock : visible + hidden) {
        if (dock == nullptr) {
          continue;
        }
        if (QAction* toggle = dock->toggleViewAction()) {
          auto* panel = qobject_cast<PanelDockWidget*>(dock);
          toggle->blockSignals(true);
          toggle->setChecked(panel != nullptr ? panel->requestedVisible()
                                              : dock->isVisible());
          toggle->blockSignals(false);
        }
      }
      center_tiling_ = false;
      return;
    }
  }

  for (QDockWidget* dock : visible + hidden) {
    if (dock != nullptr) {
      dock->blockSignals(true);
    }
  }
  // Park Ogre/GL before splitter reparent. Panels→check 3D View hits this path
  // and used to reinstall with rebind-only — every pane went transparent.
  // Mark recreate + deferred settle so each visible 3D View gets a fresh GLX.
  frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
    frame_->viewport_->parkRenderWindowInEntry(entry);
  });
  main_panel_host_->tilePanels(visible, hidden);
  for (QDockWidget* dock : visible) {
    auto* panel_dock = qobject_cast<PanelDockWidget*>(dock);
    if (panel_dock != nullptr &&
        frame_->panels_->panelTypeId(panel_dock) ==
            QLatin1String("ViewportDock")) {
      frame_->viewport_->ensureViewportPanelReady(panel_dock);
    }
  }
  for (QDockWidget* dock : visible + hidden) {
    if (dock != nullptr) {
      dock->blockSignals(false);
    }
  }
  // Recreate GL widgets after mosaic geometry lands (not immediate rebind).
  frame_->viewport_->scheduleSettleReinstallAll(48);

  // Keep Panels menu checkboxes aligned with requested dock visibility.
  for (QDockWidget* dock : visible + hidden) {
    if (dock == nullptr) {
      continue;
    }
    if (QAction* toggle = dock->toggleViewAction()) {
      auto* panel = qobject_cast<PanelDockWidget*>(dock);
      toggle->blockSignals(true);
      toggle->setChecked(panel != nullptr ? panel->requestedVisible()
                                          : dock->isVisible());
      toggle->blockSignals(false);
    }
  }

  center_tiling_ = false;
}

void FrameLayout::addSidebarDock(PanelDockWidget* dock,
                                        Qt::DockWidgetArea area) {
  if (dock == nullptr) {
    return;
  }
  configureSidebarDock(dock, area);
  frame_->addDockWidget(area, dock);
}

bool FrameLayout::isPropertyInspectorVisible() const {
  return left_sidebar_shows_properties_ && frame_->panels_->properties_dock_ != nullptr &&
         frame_->panels_->properties_dock_->isVisible();
}

void FrameLayout::raiseLeftSidebarDisplays() {
  left_sidebar_shows_properties_ = false;
  // Viewport activation (3D View click) only switches the left sidebar back to
  // Displays when it is already meant to be shown (e.g. leaving Properties).
  // After the user closed Displays or used Hide Left, do not force it open —
  // reopen only via Panels menu / toolbar toggle.
  const bool left_hidden =
      frame_->manager_ != nullptr && frame_->manager_->hideLeftDock();
  if (!displays_closed_by_user_ && !left_hidden &&
      frame_->panels_->displays_dock_ != nullptr) {
    ensureSidebarDockAttached(frame_->panels_->displays_dock_);
    frame_->panels_->displays_dock_->setCollapsed(false);
    frame_->panels_->displays_dock_->show();
    frame_->panels_->displays_dock_->raise();
    if (QAction* toggle = frame_->panels_->displays_dock_->toggleViewAction()) {
      toggle->blockSignals(true);
      toggle->setChecked(true);
      toggle->blockSignals(false);
    }
    frame_->chrome_->syncToolbarLayoutControls();
  }
  if (frame_->panels_->active_plot_panel_ != nullptr) {
    frame_->panels_->active_plot_panel_->setSettingsButtonChecked(false);
  }
  if (frame_->panels_->active_image_panel_ != nullptr) {
    frame_->panels_->active_image_panel_->setSettingsButtonChecked(false);
  }
  if (frame_->panels_->active_teleop_panel_ != nullptr) {
    frame_->panels_->active_teleop_panel_->setSettingsButtonChecked(false);
  }
  if (frame_->panels_->active_publish_panel_ != nullptr) {
    frame_->panels_->active_publish_panel_->setSettingsButtonChecked(false);
  }
  if (frame_->panels_->active_service_panel_ != nullptr) {
    frame_->panels_->active_service_panel_->setSettingsButtonChecked(false);
  }
  if (frame_->panels_->active_map_panel_ != nullptr) {
    frame_->panels_->active_map_panel_->setSettingsButtonChecked(false);
  }
  if (frame_->manager_ != nullptr) {
    frame_->manager_->setPlotSettingsVisible(false);
  }
}

void FrameLayout::raiseLeftSidebarProperties() {
  left_sidebar_shows_properties_ = true;
  if (frame_->panels_->properties_dock_ != nullptr) {
    frame_->panels_->properties_dock_->show();
    frame_->panels_->properties_dock_->raise();
  }
}

void FrameLayout::showPropertyInspector(bool visible) {
  if (visible) {
    if (frame_->panels_->inspector_plot_panel_ != nullptr) {
      frame_->panels_->bindPlotToPropertyInspector(frame_->panels_->inspector_plot_panel_);
    } else if (frame_->panels_->inspector_image_panel_ != nullptr) {
      frame_->panels_->bindImageToPropertyInspector(frame_->panels_->inspector_image_panel_);
    } else if (frame_->panels_->inspector_teleop_panel_ != nullptr) {
      frame_->panels_->bindTeleopToPropertyInspector(frame_->panels_->inspector_teleop_panel_);
    } else if (frame_->panels_->active_teleop_panel_ != nullptr) {
      frame_->panels_->bindTeleopToPropertyInspector(frame_->panels_->active_teleop_panel_);
    } else if (frame_->panels_->active_image_panel_ != nullptr) {
      frame_->panels_->bindImageToPropertyInspector(frame_->panels_->active_image_panel_);
    } else if (frame_->panels_->active_plot_panel_ != nullptr) {
      frame_->panels_->bindPlotToPropertyInspector(frame_->panels_->active_plot_panel_);
    } else if (frame_->panels_->active_publish_panel_ != nullptr) {
      frame_->panels_->bindPublishToPropertyInspector(frame_->panels_->active_publish_panel_);
    } else if (frame_->panels_->active_service_panel_ != nullptr) {
      frame_->panels_->bindServiceToPropertyInspector(frame_->panels_->active_service_panel_);
    } else if (frame_->panels_->active_map_panel_ != nullptr) {
      frame_->panels_->bindMapToPropertyInspector(frame_->panels_->active_map_panel_);
    }
    raiseLeftSidebarProperties();
  } else {
    raiseLeftSidebarDisplays();
  }
  if (frame_->panels_->active_plot_panel_ != nullptr) {
    frame_->panels_->active_plot_panel_->setSettingsButtonChecked(visible);
  }
  if (frame_->panels_->active_image_panel_ != nullptr) {
    frame_->panels_->active_image_panel_->setSettingsButtonChecked(visible);
  }
  if (frame_->panels_->active_teleop_panel_ != nullptr) {
    frame_->panels_->active_teleop_panel_->setSettingsButtonChecked(visible);
  }
  if (frame_->panels_->active_publish_panel_ != nullptr) {
    frame_->panels_->active_publish_panel_->setSettingsButtonChecked(visible);
  }
  if (frame_->panels_->active_service_panel_ != nullptr) {
    frame_->panels_->active_service_panel_->setSettingsButtonChecked(visible);
  }
  if (frame_->panels_->active_map_panel_ != nullptr) {
    frame_->panels_->active_map_panel_->setSettingsButtonChecked(visible);
  }
  if (frame_->manager_ != nullptr) {
    frame_->manager_->setPlotSettingsVisible(visible);
  }
}


}  // namespace autoviz
