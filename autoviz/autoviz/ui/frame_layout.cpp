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

namespace autoviz {

FrameLayout::FrameLayout(VisualizationFrame* frame) : frame_(frame) {}

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
         type == QLatin1String("ChannelGraphDock");
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
  // Teleop: default right, but allow drag/snap to either sidebar.
  if (frame_->panels_->panelTypeId(dock) == QLatin1String("TeleopDock")) {
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
  if (dock->isFloating() || dock->isTitleDragActive() ||
      center_dock_drag_count_ > 0) {
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
  if (dock == frame_->panels_->displays_dock_ || dock == frame_->panels_->properties_dock_) {
    return Qt::LeftDockWidgetArea;
  }
  // Teleop defaults to the right sidebar (Views / Selection peers).
  if (frame_->panels_->panelTypeId(dock) == QLatin1String("TeleopDock")) {
    return Qt::RightDockWidgetArea;
  }
  return Qt::RightDockWidgetArea;
}

void FrameLayout::ensureSidebarDockAttached(PanelDockWidget* dock) {
  if (dock == nullptr || isMainPanel(dock)) {
    return;
  }
  const bool is_teleop = frame_->panels_->panelTypeId(dock) == QLatin1String("TeleopDock");
  Qt::DockWidgetArea area = frame_->dockWidgetArea(dock);
  // Teleop always opens on the right (Views / Selection peers). Saved window
  // state or a prior left-side session must not keep it on the left when shown.
  if (is_teleop) {
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
        dock->property("panelDisposed").toBool() || dock->isFloating()) {
      continue;
    }
    // Use real widget visibility — toggle isChecked can desync when we
    // blockSignals during retile / close.
    if (dock->isVisible()) {
      visible.push_back(dock);
    } else {
      hidden.push_back(dock);
    }
  }

  if (visible.isEmpty() && frame_->viewport_->viewport_dock_ != nullptr) {
    frame_->viewport_->viewport_dock_->blockSignals(true);
    frame_->viewport_->viewport_dock_->show();
    frame_->viewport_->viewport_dock_->blockSignals(false);
    if (QAction* toggle = frame_->viewport_->viewport_dock_->toggleViewAction()) {
      toggle->blockSignals(true);
      toggle->setChecked(true);
      toggle->blockSignals(false);
    }
    visible.push_back(frame_->viewport_->viewport_dock_);
    hidden.removeAll(frame_->viewport_->viewport_dock_);
  }

  for (QDockWidget* dock : visible + hidden) {
    if (dock != nullptr) {
      dock->blockSignals(true);
    }
  }
  // Park Ogre/GL surfaces before splitter reparent (avoids GLX UAF without
  // destroying the scene — destroy/recreate races with NVIDIA teardown).
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
  frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
    frame_->viewport_->reinstallRenderWindowInEntry(entry);
  });
  for (QDockWidget* dock : visible + hidden) {
    if (dock != nullptr) {
      dock->blockSignals(false);
    }
  }

  // Keep Panels menu checkboxes aligned with actual dock visibility.
  for (QDockWidget* dock : visible + hidden) {
    if (dock == nullptr) {
      continue;
    }
    if (QAction* toggle = dock->toggleViewAction()) {
      toggle->blockSignals(true);
      toggle->setChecked(dock->isVisible());
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
  // Activating 3D View reopens Displays even after the user closed it or hid
  // the left sidebar (toolbar Hide Left / collapsed docks).
  displays_closed_by_user_ = false;
  if (frame_->panels_->displays_dock_ != nullptr) {
    if (frame_->manager_ != nullptr && frame_->manager_->hideLeftDock()) {
      hideLeftDock(false);
    }
    if (frame_->chrome_->toolbar_toggle_left_dock_action_ != nullptr &&
        !frame_->chrome_->toolbar_toggle_left_dock_action_->isChecked()) {
      frame_->chrome_->toolbar_toggle_left_dock_action_->blockSignals(true);
      frame_->chrome_->toolbar_toggle_left_dock_action_->setChecked(true);
      frame_->chrome_->toolbar_toggle_left_dock_action_->blockSignals(false);
    }
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

void FrameLayout::applyMainPanelDefaultLayout() {
  restoreExpandedMainPanel();
  if (main_panel_host_ == nullptr || frame_->viewport_->viewport_dock_ == nullptr) {
    return;
  }

  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr || !isMainPanel(dock) || dock == frame_->viewport_->viewport_dock_) {
      continue;
    }
    dock->hide();
  }

  frame_->viewport_->viewport_dock_->show();
  ensureMainPanelDockAttached(frame_->viewport_->viewport_dock_);
  last_active_dock_ = frame_->viewport_->viewport_dock_;
  tileCenterPanels();
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
      pre_expand_visible_main_panels_.isEmpty()) {
    return;
  }
  expanded_main_panel_dock_ = nullptr;
  for (const QString& name : pre_expand_visible_main_panels_) {
    if (auto* dock = frame_->findChild<PanelDockWidget*>(name)) {
      if (isMainPanel(dock)) {
        dock->show();
      }
    }
  }
  pre_expand_visible_main_panels_.clear();
  syncMainPanelExpandUi(nullptr);
  tileCenterPanels();
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

  pre_expand_visible_main_panels_.clear();
  for (PanelDockWidget* candidate : orderedDockWidgets()) {
    if (candidate != nullptr && isMainPanel(candidate) &&
        candidate->isVisible()) {
      pre_expand_visible_main_panels_.push_back(candidate->objectName());
    }
  }

  expanded_main_panel_dock_ = dock;
  ensureMainPanelDockAttached(dock);
  for (PanelDockWidget* candidate : orderedDockWidgets()) {
    if (candidate == nullptr || candidate == dock || !isMainPanel(candidate)) {
      continue;
    }
    candidate->hide();
  }
  dock->show();
  dock->raise();
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
    if (expanded_main_panel_dock_ == dock) {
      restoreExpandedMainPanel();
    }
  });
}

void FrameLayout::changePanelInDock(PanelDockWidget* source,
                                           const QString& target_object_name) {
  if (source == nullptr || target_object_name.isEmpty()) {
    return;
  }
  if (frame_->panels_->panelTypeId(source) == target_object_name) {
    return;
  }

  PanelDockWidget* target = nullptr;
  // Prefer an existing hidden dock of frame_ type (or the canonical objectName).
  if (auto* named = frame_->findChild<PanelDockWidget*>(target_object_name)) {
    if (!named->isVisible() || named == source ||
        !frame_->panels_->panelTypeSupportsMultiInstance(target_object_name)) {
      target = named;
    }
  }
  if (target == nullptr) {
    for (PanelDockWidget* dock : frame_->findChildren<PanelDockWidget*>()) {
      if (dock == nullptr || dock == source) {
        continue;
      }
      if (frame_->panels_->panelTypeId(dock) == target_object_name && !dock->isVisible()) {
        target = dock;
        break;
      }
    }
  }
  // Create on demand — Map/Publish/Service are not built at startup.
  if (target == nullptr) {
    if (target_object_name == QLatin1String("TopicsDock") ||
        target_object_name == QLatin1String("ChannelsDock")) {
      target = frame_->panels_->channel_dock_;
    } else if (target_object_name == QLatin1String("ChannelBrowserDock")) {
      target = frame_->panels_->channels_dock_;
    } else if (target_object_name == QLatin1String("ViewportDock")) {
      target = frame_->viewport_->createViewportPanelDock();
    } else if (target_object_name == QLatin1String("PlotDock")) {
      target = frame_->panels_->createPlotPanelDock();
    } else if (target_object_name == QLatin1String("TableDock")) {
      target = frame_->panels_->createTablePanelDock();
    } else if (target_object_name == QLatin1String("ImageDock")) {
      target = frame_->panels_->createImagePanelDock();
    } else if (target_object_name == QLatin1String("TeleopDock")) {
      target = frame_->panels_->teleop_dock_ != nullptr ? frame_->panels_->teleop_dock_ : frame_->panels_->createTeleopPanelDock();
      frame_->panels_->teleop_dock_ = target;
    } else if (target_object_name == QLatin1String("TfTreeDock")) {
      target = frame_->panels_->createTfTreePanelDock();
    } else if (target_object_name == QLatin1String("PublishDock")) {
      target = frame_->panels_->createPublishPanelDock();
    } else if (target_object_name == QLatin1String("MapDock")) {
      target = frame_->panels_->createMapPanelDock();
    } else if (target_object_name == QLatin1String("ServiceDock")) {
      target = frame_->panels_->createServicePanelDock();
    } else if (target_object_name == QLatin1String("ChannelGraphDock")) {
      target = frame_->panels_->channel_graph_dock_ != nullptr
                   ? frame_->panels_->channel_graph_dock_
                   : frame_->panels_->createChannelGraphPanelDock();
      frame_->panels_->channel_graph_dock_ = target;
    } else {
      target = frame_->findChild<PanelDockWidget*>(target_object_name);
    }
  }
  if (target == nullptr || target == source) {
    return;
  }

  const bool source_in_main =
      main_panel_host_ != nullptr && main_panel_host_->hostsPanel(source);
  const bool was_floating = source->isFloating();
  const QRect float_geometry = source->geometry();
  QMainWindow* source_host = dockHostForPanel(source);
  const Qt::DockWidgetArea source_area =
      source_host != nullptr ? source_host->dockWidgetArea(source)
                             : Qt::NoDockWidgetArea;

  PanelDockWidget* tab_anchor = nullptr;
  if (!source_in_main && source_host != nullptr) {
    for (PanelDockWidget* dock : orderedDockWidgets()) {
      if (dock == nullptr || dock == source) {
        continue;
      }
      const QList<QDockWidget*> tabified = source_host->tabifiedDockWidgets(dock);
      if (tabified.contains(source)) {
        tab_anchor = dock;
        break;
      }
    }
    if (tab_anchor == nullptr) {
      const QList<QDockWidget*> tabified =
          source_host->tabifiedDockWidgets(source);
      if (!tabified.isEmpty()) {
        tab_anchor = qobject_cast<PanelDockWidget*>(tabified.first());
      }
    }
  }

  // Detach source from its host.
  if (source_in_main) {
    main_panel_host_->removePanel(source);
  } else if (source_host != nullptr &&
             source_host->dockWidgetArea(source) != Qt::NoDockWidgetArea) {
    source_host->removeDockWidget(source);
  }
  source->hide();
  // Drop the Ogre native window immediately when leaving a 3D View. Waiting
  // for the next tick leaves the GL surface covering the new panel.
  if (frame_->panels_->panelTypeId(source) == QLatin1String("ViewportDock")) {
    if (ViewportPanelEntry* entry = frame_->viewport_->viewportEntryForDock(source)) {
      if (entry->ogre_viewport != nullptr) {
        entry->ogre_viewport->hideNativeSurface();
      }
    }
  }

  // Detach target from wherever it currently lives.
  if (main_panel_host_ != nullptr && main_panel_host_->hostsPanel(target)) {
    main_panel_host_->removePanel(target);
  }
  if (frame_->dockWidgetArea(target) != Qt::NoDockWidgetArea) {
    frame_->removeDockWidget(target);
  }
  if (main_panel_host_ != nullptr &&
      main_panel_host_->dockWidgetArea(target) != Qt::NoDockWidgetArea) {
    main_panel_host_->removeDockWidget(target);
  }

  if (was_floating) {
    target->setFloating(true);
    target->setGeometry(float_geometry);
  } else if (!isMainPanel(target)) {
    // Sidebar panels (e.g. Teleop): never place in the center host.
    Qt::DockWidgetArea area = defaultSidebarArea(target);
    if (frame_->panels_->panelTypeId(target) == QLatin1String("TeleopDock")) {
      area = Qt::RightDockWidgetArea;
    } else if (!source_in_main &&
               (source_area == Qt::LeftDockWidgetArea ||
                source_area == Qt::RightDockWidgetArea)) {
      area = source_area;
    }
    configureSidebarDock(target, area);
    frame_->addDockWidget(area, target);
    if (tab_anchor != nullptr && !source_in_main) {
      frame_->tabifyDockWidget(tab_anchor, target);
    }
    ensureSidebarDockAttached(target);
  } else if (source_in_main || isMainPanel(target)) {
    // Map/Plot/… belong in the center host (AllowedAreas is NoDockWidgetArea).
    configureMainPanelDock(target);
    wireMainPanelExpandTracking(target);
    if (main_panel_host_ != nullptr) {
      main_panel_host_->addPanel(target);
    }
  } else if (source_host != nullptr && source_area != Qt::NoDockWidgetArea) {
    source_host->addDockWidget(source_area, target);
    if (tab_anchor != nullptr) {
      source_host->tabifyDockWidget(tab_anchor, target);
    }
  } else {
    addMainPanelDock(target, Qt::LeftDockWidgetArea);
  }

  target->show();
  target->raise();
  if (QWidget* content = target->widget()) {
    content->show();
    content->raise();
  }
  last_active_dock_ = target;
  if (frame_->panels_->panelTypeId(target) == QLatin1String("ViewportDock")) {
    frame_->viewport_->ensureViewportPanelReady(target);
    frame_->viewport_->setActiveViewportDock(target);
  }
  frame_->panels_->activatePanelDock(target);
  scheduleTileCenterPanels();
  frame_->panels_->syncDeletePanelMenu();
  frame_->session_->markConfigModified();
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
        dock == frame_->panels_->properties_dock_) {
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
  if (area != Qt::LeftDockWidgetArea && area != Qt::RightDockWidgetArea) {
    return false;
  }
  for (PanelDockWidget* dock : frame_->findChildren<PanelDockWidget*>()) {
    if (dock == nullptr || !dock->isVisible() || dock->isFloating() ||
        dock == frame_->panels_->time_dock_ || isMainPanel(dock)) {
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
  // Floating panels are intentionally undocked — do not force-reattach.
  if (dock_widget != nullptr && dock_widget->isFloating()) {
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
      ensureMainPanelDockAttached(dock_widget);
    }
    // Directed Split owns layout until the next explicit grid tile.
    if (!center_manual_layout_) {
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

void FrameLayout::showPanelByObjectName(const QString& object_name) {
  if (object_name.isEmpty()) {
    return;
  }

  if (frame_->panels_->panelTypeSupportsMultiInstance(object_name)) {
    PanelDockWidget* existing_dock = nullptr;
    for (PanelDockWidget* dock : frame_->findChildren<PanelDockWidget*>()) {
      if (dock != nullptr && frame_->panels_->panelTypeId(dock) == object_name) {
        existing_dock = dock;
        break;
      }
    }
    if (existing_dock != nullptr && !existing_dock->isVisible()) {
      ensureMainPanelDockAttached(existing_dock);
      if (frame_->panels_->panelTypeId(existing_dock) == QLatin1String("ViewportDock")) {
        frame_->viewport_->ensureViewportPanelReady(existing_dock);
      }
      existing_dock->show();
      existing_dock->raise();
      last_active_dock_ = existing_dock;
      frame_->panels_->activatePanelDock(existing_dock);
      frame_->panels_->syncDeletePanelMenu();
      frame_->session_->markConfigModified();
      return;
    }
    PanelDockWidget* visible_dock = nullptr;
    for (PanelDockWidget* dock : frame_->findChildren<PanelDockWidget*>()) {
      if (dock != nullptr && frame_->panels_->panelTypeId(dock) == object_name && dock->isVisible()) {
        visible_dock = dock;
        break;
      }
    }
    if (visible_dock != nullptr) {
      PanelDockWidget* duplicate = frame_->panels_->duplicatePanelDock(visible_dock);
      if (duplicate != nullptr) {
        QMainWindow* host = dockHostForPanel(visible_dock);
        if (host != nullptr) {
          Qt::DockWidgetArea area = host->dockWidgetArea(visible_dock);
          if (area == Qt::NoDockWidgetArea) {
            area = isMainPanel(visible_dock) ? Qt::LeftDockWidgetArea
                                             : Qt::RightDockWidgetArea;
          }
          host->addDockWidget(area, duplicate);
        }
        duplicate->show();
        duplicate->raise();
        last_active_dock_ = duplicate;
        frame_->panels_->activatePanelDock(duplicate);
        frame_->panels_->syncDeletePanelMenu();
        frame_->session_->markConfigModified();
        return;
      }
    }
  }

  auto* dock = frame_->findChild<PanelDockWidget*>(object_name);
  if (dock == nullptr && object_name == QLatin1String("TopicsDock")) {
    dock = frame_->panels_->channels_dock_;
  }
  if (dock == nullptr && object_name == QLatin1String("PlotDock")) {
    dock = frame_->panels_->createPlotPanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("TableDock")) {
    dock = frame_->panels_->createTablePanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("ImageDock")) {
    dock = frame_->panels_->createImagePanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("TeleopDock")) {
    dock = frame_->panels_->teleop_dock_;
    if (dock == nullptr) {
      dock = frame_->panels_->createTeleopPanelDock(object_name);
      frame_->panels_->teleop_dock_ = dock;
      addSidebarDock(dock, Qt::RightDockWidgetArea);
    }
  }
  if (dock == nullptr && object_name == QLatin1String("TfTreeDock")) {
    dock = frame_->panels_->createTfTreePanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("PublishDock")) {
    dock = frame_->panels_->createPublishPanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("MapDock")) {
    dock = frame_->panels_->createMapPanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("ServiceDock")) {
    dock = frame_->panels_->createServicePanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("ChannelGraphDock")) {
    dock = frame_->panels_->channel_graph_dock_;
    if (dock == nullptr) {
      dock = frame_->panels_->createChannelGraphPanelDock(object_name);
      frame_->panels_->channel_graph_dock_ = dock;
      addMainPanelDock(dock, Qt::LeftDockWidgetArea);
    }
  }
  if (dock == nullptr) {
    return;
  }
  if (isMainPanel(dock)) {
    ensureMainPanelDockAttached(dock);
  } else {
    ensureSidebarDockAttached(dock);
  }
  dock->show();
  dock->raise();
  last_active_dock_ = dock;
  frame_->panels_->activatePanelDock(dock);
  scheduleTileCenterPanels();
  frame_->panels_->syncDeletePanelMenu();
  frame_->session_->markConfigModified();
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

      // Center host uses QSplitter — sizes already set in splitPanel/tilePanels.
      const bool center_split =
          host_guard.data() == static_cast<QMainWindow*>(main_panel_host_) ||
          (main_panel_host_ != nullptr &&
           main_panel_host_->hostsPanel(source_guard.data()));
      if (!center_split && orientation == Qt::Vertical) {
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
    suppress_center_tile_ = false;
    syncCenterLayout();
    ensureTimeDockAtBottom();
    if (QStatusBar* status = frame_->statusBar()) {
      status->setVisible(true);
      status->raise();
    }
    // Force viewport hosts to relayout so GL widgets fill the new dock size.
    for (PanelDockWidget* dock : {source_guard.data(), duplicate_guard.data()}) {
      if (dock == nullptr) {
        continue;
      }
      if (ViewportPanelEntry* entry = frame_->viewport_->viewportEntryForDock(dock)) {
        frame_->viewport_->ensureViewportPanelReady(dock);
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
                                   frame_->panels_->selection_dock_,  frame_->panels_->tool_props_dock_, frame_->panels_->views_dock_,
                                   frame_->panels_->time_dock_,       frame_->panels_->channel_dock_,
                                   frame_->panels_->channels_dock_,
                                   frame_->panels_->tf_dock_, frame_->panels_->image_dock_,
                                   frame_->panels_->plot_dock_,       frame_->panels_->teleop_dock_,
                                   frame_->panels_->channel_graph_dock_};
  for (PanelDockWidget* dock : frame_->findChildren<PanelDockWidget*>()) {
    if (dock != nullptr && !docks.contains(dock)) {
      docks.push_back(dock);
    }
  }
  return docks;
}

}  // namespace autoviz
