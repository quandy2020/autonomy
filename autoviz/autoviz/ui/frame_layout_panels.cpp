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
    } else if (target_object_name == QLatin1String("RecordDock")) {
      target = frame_->panels_->record_dock_ != nullptr
                   ? frame_->panels_->record_dock_
                   : frame_->panels_->createRecordPanelDock();
      frame_->panels_->record_dock_ = target;
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
    } else if (target_object_name == QLatin1String("NavigationDock")) {
      target = frame_->panels_->navigation_dock_ != nullptr
                   ? frame_->panels_->navigation_dock_
                   : frame_->panels_->createNavigationPanelDock();
      frame_->panels_->navigation_dock_ = target;
    } else if (target_object_name == QLatin1String("TfTreeDock")) {
      target = frame_->panels_->createTfTreePanelDock();
    } else if (target_object_name == QLatin1String("EvidenceDock")) {
      target = frame_->panels_->createEvidencePanelDock();
    } else if (target_object_name == QLatin1String("OperatorToolsDock")) {
      target = frame_->panels_->createOperatorToolsPanelDock();
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

  // Drop the Ogre native window immediately when leaving a 3D View. Waiting
  // for the next tick leaves the GL surface covering the new panel.
  if (frame_->panels_->panelTypeId(source) == QLatin1String("ViewportDock")) {
    if (ViewportPanelEntry* entry =
            frame_->viewport_->viewportEntryForDock(source)) {
      if (entry->ogre_viewport != nullptr) {
        entry->ogre_viewport->hideNativeSurface();
      }
      frame_->viewport_->parkRenderWindowInEntry(*entry);
    }
  }

  // Detach target from wherever it currently lives (not the source leaf yet).
  if (main_panel_host_ != nullptr && main_panel_host_->hostsPanel(target) &&
      target != source) {
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
    if (source_in_main) {
      main_panel_host_->removePanel(source);
    } else if (source_host != nullptr &&
               source_host->dockWidgetArea(source) != Qt::NoDockWidgetArea) {
      source_host->removeDockWidget(source);
    }
    source->hide();
    target->setFloating(true);
    target->setGeometry(float_geometry);
  } else if (!isMainPanel(target)) {
    // Sidebar panels (e.g. Teleop): never place in the center host.
    if (source_in_main) {
      main_panel_host_->removePanel(source);
    } else if (source_host != nullptr &&
               source_host->dockWidgetArea(source) != Qt::NoDockWidgetArea) {
      source_host->removeDockWidget(source);
    }
    source->hide();
    Qt::DockWidgetArea area = defaultSidebarArea(target);
    if (frame_->panels_->panelTypeId(target) == QLatin1String("TeleopDock") ||
        frame_->panels_->panelTypeId(target) == QLatin1String("NavigationDock")) {
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
  } else if (source_in_main && main_panel_host_ != nullptr) {
    // Foxglove Change: swap into the same mosaic leaf so Split geometry is
    // preserved. removePanel+addPanel collapsed the leaf then appended Map at
    // the root — leaving an empty middle pane between 3D View and Map.
    configureMainPanelDock(target);
    wireMainPanelExpandTracking(target);
    if (!main_panel_host_->replacePanel(source, target)) {
      main_panel_host_->removePanel(source);
      source->hide();
      main_panel_host_->addPanel(target);
    } else {
      source->hide();
    }
    // Keep the Split mosaic; do not let scheduleTileCenterPanels rebuild a grid.
    center_manual_layout_ = true;
    ++center_tile_epoch_;
  } else if (isMainPanel(target)) {
    if (source_host != nullptr &&
        source_host->dockWidgetArea(source) != Qt::NoDockWidgetArea) {
      source_host->removeDockWidget(source);
    }
    source->hide();
    configureMainPanelDock(target);
    wireMainPanelExpandTracking(target);
    if (main_panel_host_ != nullptr) {
      main_panel_host_->addPanel(target);
    }
  } else if (source_host != nullptr && source_area != Qt::NoDockWidgetArea) {
    source_host->removeDockWidget(source);
    source->hide();
    source_host->addDockWidget(source_area, target);
    if (tab_anchor != nullptr) {
      source_host->tabifyDockWidget(tab_anchor, target);
    }
  } else {
    source->hide();
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

  // Change replaces the leaf — destroy Split duplicates of the old type so the
  // Panels menu does not keep leftover "3D View" / "Plot" toggles. Keep the
  // canonical primary (objectName == typeId) hidden for reopen.
  const QString source_type = frame_->panels_->panelTypeId(source);
  const bool source_multi =
      frame_->panels_->panelTypeSupportsMultiInstance(source_type);
  const bool source_canonical = source->objectName() == source_type;
  if (source_multi && !source_canonical) {
    if (expanded_main_panel_dock_ == source) {
      expanded_main_panel_dock_ = nullptr;
    }
    if (last_active_dock_ == source) {
      last_active_dock_ = target;
    }
    if (QAction* toggle = source->toggleViewAction()) {
      if (frame_->chrome_->panels_menu_ != nullptr) {
        frame_->chrome_->panels_menu_->removeAction(toggle);
      }
      frame_->chrome_->panels_menu_toggle_actions_.removeAll(
          QPointer<QAction>(toggle));
    }
    source->setProperty("panelDisposed", true);
    if (source_type == QLatin1String("ViewportDock")) {
      frame_->viewport_->removeViewportPanel(source);
    }
    if (main_panel_host_ != nullptr && main_panel_host_->hostsPanel(source)) {
      main_panel_host_->removePanel(source);
    }
    const QPointer<PanelDockWidget> source_guard(source);
    QObject::connect(source, &QObject::destroyed, frame_, [this]() {
              frame_->chrome_->rebuildPanelsMenuToggles();
              frame_->panels_->syncDeletePanelMenu();
            },
            static_cast<Qt::ConnectionType>(Qt::QueuedConnection |
                                            Qt::SingleShotConnection));
    source->deleteLater();
    Q_UNUSED(source_guard);
  }

  if (main_panel_host_ != nullptr) {
    main_panel_host_->syncHorizontalDockLayout();
  }
  // Directed Change after Split owns the mosaic — skip auto grid retile.
  scheduleTileCenterPanels();
  frame_->chrome_->rebuildPanelsMenuToggles();
  frame_->panels_->syncDeletePanelMenu();
  frame_->session_->markConfigModified();
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
      center_manual_layout_ = false;
      suppress_center_tile_ = false;
      ensureMainPanelDockAttached(existing_dock);
      if (frame_->panels_->panelTypeId(existing_dock) == QLatin1String("ViewportDock")) {
        frame_->viewport_->ensureViewportPanelReady(existing_dock);
        frame_->viewport_->setActiveViewportDock(existing_dock);
      }
      existing_dock->show();
      existing_dock->raise();
      last_active_dock_ = existing_dock;
      frame_->panels_->activatePanelDock(existing_dock);
      scheduleTileCenterPanels();
      frame_->chrome_->rebuildPanelsMenuToggles();
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
  if (dock == nullptr && object_name == QLatin1String("RecordDock")) {
    dock = frame_->panels_->record_dock_;
    if (dock == nullptr) {
      dock = frame_->panels_->createRecordPanelDock(object_name);
      frame_->panels_->record_dock_ = dock;
      addSidebarDock(dock, Qt::RightDockWidgetArea);
      if (frame_->panels_->views_dock_ != nullptr) {
        frame_->tabifyDockWidget(frame_->panels_->views_dock_, dock);
      }
    }
  }
  if (dock == nullptr && object_name == QLatin1String("TeleopDock")) {
    dock = frame_->panels_->teleop_dock_;
    if (dock == nullptr) {
      dock = frame_->panels_->createTeleopPanelDock(object_name);
      frame_->panels_->teleop_dock_ = dock;
      addSidebarDock(dock, Qt::RightDockWidgetArea);
    }
  }
  if (dock == nullptr && object_name == QLatin1String("NavigationDock")) {
    dock = frame_->panels_->navigation_dock_;
    if (dock == nullptr) {
      dock = frame_->panels_->createNavigationPanelDock(object_name);
      frame_->panels_->navigation_dock_ = dock;
      addSidebarDock(dock, Qt::RightDockWidgetArea);
    }
  }
  if (dock == nullptr && object_name == QLatin1String("TfTreeDock")) {
    dock = frame_->panels_->createTfTreePanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("EvidenceDock")) {
    dock = frame_->panels_->createEvidencePanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("OperatorToolsDock")) {
    dock = frame_->panels_->createOperatorToolsPanelDock(object_name);
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


}  // namespace autoviz
