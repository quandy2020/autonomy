/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_panels.hpp"
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/ui/frame_detail.hpp"
#include "autoviz/common/selection.hpp"
#include "autoviz/display/display_group.hpp"
#include "autoviz/ui/displays/panel.hpp"
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
#include "autoviz/ui/image/image_view_widget.hpp"
#include "autoviz/ui/panel_host.hpp"
#include "autoviz/ui/panel/role.hpp"
#include "autoviz/ui/inspector/property_panel.hpp"
#include "autoviz/ui/dialog/import_record.hpp"
#include "autoviz/ui/dialog/record_open.hpp"
#include "autoviz/ui/plot/plot_config_io.hpp"
#include "autoviz/ui/publish/publish_config_io.hpp"
#include "autoviz/ui/service/service_config_io.hpp"
#include "autoviz/ui/teleop/teleop_config_io.hpp"
#include "autoviz/ui/map/map_config_io.hpp"
#include "autoviz/ui/image/image_config_io.hpp"
#include "autoviz/ui/plot/plot_panel.hpp"
#include "autoviz/ui/table/table_config_io.hpp"
#include "autoviz/ui/table/table_panel.hpp"
#include "autoviz/ui/teleop/teleop_panel.hpp"
#include "autoviz/ui/raw/panel.hpp"
#include "autoviz/ui/channels/channels_panel.hpp"
#include "autoviz/ui/record/record_panel.hpp"
#include "autoviz/ui/inspector/selection_panel.hpp"
#include "autoviz/ui/inspector/tool_properties_panel.hpp"
#include "autoviz/ui/tf_tree/panel.hpp"
#include "autoviz/ui/publish/publish_panel.hpp"
#include "autoviz/ui/map/map_panel.hpp"
#include "autoviz/ui/channel_graph/channel_graph_panel.hpp"
#include "autoviz/ui/channel_graph/channel_graph_config_io.hpp"
#include "autoviz/ui/service/service_panel.hpp"
#include "autoviz/ui/time/panel.hpp"
#include "autoviz/ui/panel/name_map.hpp"
#include "autoviz/ui/viewport_toolbar.hpp"
#include "autoviz/ui/viewport_hud.hpp"
#include "autoviz/ui/views/panel.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"

namespace autoviz {

FramePanels::FramePanels(VisualizationFrame* frame) : frame_(frame) {}

void FramePanels::bindPlotToPropertyInspector(plot::PlotPanel* panel) {
  if (panel == nullptr || property_inspector_panel_ == nullptr) {
    return;
  }
  if (inspector_plot_panel_ != nullptr && inspector_plot_panel_ != panel) {
    inspector_plot_panel_->recallSettingsWidget();
  }
  if (inspector_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(inspector_image_panel_);
  }
  if (inspector_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(inspector_teleop_panel_);
  }
  inspector_plot_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(panel->settingsWidgetForInspector(),
                                              title.isEmpty() ? frame_->tr("Plot") : title);
  panel->refreshSettingsChannels();
  frame_->layout_->raiseLeftSidebarProperties();
  panel->setSettingsButtonChecked(frame_->layout_->isPropertyInspectorVisible());
}

void FramePanels::clearPropertyInspectorForPlot(plot::PlotPanel* panel) {
  if (panel == nullptr) {
    return;
  }
  panel->recallSettingsWidget();
  if (inspector_plot_panel_ == panel) {
    inspector_plot_panel_ = nullptr;
    if (property_inspector_panel_ != nullptr) {
      property_inspector_panel_->clearContent();
    }
  }
}

void FramePanels::bindImageToPropertyInspector(image::ImagePanel* panel) {
  if (panel == nullptr || property_inspector_panel_ == nullptr) {
    return;
  }
  if (inspector_image_panel_ != nullptr && inspector_image_panel_ != panel) {
    inspector_image_panel_->recallSettingsWidget();
  }
  if (inspector_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(inspector_plot_panel_);
  }
  if (inspector_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(inspector_teleop_panel_);
  }
  inspector_image_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(panel->settingsWidgetForInspector(),
                                              title.isEmpty() ? frame_->tr("Image") : title);
  panel->refreshSettingsChannels();
  frame_->layout_->raiseLeftSidebarProperties();
  panel->setSettingsButtonChecked(frame_->layout_->isPropertyInspectorVisible());
}

void FramePanels::clearPropertyInspectorForImage(image::ImagePanel* panel) {
  if (panel == nullptr) {
    return;
  }
  panel->recallSettingsWidget();
  if (inspector_image_panel_ == panel) {
    inspector_image_panel_ = nullptr;
    if (property_inspector_panel_ != nullptr) {
      property_inspector_panel_->clearContent();
    }
  }
}

void FramePanels::bindTeleopToPropertyInspector(teleop::TeleopPanel* panel) {
  if (panel == nullptr || property_inspector_panel_ == nullptr) {
    return;
  }
  if (inspector_teleop_panel_ != nullptr && inspector_teleop_panel_ != panel) {
    inspector_teleop_panel_->recallSettingsWidget();
  }
  if (inspector_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(inspector_plot_panel_);
  }
  if (inspector_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(inspector_image_panel_);
  }
  inspector_teleop_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(panel->settingsWidgetForInspector(),
                                              title.isEmpty() ? frame_->tr("Teleop") : title);
  frame_->layout_->raiseLeftSidebarProperties();
  panel->setSettingsButtonChecked(frame_->layout_->isPropertyInspectorVisible());
}

void FramePanels::clearPropertyInspectorForTeleop(teleop::TeleopPanel* panel) {
  if (panel == nullptr) {
    return;
  }
  panel->recallSettingsWidget();
  if (inspector_teleop_panel_ == panel) {
    inspector_teleop_panel_ = nullptr;
    if (property_inspector_panel_ != nullptr) {
      property_inspector_panel_->clearContent();
    }
  }
}

void FramePanels::bindPublishToPropertyInspector(
    publish_panel::PublishPanel* panel) {
  if (panel == nullptr || property_inspector_panel_ == nullptr) {
    return;
  }
  if (inspector_publish_panel_ != nullptr && inspector_publish_panel_ != panel) {
    inspector_publish_panel_->recallSettingsWidget();
  }
  if (inspector_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(inspector_plot_panel_);
  }
  if (inspector_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(inspector_image_panel_);
  }
  if (inspector_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(inspector_teleop_panel_);
  }
  if (inspector_service_panel_ != nullptr) {
    clearPropertyInspectorForService(inspector_service_panel_);
  }
  inspector_publish_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(
      panel->settingsWidgetForInspector(),
      title.isEmpty() ? frame_->tr("Publish") : title);
  frame_->layout_->raiseLeftSidebarProperties();
  panel->setSettingsButtonChecked(frame_->layout_->isPropertyInspectorVisible());
}

void FramePanels::clearPropertyInspectorForPublish(
    publish_panel::PublishPanel* panel) {
  if (panel == nullptr) {
    return;
  }
  panel->recallSettingsWidget();
  if (inspector_publish_panel_ == panel) {
    inspector_publish_panel_ = nullptr;
    if (property_inspector_panel_ != nullptr) {
      property_inspector_panel_->clearContent();
    }
  }
}

void FramePanels::bindServiceToPropertyInspector(
    service_panel::ServicePanel* panel) {
  if (panel == nullptr || property_inspector_panel_ == nullptr) {
    return;
  }
  if (inspector_service_panel_ != nullptr && inspector_service_panel_ != panel) {
    inspector_service_panel_->recallSettingsWidget();
  }
  if (inspector_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(inspector_plot_panel_);
  }
  if (inspector_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(inspector_image_panel_);
  }
  if (inspector_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(inspector_teleop_panel_);
  }
  if (inspector_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(inspector_publish_panel_);
  }
  if (inspector_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(inspector_map_panel_);
  }
  inspector_service_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(panel->settingsWidgetForInspector(),
                                              title.isEmpty() ? frame_->tr("Service Call")
                                                                : title);
  frame_->layout_->raiseLeftSidebarProperties();
  panel->setSettingsButtonChecked(frame_->layout_->isPropertyInspectorVisible());
}

void FramePanels::clearPropertyInspectorForService(
    service_panel::ServicePanel* panel) {
  if (panel == nullptr) {
    return;
  }
  panel->recallSettingsWidget();
  if (inspector_service_panel_ == panel) {
    inspector_service_panel_ = nullptr;
    if (property_inspector_panel_ != nullptr) {
      property_inspector_panel_->clearContent();
    }
  }
}

void FramePanels::bindMapToPropertyInspector(map::MapPanel* panel) {
  if (panel == nullptr || property_inspector_panel_ == nullptr) {
    return;
  }
  if (inspector_map_panel_ != nullptr && inspector_map_panel_ != panel) {
    inspector_map_panel_->recallSettingsWidget();
  }
  if (inspector_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(inspector_plot_panel_);
  }
  if (inspector_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(inspector_image_panel_);
  }
  if (inspector_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(inspector_teleop_panel_);
  }
  if (inspector_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(inspector_publish_panel_);
  }
  inspector_map_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(
      panel->settingsWidgetForInspector(),
      title.isEmpty() ? frame_->tr("Map") : title);
  frame_->layout_->raiseLeftSidebarProperties();
  panel->setSettingsButtonChecked(frame_->layout_->isPropertyInspectorVisible());
}

void FramePanels::clearPropertyInspectorForMap(map::MapPanel* panel) {
  if (panel == nullptr) {
    return;
  }
  panel->recallSettingsWidget();
  if (inspector_map_panel_ == panel) {
    inspector_map_panel_ = nullptr;
    if (property_inspector_panel_ != nullptr) {
      property_inspector_panel_->clearContent();
    }
  }
}

void FramePanels::syncDeletePanelMenu() {
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    unregisterDeletePanelAction(dock);
  }
  QVector<PanelDockWidget*> visible_docks;
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock != nullptr && dock->isVisible()) {
      visible_docks.push_back(dock);
    }
  }
  std::sort(visible_docks.begin(), visible_docks.end(),
            [](PanelDockWidget* a, PanelDockWidget* b) {
              return QString::localeAwareCompare(a->windowTitle(),
                                                 b->windowTitle()) < 0;
            });
  for (PanelDockWidget* dock : visible_docks) {
    registerDeletePanelAction(dock);
  }
}

bool FramePanels::panelTypeSupportsMultiInstance(
    const QString& panel_type_id) const {
  return panel_type_id == QLatin1String("PlotDock") ||
         panel_type_id == QLatin1String("ImageDock") ||
         panel_type_id == QLatin1String("TeleopDock") ||
         panel_type_id == QLatin1String("TfTreeDock") ||
         panel_type_id == QLatin1String("PublishDock") ||
         panel_type_id == QLatin1String("MapDock") ||
         panel_type_id == QLatin1String("ChannelGraphDock") ||
         panel_type_id == QLatin1String("ServiceDock") ||
         panel_type_id == QLatin1String("TableDock") ||
         panel_type_id == QLatin1String("ViewportDock");
}

QString FramePanels::panelTypeId(const PanelDockWidget* dock) const {
  if (dock == nullptr) {
    return {};
  }
  const QVariant type_id = dock->property("panelTypeId");
  if (type_id.isValid()) {
    return type_id.toString();
  }
  return dock->objectName();
}

QString FramePanels::uniquePanelObjectName(const QString& base) const {
  if (frame_->findChild<PanelDockWidget*>(base) == nullptr) {
    return base;
  }
  for (int i = 2; i < 1000; ++i) {
    const QString candidate = QStringLiteral("%1_%2").arg(base).arg(i);
    if (frame_->findChild<PanelDockWidget*>(candidate) == nullptr) {
      return candidate;
    }
  }
  return QStringLiteral("%1_%2").arg(base).arg(QDateTime::currentMSecsSinceEpoch());
}

void FramePanels::registerPanelDock(PanelDockWidget* dock) {
  if (dock == nullptr || dock->property("panelDockRegistered").toBool()) {
    return;
  }
  dock->setProperty("panelDockRegistered", true);
  IconLoader::applyDockPanelChrome(dock, panelTypeId(dock));
  QObject::connect(dock, &PanelDockWidget::activated, frame_, [this, dock]() {
    frame_->layout_->last_active_dock_ = dock;
    if (panelTypeId(dock) == QLatin1String("ViewportDock")) {
      frame_->viewport_->setActiveViewportDock(dock);
      return;
    }
    if (panelTypeId(dock) == QLatin1String("PlotDock")) {
      if (auto* panel = qobject_cast<plot::PlotPanel*>(dock->widget())) {
        setActivePlotPanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("ImageDock")) {
      if (auto* panel = qobject_cast<image::ImagePanel*>(dock->widget())) {
        setActiveImagePanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("TeleopDock")) {
      if (auto* panel = qobject_cast<teleop::TeleopPanel*>(dock->widget())) {
        setActiveTeleopPanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("PublishDock")) {
      if (auto* panel = qobject_cast<publish_panel::PublishPanel*>(dock->widget())) {
        setActivePublishPanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("ServiceDock")) {
      if (auto* panel = qobject_cast<service_panel::ServicePanel*>(dock->widget())) {
        setActiveServicePanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("MapDock")) {
      if (auto* panel = qobject_cast<map::MapPanel*>(dock->widget())) {
        setActiveMapPanel(panel);
      }
    }
  });
  QObject::connect(dock, &QDockWidget::visibilityChanged, frame_, &VisualizationFrame::onDockPanelVisibilityChange,

          Qt::UniqueConnection);
  QObject::connect(frame_, &VisualizationFrame::fullScreenChange, dock,
          &PanelDockWidget::overrideVisibility, Qt::UniqueConnection);
  QObject::connect(dock, &QDockWidget::dockLocationChanged, frame_, &VisualizationFrame::markConfigModified);
  QObject::connect(dock, &PanelDockWidget::titleDragStarted, frame_, [this, dock]() {
    if (!frame_->layout_->isMainPanel(dock)) {
      return;
    }
    ++frame_->layout_->center_dock_drag_count_;
    ++frame_->layout_->center_tile_epoch_;
    frame_->layout_->center_tile_pending_ = false;
    frame_->layout_->center_manual_layout_ = true;
  });
  QObject::connect(dock, &PanelDockWidget::titleDragFinished, frame_, [this, dock]() {
    if (!frame_->layout_->isMainPanel(dock)) {
      return;
    }
    if (frame_->layout_->center_dock_drag_count_ > 0) {
      --frame_->layout_->center_dock_drag_count_;
    }
    // Do not force-reattach or setFloating(false) here — that cancels dock
    // snap (吸附) and can SIGSEGV while Qt is still finishing the drop.
    Q_UNUSED(dock);
    // Keep manual layout after user drag/snap; only refresh host geometry.
    frame_->layout_->syncCenterLayout();
    frame_->session_->markConfigModified();
  });
  QObject::connect(dock, &QDockWidget::topLevelChanged, frame_, [this](bool /*floating*/) {
    frame_->session_->markConfigModified();
  });
  QObject::connect(dock, &PanelDockWidget::closed, frame_, [this, dock]() {
    // Drag may have left retile suppressed; always clear on close.
    frame_->layout_->suppress_center_tile_ = false;
    frame_->layout_->center_dock_drag_count_ = 0;
    if (QAction* toggle = dock->toggleViewAction()) {
      toggle->blockSignals(true);
      toggle->setChecked(false);
      toggle->blockSignals(false);
    }
    if (dock == displays_dock_) {
      frame_->layout_->displays_closed_by_user_ = true;
    }
    unregisterDeletePanelAction(dock);

    const bool is_primary =
        dock == frame_->viewport_->viewport_dock_ || dock == image_dock_ || dock == plot_dock_ ||
        dock == tf_dock_ || dock == channel_graph_dock_ ||
        dock == teleop_dock_ ||
        dock == channel_dock_ || dock == channels_dock_ ||
        dock == displays_dock_ || dock == record_dock_ ||
        dock == properties_dock_ ||
        dock == views_dock_ ||
        dock == selection_dock_ || dock == tool_props_dock_ ||
        dock == time_dock_ ||
        // Canonical singleton object names (startup primary instances).
        dock->objectName() == QLatin1String("ChannelGraphDock") ||
        dock->objectName() == QLatin1String("TeleopDock") ||
        dock->objectName() == QLatin1String("RecordDock");
    const bool drop_duplicate = frame_->layout_->isMainPanel(dock) && !is_primary;
    if (drop_duplicate) {
      if (frame_->layout_->expanded_main_panel_dock_ == dock) {
        frame_->layout_->expanded_main_panel_dock_ = nullptr;
      }
      if (frame_->layout_->last_active_dock_ == dock) {
        frame_->layout_->last_active_dock_ = nullptr;
      }
      // Detach menu toggle before destroy — action is owned by the dock.
      if (QAction* toggle = dock->toggleViewAction()) {
        if (frame_->chrome_->panels_menu_ != nullptr) {
          frame_->chrome_->panels_menu_->removeAction(toggle);
        }
        frame_->chrome_->panels_menu_toggle_actions_.removeAll(QPointer<QAction>(toggle));
      }
      dock->setProperty("panelDisposed", true);
      if (panelTypeId(dock) == QLatin1String("ViewportDock")) {
        frame_->viewport_->removeViewportPanel(dock);
      }
    }

    // Defer host removal / destroy / menu rebuild out of closeEvent.
    const QPointer<PanelDockWidget> dock_guard(dock);
    QTimer::singleShot(0, frame_, [this, dock_guard, drop_duplicate]() {
      if (drop_duplicate && dock_guard) {
        if (frame_->layout_->main_panel_host_ != nullptr &&
            frame_->layout_->main_panel_host_->hostsPanel(dock_guard.data())) {
          frame_->layout_->main_panel_host_->removePanel(dock_guard.data());
        }
        if (QMainWindow* host = frame_->layout_->dockHostForPanel(dock_guard.data())) {
          if (host->dockWidgetArea(dock_guard.data()) != Qt::NoDockWidgetArea) {
            host->removeDockWidget(dock_guard.data());
          }
        }
        if (frame_->dockWidgetArea(dock_guard.data()) != Qt::NoDockWidgetArea) {
          frame_->removeDockWidget(dock_guard.data());
        }
        // Rebuild after the dock (and its toggleViewAction) is gone.
        QObject::connect(dock_guard.data(), &QObject::destroyed, frame_, [this]() {
                  frame_->chrome_->rebuildPanelsMenuToggles();
                  frame_->layout_->scheduleTileCenterPanels();
                  syncDeletePanelMenu();
                },
                static_cast<Qt::ConnectionType>(Qt::QueuedConnection |
                                                Qt::SingleShotConnection));
        dock_guard->deleteLater();
        return;
      }
      frame_->chrome_->rebuildPanelsMenuToggles();
      frame_->layout_->scheduleTileCenterPanels();
      syncDeletePanelMenu();
    });
    frame_->session_->markConfigModified();
  });
  frame_->layout_->wireMainPanelExpandTracking(dock);
  frame_->chrome_->registerPanelMenuToggle(dock);
}

void FramePanels::updatePlotDockTitle(PanelDockWidget* dock,
                                             plot::PlotPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Plot") : title);
}

void FramePanels::capturePlotPanelConfigs() {
  std::vector<common::PlotPanelPersistConfig> panels;
  panels.reserve(4);
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || panelTypeId(dock) != QLatin1String("PlotDock")) {
      continue;
    }
    auto* panel = qobject_cast<plot::PlotPanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panels.push_back(plot::ToPersistConfig(dock->objectName(), panel->config()));
  }
  frame_->manager_->setPlotPanels(panels);
}

void FramePanels::ensurePlotDockExists(const QString& object_name) {
  if (object_name.isEmpty() ||
      frame_->findChild<PanelDockWidget*>(object_name) != nullptr) {
    return;
  }
  PanelDockWidget* dock = createPlotPanelDock(object_name);
  frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
}

void FramePanels::restorePlotPanelConfigs() {
  const std::vector<common::PlotPanelPersistConfig>& saved =
      frame_->manager_->plotPanels();
  if (saved.empty()) {
    if (plot_panel_ != nullptr) {
      updatePlotDockTitle(plot_dock_, plot_panel_);
    }
    return;
  }

  for (const common::PlotPanelPersistConfig& entry : saved) {
    ensurePlotDockExists(QString::fromStdString(entry.object_name));
  }

  for (const common::PlotPanelPersistConfig& entry : saved) {
    auto* dock = frame_->findChild<PanelDockWidget*>(
        QString::fromStdString(entry.object_name));
    auto* panel = dock != nullptr
                      ? qobject_cast<plot::PlotPanel*>(dock->widget())
                      : nullptr;
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(plot::FromPersistConfig(entry));
    updatePlotDockTitle(dock, panel);
    if (entry.settings_visible && property_inspector_panel_ != nullptr) {
      setActivePlotPanel(panel);
      frame_->layout_->showPropertyInspector(true);
    }
  }
}

void FramePanels::captureImagePanelConfigs() {
  std::vector<common::ImagePanelPersistConfig> panels;
  panels.reserve(4);
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || panelTypeId(dock) != QLatin1String("ImageDock")) {
      continue;
    }
    auto* panel = qobject_cast<image::ImagePanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panels.push_back(image::ToPersistConfig(dock->objectName(), panel->config()));
  }
  frame_->manager_->setImagePanels(panels);
}

void FramePanels::ensureImageDockExists(const QString& object_name) {
  if (object_name.isEmpty() ||
      frame_->findChild<PanelDockWidget*>(object_name) != nullptr) {
    return;
  }
  PanelDockWidget* dock = createImagePanelDock(object_name);
  frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
}

void FramePanels::restoreImagePanelConfigs() {
  const std::vector<common::ImagePanelPersistConfig>& saved =
      frame_->manager_->imagePanels();
  if (saved.empty()) {
    if (image_panel_ != nullptr) {
      updateImageDockTitle(image_dock_, image_panel_);
    }
    return;
  }

  for (const common::ImagePanelPersistConfig& entry : saved) {
    ensureImageDockExists(QString::fromStdString(entry.object_name));
  }

  for (const common::ImagePanelPersistConfig& entry : saved) {
    auto* dock = frame_->findChild<PanelDockWidget*>(
        QString::fromStdString(entry.object_name));
    auto* panel = dock != nullptr
                      ? qobject_cast<image::ImagePanel*>(dock->widget())
                      : nullptr;
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(image::FromPersistConfig(entry));
    updateImageDockTitle(dock, panel);
    if (entry.settings_visible && property_inspector_panel_ != nullptr) {
      setActiveImagePanel(panel);
      frame_->layout_->showPropertyInspector(true);
    }
  }
}

void FramePanels::capturePublishPanelConfigs() {
  std::vector<common::PublishPanelPersistConfig> panels;
  panels.reserve(4);
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || panelTypeId(dock) != QLatin1String("PublishDock")) {
      continue;
    }
    auto* panel = qobject_cast<publish_panel::PublishPanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panels.push_back(
        publish_panel::ToPersistConfig(dock->objectName(), panel->config()));
  }
  frame_->manager_->setPublishPanels(panels);
}

void FramePanels::captureServicePanelConfigs() {
  std::vector<common::ServicePanelPersistConfig> panels;
  panels.reserve(4);
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || panelTypeId(dock) != QLatin1String("ServiceDock")) {
      continue;
    }
    auto* panel = qobject_cast<service_panel::ServicePanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panels.push_back(service_panel::ToPersistConfig(
        dock->objectName(), panel->config(), panel->settingsVisible()));
  }
  frame_->manager_->setServicePanels(panels);
}

void FramePanels::captureTeleopPanelConfigs() {
  std::vector<common::TeleopPanelPersistConfig> panels;
  panels.reserve(2);
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || panelTypeId(dock) != QLatin1String("TeleopDock")) {
      continue;
    }
    auto* panel = qobject_cast<teleop::TeleopPanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panels.push_back(
        teleop::ToPersistConfig(dock->objectName(), panel->config()));
  }
  frame_->manager_->setTeleopPanels(panels);
}

void FramePanels::captureMapPanelConfigs() {
  std::vector<common::MapPanelPersistConfig> panels;
  panels.reserve(2);
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || panelTypeId(dock) != QLatin1String("MapDock")) {
      continue;
    }
    auto* panel = qobject_cast<map::MapPanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panels.push_back(map::ToPersistConfig(dock->objectName(), panel->config(),
                                          panel->settingsVisible()));
  }
  frame_->manager_->setMapPanels(panels);
}

void FramePanels::restoreTeleopPanelConfigs() {
  const std::vector<common::TeleopPanelPersistConfig>& saved =
      frame_->manager_->teleopPanels();
  if (saved.empty()) {
    return;
  }
  for (const common::TeleopPanelPersistConfig& entry : saved) {
    const QString object_name = QString::fromStdString(entry.object_name);
    if (object_name.isEmpty()) {
      continue;
    }
    auto* dock = frame_->findChild<PanelDockWidget*>(object_name);
    if (dock == nullptr) {
      dock = createTeleopPanelDock(object_name);
      frame_->layout_->addSidebarDock(dock, Qt::RightDockWidgetArea);
    }
    auto* panel = qobject_cast<teleop::TeleopPanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(teleop::FromPersistConfig(entry));
    updateTeleopDockTitle(dock, panel);
    if (entry.settings_visible && property_inspector_panel_ != nullptr) {
      setActiveTeleopPanel(panel);
      frame_->layout_->showPropertyInspector(true);
    }
  }
}

void FramePanels::restoreMapPanelConfigs() {
  const std::vector<common::MapPanelPersistConfig>& saved =
      frame_->manager_->mapPanels();
  if (saved.empty()) {
    return;
  }
  for (const common::MapPanelPersistConfig& entry : saved) {
    const QString object_name = QString::fromStdString(entry.object_name);
    if (object_name.isEmpty()) {
      continue;
    }
    auto* dock = frame_->findChild<PanelDockWidget*>(object_name);
    if (dock == nullptr) {
      dock = createMapPanelDock(object_name);
      frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
    }
    auto* panel = qobject_cast<map::MapPanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(map::FromPersistConfig(entry));
    updateMapDockTitle(dock, panel);
    if (entry.settings_visible && property_inspector_panel_ != nullptr) {
      setActiveMapPanel(panel);
      frame_->layout_->showPropertyInspector(true);
    }
  }
}

void FramePanels::restoreServicePanelConfigs() {
  const std::vector<common::ServicePanelPersistConfig>& saved =
      frame_->manager_->servicePanels();
  if (saved.empty()) {
    return;
  }
  for (const common::ServicePanelPersistConfig& entry : saved) {
    const QString object_name = QString::fromStdString(entry.object_name);
    if (object_name.isEmpty()) {
      continue;
    }
    auto* dock = frame_->findChild<PanelDockWidget*>(object_name);
    if (dock == nullptr) {
      dock = createServicePanelDock(object_name);
      frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
    }
    auto* panel = qobject_cast<service_panel::ServicePanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(service_panel::FromPersistConfig(entry));
    updateServiceDockTitle(dock, panel);
    if (entry.settings_visible && property_inspector_panel_ != nullptr) {
      setActiveServicePanel(panel);
      frame_->layout_->showPropertyInspector(true);
    }
  }
}

void FramePanels::ensurePublishDockExists(const QString& object_name) {
  if (object_name.isEmpty() ||
      frame_->findChild<PanelDockWidget*>(object_name) != nullptr) {
    return;
  }
  PanelDockWidget* dock = createPublishPanelDock(object_name);
  frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
}

void FramePanels::restorePublishPanelConfigs() {
  const std::vector<common::PublishPanelPersistConfig>& saved =
      frame_->manager_->publishPanels();
  if (saved.empty()) {
    return;
  }

  for (const common::PublishPanelPersistConfig& entry : saved) {
    ensurePublishDockExists(QString::fromStdString(entry.object_name));
  }

  for (const common::PublishPanelPersistConfig& entry : saved) {
    auto* dock = frame_->findChild<PanelDockWidget*>(
        QString::fromStdString(entry.object_name));
    auto* panel = dock != nullptr
                      ? qobject_cast<publish_panel::PublishPanel*>(dock->widget())
                      : nullptr;
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(publish_panel::FromPersistConfig(entry));
    updatePublishDockTitle(dock, panel);
    if (entry.settings_visible && property_inspector_panel_ != nullptr) {
      setActivePublishPanel(panel);
      frame_->layout_->showPropertyInspector(true);
    }
  }
}

void FramePanels::refreshAllPlotSettingsChannels() {
  for (PanelDockWidget* dock : frame_->findChildren<PanelDockWidget*>()) {
    const QString type = panelTypeId(dock);
    if (type == QLatin1String("PlotDock")) {
      if (auto* panel = qobject_cast<plot::PlotPanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    } else if (type == QLatin1String("ImageDock")) {
      if (auto* panel = qobject_cast<image::ImagePanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    } else if (type == QLatin1String("PublishDock")) {
      if (auto* panel = qobject_cast<publish_panel::PublishPanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    } else if (type == QLatin1String("ServiceDock")) {
      if (auto* panel = qobject_cast<service_panel::ServicePanel*>(dock->widget())) {
        panel->refreshServices();
      }
    } else if (type == QLatin1String("MapDock")) {
      if (auto* panel = qobject_cast<map::MapPanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    }
  }
}

void FramePanels::applyPlotSettingsVisibilityFromSession() {
  if (!frame_->manager_->plotPanels().empty()) {
    return;
  }
  frame_->layout_->showPropertyInspector(frame_->manager_->plotSettingsVisible());
}

void FramePanels::installPlotFocusTracking() {
  QObject::connect(qApp, &QApplication::focusChanged, frame_, [this](QWidget* /*old_focus*/, QWidget* new_focus) {
            if (new_focus == nullptr) {
              return;
            }
            for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
              if (panelTypeId(dock) != QLatin1String("PlotDock") || !dock->isVisible()) {
                continue;
              }
              if (dock->isAncestorOf(new_focus)) {
                if (auto* panel = qobject_cast<plot::PlotPanel*>(dock->widget())) {
                  setActivePlotPanel(panel);
                }
                break;
              }
            }
          });
}

void FramePanels::installImageFocusTracking() {
  QObject::connect(qApp, &QApplication::focusChanged, frame_, [this](QWidget* /*old_focus*/, QWidget* new_focus) {
            if (new_focus == nullptr) {
              return;
            }
            for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
              if (panelTypeId(dock) != QLatin1String("ImageDock") || !dock->isVisible()) {
                continue;
              }
              if (dock->isAncestorOf(new_focus)) {
                if (auto* panel = qobject_cast<image::ImagePanel*>(dock->widget())) {
                  setActiveImagePanel(panel);
                }
                break;
              }
            }
          });
}

void FramePanels::setActivePlotPanel(plot::PlotPanel* panel) {
  if (active_plot_panel_ == panel) {
    if (panel != nullptr) {
      bindPlotToPropertyInspector(panel);
    }
    return;
  }
  if (active_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(active_plot_panel_);
  }
  if (panel != nullptr && active_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(active_image_panel_);
    active_image_panel_ = nullptr;
  }
  if (panel != nullptr && active_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(active_teleop_panel_);
    active_teleop_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  active_plot_panel_ = panel;
  if (panel != nullptr) {
    bindPlotToPropertyInspector(panel);
  }
}

void FramePanels::setActiveImagePanel(image::ImagePanel* panel) {
  if (active_image_panel_ == panel) {
    if (panel != nullptr) {
      bindImageToPropertyInspector(panel);
    }
    return;
  }
  if (active_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(active_image_panel_);
  }
  if (panel != nullptr && active_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(active_plot_panel_);
    active_plot_panel_ = nullptr;
  }
  if (panel != nullptr && active_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(active_teleop_panel_);
    active_teleop_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  active_image_panel_ = panel;
  if (panel != nullptr) {
    bindImageToPropertyInspector(panel);
  }
}

void FramePanels::setActiveTeleopPanel(teleop::TeleopPanel* panel) {
  if (active_teleop_panel_ == panel) {
    if (panel != nullptr) {
      bindTeleopToPropertyInspector(panel);
    }
    return;
  }
  if (active_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(active_teleop_panel_);
  }
  if (panel != nullptr && active_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(active_plot_panel_);
    active_plot_panel_ = nullptr;
  }
  if (panel != nullptr && active_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(active_image_panel_);
    active_image_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  active_teleop_panel_ = panel;
  if (panel != nullptr) {
    bindTeleopToPropertyInspector(panel);
  }
}

void FramePanels::activatePanelDock(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  if (auto* plot = qobject_cast<plot::PlotPanel*>(dock->widget())) {
    setActivePlotPanel(plot);
  } else if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
    setActiveImagePanel(image);
  } else if (auto* teleop = qobject_cast<teleop::TeleopPanel*>(dock->widget())) {
    setActiveTeleopPanel(teleop);
  } else if (auto* publish = qobject_cast<publish_panel::PublishPanel*>(dock->widget())) {
    setActivePublishPanel(publish);
  } else if (auto* service = qobject_cast<service_panel::ServicePanel*>(dock->widget())) {
    setActiveServicePanel(service);
  } else   if (auto* map_panel = qobject_cast<map::MapPanel*>(dock->widget())) {
    setActiveMapPanel(map_panel);
  }
  if (property_inspector_panel_ != nullptr &&
      property_inspector_panel_->contentWidget() != nullptr) {
    frame_->layout_->raiseLeftSidebarProperties();
  }
}

void FramePanels::setActivePublishPanel(publish_panel::PublishPanel* panel) {
  if (active_publish_panel_ == panel) {
    if (panel != nullptr) {
      bindPublishToPropertyInspector(panel);
    }
    return;
  }
  if (active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
  }
  if (panel != nullptr && active_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(active_plot_panel_);
    active_plot_panel_ = nullptr;
  }
  if (panel != nullptr && active_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(active_image_panel_);
    active_image_panel_ = nullptr;
  }
  if (panel != nullptr && active_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(active_teleop_panel_);
    active_teleop_panel_ = nullptr;
  }
  if (panel != nullptr && active_service_panel_ != nullptr) {
    clearPropertyInspectorForService(active_service_panel_);
    active_service_panel_ = nullptr;
  }
  active_publish_panel_ = panel;
  if (panel != nullptr) {
    bindPublishToPropertyInspector(panel);
  }
}

void FramePanels::setActiveServicePanel(service_panel::ServicePanel* panel) {
  if (active_service_panel_ == panel) {
    if (panel != nullptr) {
      bindServiceToPropertyInspector(panel);
    }
    return;
  }
  if (active_service_panel_ != nullptr) {
    clearPropertyInspectorForService(active_service_panel_);
  }
  if (panel != nullptr && active_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(active_plot_panel_);
    active_plot_panel_ = nullptr;
  }
  if (panel != nullptr && active_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(active_image_panel_);
    active_image_panel_ = nullptr;
  }
  if (panel != nullptr && active_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(active_teleop_panel_);
    active_teleop_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  active_service_panel_ = panel;
  if (panel != nullptr) {
    bindServiceToPropertyInspector(panel);
  }
}

void FramePanels::setActiveMapPanel(map::MapPanel* panel) {
  if (active_map_panel_ == panel) {
    if (panel != nullptr) {
      bindMapToPropertyInspector(panel);
    }
    return;
  }
  if (active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
  }
  if (panel != nullptr && active_plot_panel_ != nullptr) {
    clearPropertyInspectorForPlot(active_plot_panel_);
    active_plot_panel_ = nullptr;
  }
  if (panel != nullptr && active_image_panel_ != nullptr) {
    clearPropertyInspectorForImage(active_image_panel_);
    active_image_panel_ = nullptr;
  }
  if (panel != nullptr && active_teleop_panel_ != nullptr) {
    clearPropertyInspectorForTeleop(active_teleop_panel_);
    active_teleop_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  active_map_panel_ = panel;
  if (panel != nullptr) {
    bindMapToPropertyInspector(panel);
  }
}

void FramePanels::wirePlotPanel(PanelDockWidget* dock,
                                       plot::PlotPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &plot::PlotPanel::activated, frame_, [this, panel]() { setActivePlotPanel(panel); });
  QObject::connect(panel, &plot::PlotPanel::settingsToggled, frame_, [this, panel](bool visible) {
            setActivePlotPanel(panel);
            frame_->layout_->showPropertyInspector(visible);
            frame_->session_->markConfigModified();
          });
  QObject::connect(panel, &plot::PlotPanel::panelSplitRequested, frame_, [this, dock](Qt::Orientation orientation) {
            frame_->layout_->onSplitActiveDock(dock, orientation);
          });
  QObject::connect(panel, &plot::PlotPanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  QObject::connect(panel, &plot::PlotPanel::panelExpandRequested, frame_, [this, dock]() { frame_->layout_->expandPanelDock(dock); });
  QObject::connect(panel, &plot::PlotPanel::panelChangeRequested, frame_, [this, dock](const QString& object_name) {
            frame_->layout_->changePanelInDock(dock, object_name);
          });
  QObject::connect(panel, &plot::PlotPanel::configChanged, frame_, [this, dock, panel]() {
    updatePlotDockTitle(dock, panel);
    if (panel == active_plot_panel_ && property_inspector_panel_ != nullptr) {
      const QString title = panel->config().title.trimmed();
      property_inspector_panel_->setContentWidget(
          panel->settingsWidgetForInspector(),
          title.isEmpty() ? frame_->tr("Plot") : title);
    }
    frame_->manager_->setPlotSettingsVisible(frame_->layout_->isPropertyInspectorVisible());
    frame_->session_->markConfigModified();
  });
  QObject::connect(dock, &QDockWidget::visibilityChanged, frame_, [this, dock, panel](bool visible) {
            if (visible || panel != active_plot_panel_) {
              return;
            }
            plot::PlotPanel* fallback = nullptr;
            for (PanelDockWidget* candidate : frame_->layout_->orderedDockWidgets()) {
              if (candidate == nullptr || candidate == dock ||
                  panelTypeId(candidate) != QLatin1String("PlotDock") ||
                  !candidate->isVisible()) {
                continue;
              }
              fallback = qobject_cast<plot::PlotPanel*>(candidate->widget());
              if (fallback != nullptr) {
                break;
              }
            }
            setActivePlotPanel(fallback);
          });
}

PanelDockWidget* FramePanels::createPlotPanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("PlotDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Plot"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("PlotDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelPlot")));
  auto* panel = new plot::PlotPanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wirePlotPanel(dock, panel);
  updatePlotDockTitle(dock, panel);
  frame_->layout_->configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void FramePanels::updateTableDockTitle(PanelDockWidget* dock,
                                       table_panel::TablePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Table") : title);
}

void FramePanels::captureTablePanelConfigs() {
  std::vector<common::TablePanelPersistConfig> panels;
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || panelTypeId(dock) != QLatin1String("TableDock")) {
      continue;
    }
    auto* panel = qobject_cast<table_panel::TablePanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panels.push_back(
        table_panel::ToPersistConfig(dock->objectName(), panel->config()));
  }
  frame_->manager_->setTablePanels(panels);
}

void FramePanels::captureChannelGraphPanelConfigs() {
  std::vector<common::ChannelGraphPanelPersistConfig> panels;
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr ||
        panelTypeId(dock) != QLatin1String("ChannelGraphDock")) {
      continue;
    }
    auto* panel = qobject_cast<channel_graph::ChannelGraphPanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panels.push_back(
        channel_graph::ToPersistConfig(dock->objectName(), panel->config()));
  }
  frame_->manager_->setChannelGraphPanels(panels);
}

void FramePanels::captureTfTreePanelConfigs() {
  std::vector<common::TfTreePanelPersistConfig> panels;
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || panelTypeId(dock) != QLatin1String("TfTreeDock")) {
      continue;
    }
    auto* panel = qobject_cast<TfTreePanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    common::TfTreePanelPersistConfig entry = panel->config();
    entry.object_name = dock->objectName().toStdString();
    panels.push_back(std::move(entry));
  }
  frame_->manager_->setTfTreePanels(panels);
}

void FramePanels::restoreTablePanelConfigs() {
  const std::vector<common::TablePanelPersistConfig>& saved =
      frame_->manager_->tablePanels();
  if (saved.empty()) {
    return;
  }
  for (const common::TablePanelPersistConfig& entry : saved) {
    const QString object_name = QString::fromStdString(entry.object_name);
    if (object_name.isEmpty()) {
      continue;
    }
    auto* dock = frame_->findChild<PanelDockWidget*>(object_name);
    if (dock == nullptr) {
      dock = createTablePanelDock(object_name);
      frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
    }
    auto* panel = qobject_cast<table_panel::TablePanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(table_panel::FromPersistConfig(entry));
    updateTableDockTitle(dock, panel);
  }
}

void FramePanels::restoreChannelGraphPanelConfigs() {
  const std::vector<common::ChannelGraphPanelPersistConfig>& saved =
      frame_->manager_->channelGraphPanels();
  if (saved.empty()) {
    return;
  }
  for (const common::ChannelGraphPanelPersistConfig& entry : saved) {
    const QString object_name = QString::fromStdString(entry.object_name);
    if (object_name.isEmpty()) {
      continue;
    }
    auto* dock = frame_->findChild<PanelDockWidget*>(object_name);
    if (dock == nullptr) {
      dock = createChannelGraphPanelDock(object_name);
      frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
    }
    auto* panel =
        qobject_cast<channel_graph::ChannelGraphPanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(channel_graph::FromPersistConfig(entry));
  }
}

void FramePanels::restoreTfTreePanelConfigs() {
  const std::vector<common::TfTreePanelPersistConfig>& saved =
      frame_->manager_->tfTreePanels();
  if (saved.empty()) {
    return;
  }
  for (const common::TfTreePanelPersistConfig& entry : saved) {
    const QString object_name = QString::fromStdString(entry.object_name);
    if (object_name.isEmpty()) {
      continue;
    }
    auto* dock = frame_->findChild<PanelDockWidget*>(object_name);
    if (dock == nullptr) {
      dock = createTfTreePanelDock(object_name);
      frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
    }
    auto* panel = qobject_cast<TfTreePanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(entry);
  }
}

void FramePanels::wireTablePanel(PanelDockWidget* dock,
                                 table_panel::TablePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &table_panel::TablePanel::activated, frame_,
                   [this]() {});
  QObject::connect(panel, &table_panel::TablePanel::panelSplitRequested, frame_,
                   [this, dock](Qt::Orientation orientation) {
                     frame_->layout_->onSplitActiveDock(dock, orientation);
                   });
  QObject::connect(panel, &table_panel::TablePanel::panelRemoveRequested, dock,
                   &QDockWidget::close);
  QObject::connect(panel, &table_panel::TablePanel::panelExpandRequested, frame_,
                   [this, dock]() { frame_->layout_->expandPanelDock(dock); });
  QObject::connect(panel, &table_panel::TablePanel::panelChangeRequested, frame_,
                   [this, dock](const QString& object_name) {
                     frame_->layout_->changePanelInDock(dock, object_name);
                   });
  QObject::connect(panel, &table_panel::TablePanel::configChanged, frame_,
                   [this, dock, panel]() {
                     updateTableDockTitle(dock, panel);
                     frame_->session_->markConfigModified();
                   });
}

PanelDockWidget* FramePanels::createTablePanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("TableDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Table"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("TableDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelTable")));
  auto* panel = new table_panel::TablePanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireTablePanel(dock, panel);
  updateTableDockTitle(dock, panel);
  frame_->layout_->configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void FramePanels::updateImageDockTitle(PanelDockWidget* dock,
                                              image::ImagePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Image") : title);
}

void FramePanels::wireImagePanel(PanelDockWidget* dock,
                                      image::ImagePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &image::ImagePanel::activated, frame_, [this, panel]() { setActiveImagePanel(panel); });
  QObject::connect(panel, &image::ImagePanel::settingsToggled, frame_, [this, panel](bool visible) {
            setActiveImagePanel(panel);
            frame_->layout_->showPropertyInspector(visible);
            frame_->session_->markConfigModified();
          });
  QObject::connect(panel, &image::ImagePanel::panelSplitRequested, frame_, [this, dock](Qt::Orientation orientation) {
            frame_->layout_->onSplitActiveDock(dock, orientation);
          });
  QObject::connect(panel, &image::ImagePanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  QObject::connect(panel, &image::ImagePanel::panelExpandRequested, frame_, [this, dock]() { frame_->layout_->expandPanelDock(dock); });
  QObject::connect(panel, &image::ImagePanel::panelChangeRequested, frame_, [this, dock](const QString& object_name) {
            frame_->layout_->changePanelInDock(dock, object_name);
          });
  QObject::connect(panel, &image::ImagePanel::configChanged, frame_, [this, dock, panel]() {
    updateImageDockTitle(dock, panel);
    if (panel == active_image_panel_ && property_inspector_panel_ != nullptr) {
      const QString title = panel->config().title.trimmed();
      property_inspector_panel_->setContentWidget(
          panel->settingsWidgetForInspector(),
          title.isEmpty() ? frame_->tr("Image") : title);
    }
    frame_->session_->markConfigModified();
  });
  QObject::connect(dock, &QDockWidget::visibilityChanged, frame_, [this, dock, panel](bool visible) {
            if (visible || panel != active_image_panel_) {
              return;
            }
            image::ImagePanel* fallback = nullptr;
            for (PanelDockWidget* candidate : frame_->layout_->orderedDockWidgets()) {
              if (candidate == nullptr || candidate == dock ||
                  panelTypeId(candidate) != QLatin1String("ImageDock") ||
                  !candidate->isVisible()) {
                continue;
              }
              fallback = qobject_cast<image::ImagePanel*>(candidate->widget());
              if (fallback != nullptr) {
                break;
              }
            }
            setActiveImagePanel(fallback);
          });
}

PanelDockWidget* FramePanels::createImagePanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("ImageDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Image"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("ImageDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelImage")));
  auto* panel = new image::ImagePanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireImagePanel(dock, panel);
  updateImageDockTitle(dock, panel);
  frame_->layout_->configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void FramePanels::syncImageDisplayWindows() {
  if (frame_ == nullptr || frame_->manager_ == nullptr ||
      frame_->layout_ == nullptr) {
    return;
  }

  const auto disableImageDisplayByName = [this](const QString& name) {
    if (frame_ == nullptr || frame_->manager_ == nullptr || name.isEmpty()) {
      return;
    }
    const auto& displays = frame_->manager_->displays();
    for (std::size_t i = 0; i < displays.size(); ++i) {
      display::Display* top = displays[i];
      if (top == nullptr) {
        continue;
      }
      if (top->typeId() == "Image" &&
          QString::fromStdString(top->name()) == name) {
        if (top->enabled()) {
          frame_->manager_->setDisplayEnabled(i, false, -1);
        }
        return;
      }
      if (auto* group = dynamic_cast<display::DisplayGroup*>(top)) {
        const auto& children = group->children();
        for (std::size_t c = 0; c < children.size(); ++c) {
          display::Display* child = children[c].get();
          if (child != nullptr && child->typeId() == "Image" &&
              QString::fromStdString(child->name()) == name) {
            if (child->enabled()) {
              frame_->manager_->setDisplayEnabled(i, false,
                                                  static_cast<int>(c));
            }
            return;
          }
        }
      }
    }
  };

  const auto destroyDockQuietly = [this](PanelDockWidget* dock) {
    if (dock == nullptr) {
      return;
    }
    dock->setProperty("suppressImageDisplayDisable", true);
    unregisterDeletePanelAction(dock);
    if (frame_->layout_->main_panel_host_ != nullptr &&
        frame_->layout_->main_panel_host_->hostsPanel(dock)) {
      frame_->layout_->main_panel_host_->removePanel(dock);
    }
    if (QMainWindow* host = frame_->layout_->dockHostForPanel(dock)) {
      if (host->dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
        host->removeDockWidget(dock);
      }
    }
    if (frame_->dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
      frame_->removeDockWidget(dock);
    }
    if (frame_->layout_->expanded_main_panel_dock_ == dock) {
      frame_->layout_->expanded_main_panel_dock_ = nullptr;
    }
    if (frame_->layout_->last_active_dock_ == dock) {
      frame_->layout_->last_active_dock_ = nullptr;
    }
    dock->deleteLater();
  };

  QSet<QString> wanted;
  const auto consider = [&](display::Display* display) {
    if (display == nullptr || display->typeId() != "Image") {
      return;
    }
    const QString name = QString::fromStdString(display->name());
    if (name.isEmpty() || !display->enabled()) {
      return;
    }
    wanted.insert(name);

    QPointer<PanelDockWidget>& dock = image_display_docks_[name];
    if (!dock.isNull() && !dock->property("panelDisposed").toBool()) {
      dock->setPanelTitle(name);
      if (!dock->isVisible()) {
        // Re-attach after a previous close (flex dock stays on the frame).
        if (frame_->dockWidgetArea(dock) == Qt::NoDockWidgetArea &&
            (frame_->layout_->main_panel_host_ == nullptr ||
             !frame_->layout_->main_panel_host_->hostsPanel(dock))) {
          frame_->layout_->configureFlexibleDock(dock);
          frame_->addDockWidget(Qt::LeftDockWidgetArea, dock);
        }
        dock->show();
        dock->raise();
      }
      return;
    }

    const QString object_name =
        uniquePanelObjectName(QStringLiteral("ImageDisplayDock"));
    dock = new PanelDockWidget(name, frame_);
    dock->setObjectName(object_name);
    dock->setProperty("panelTypeId", QStringLiteral("ImageDisplayDock"));
    dock->setProperty("imageDisplayName", name);
    dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelImage")));

    auto* view = new image::ImageViewWidget(dock);
    view->setObjectName(QStringLiteral("ImageDisplayView"));
    view->setBackgroundColor(Qt::white);
    view->setStatusText(frame_->tr("No Image"));
    dock->setContentWidget(view);

    // Flexible outer-frame dock: Qt native drag / 吸附 to any dock area.
    // (MainPanelHost path uses NoDockWidgetArea and blocks arbitrary snap.)
    frame_->layout_->configureFlexibleDock(dock);

    QObject::connect(dock, &PanelDockWidget::closed, frame_,
                     [this, name, disableImageDisplayByName]() {
                       const QPointer<PanelDockWidget> closing =
                           image_display_docks_.value(name);
                       const bool suppressed =
                           !closing.isNull() &&
                           closing->property("suppressImageDisplayDisable")
                               .toBool();
                       image_display_docks_.remove(name);
                       if (suppressed) {
                         return;
                       }
                       disableImageDisplayByName(name);
                       if (displays_panel_ != nullptr) {
                         // Rebuild tree so the Displays checkbox unchecks.
                         displays_panel_->refresh();
                       }
                       // Tear down the closed dock (not a MainPanel drop_duplicate).
                       if (!closing.isNull()) {
                         closing->setProperty("suppressImageDisplayDisable",
                                              true);
                         unregisterDeletePanelAction(closing.data());
                         if (frame_->dockWidgetArea(closing.data()) !=
                             Qt::NoDockWidgetArea) {
                           frame_->removeDockWidget(closing.data());
                         }
                         closing->deleteLater();
                       }
                     });

    registerPanelDock(dock);
    frame_->addDockWidget(Qt::LeftDockWidgetArea, dock);
    if (displays_dock_ != nullptr &&
        frame_->dockWidgetArea(displays_dock_) == Qt::LeftDockWidgetArea) {
      // Prefer splitting beside Displays rather than burying under a tab.
      frame_->splitDockWidget(displays_dock_, dock, Qt::Vertical);
    }
    dock->show();
    dock->raise();
  };

  const auto& displays = frame_->manager_->displays();
  for (std::size_t i = 0; i < displays.size(); ++i) {
    display::Display* top = displays[i];
    consider(top);
    if (auto* group = dynamic_cast<display::DisplayGroup*>(top)) {
      const auto& children = group->children();
      for (std::size_t c = 0; c < children.size(); ++c) {
        consider(children[c].get());
      }
    }
  }

  for (auto it = image_display_docks_.begin();
       it != image_display_docks_.end();) {
    if (wanted.contains(it.key())) {
      ++it;
      continue;
    }
    PanelDockWidget* dock = it.value().data();
    it = image_display_docks_.erase(it);
    destroyDockQuietly(dock);
  }
}

void FramePanels::updateImageDisplayWindowFrame(const QString& source,
                                                const QImage& image) {
  if (source.isEmpty() || image.isNull()) {
    return;
  }
  const auto it = image_display_docks_.constFind(source);
  if (it == image_display_docks_.constEnd() || it.value().isNull()) {
    return;
  }
  auto* view = it.value()->findChild<image::ImageViewWidget*>(
      QStringLiteral("ImageDisplayView"));
  if (view == nullptr) {
    view = qobject_cast<image::ImageViewWidget*>(it.value()->widget());
  }
  if (view != nullptr) {
    view->setFrame(image);
  }
}

void FramePanels::updateTeleopDockTitle(PanelDockWidget* dock,
                                             teleop::TeleopPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Teleop") : title);
}

void FramePanels::wireTeleopPanel(PanelDockWidget* dock,
                                       teleop::TeleopPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &teleop::TeleopPanel::activated, frame_, [this, panel]() { setActiveTeleopPanel(panel); });
  QObject::connect(panel, &teleop::TeleopPanel::settingsToggled, frame_, [this, panel](bool visible) {
            setActiveTeleopPanel(panel);
            frame_->layout_->showPropertyInspector(visible);
            frame_->session_->markConfigModified();
          });
  QObject::connect(panel, &teleop::TeleopPanel::panelSplitRequested, frame_, [this, dock](Qt::Orientation orientation) {
            frame_->layout_->onSplitActiveDock(dock, orientation);
          });
  QObject::connect(panel, &teleop::TeleopPanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  QObject::connect(panel, &teleop::TeleopPanel::panelChangeRequested, frame_, [this, dock](const QString& object_name) {
            frame_->layout_->changePanelInDock(dock, object_name);
          });
  QObject::connect(panel, &teleop::TeleopPanel::configChanged, frame_, [this, dock, panel]() {
            updateTeleopDockTitle(dock, panel);
            if (panel == active_teleop_panel_ &&
                property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? frame_->tr("Teleop") : title);
            }
            frame_->session_->markConfigModified();
          });
  QObject::connect(dock, &QDockWidget::visibilityChanged, frame_, [this, dock, panel](bool visible) {
            if (visible || panel != active_teleop_panel_) {
              return;
            }
            teleop::TeleopPanel* fallback = nullptr;
            for (PanelDockWidget* candidate : frame_->layout_->orderedDockWidgets()) {
              if (candidate == nullptr || candidate == dock ||
                  panelTypeId(candidate) != QLatin1String("TeleopDock") ||
                  !candidate->isVisible()) {
                continue;
              }
              fallback = qobject_cast<teleop::TeleopPanel*>(candidate->widget());
              if (fallback != nullptr) {
                break;
              }
            }
            setActiveTeleopPanel(fallback);
          });
}

PanelDockWidget* FramePanels::createTeleopPanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("TeleopDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Teleop"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("TeleopDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelTeleop")));
  auto* panel = new teleop::TeleopPanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireTeleopPanel(dock, panel);
  updateTeleopDockTitle(dock, panel);
  // Teleop lives in the right sidebar; can drag/snap to left as well.
  frame_->layout_->configureSidebarDock(dock, Qt::RightDockWidgetArea);
  registerPanelDock(dock);
  if (dock_name == QLatin1String("TeleopDock")) {
    teleop_dock_ = dock;
  }
  return dock;
}

void FramePanels::wireRecordPanel(PanelDockWidget* dock, RecordPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &RecordPanel::openRecordRequested, frame_,
                   &VisualizationFrame::onOpenRecord);
  QObject::connect(
      panel, &RecordPanel::openInRawMessagesRequested, frame_,
      [this](const QString& channel) {
        if (channel_dock_ != nullptr) {
          channel_dock_->show();
          channel_dock_->raise();
        }
        if (raw_messages_panel_ != nullptr) {
          raw_messages_panel_->selectChannel(channel);
        }
      });
  QObject::connect(panel, &RecordPanel::panelRemoveRequested, dock,
                   &QDockWidget::close);
  QObject::connect(panel, &RecordPanel::panelExpandRequested, frame_,
                   [this, dock]() { frame_->layout_->expandPanelDock(dock); });
  QObject::connect(panel, &RecordPanel::panelChangeRequested, frame_,
                   [this, dock](const QString& object_name) {
                     frame_->layout_->changePanelInDock(dock, object_name);
                   });
}

PanelDockWidget* FramePanels::createRecordPanelDock(
    const QString& object_name) {
  if (record_dock_ != nullptr &&
      (object_name.isEmpty() ||
       object_name == QLatin1String("RecordDock"))) {
    return record_dock_;
  }
  const QString dock_name =
      object_name.isEmpty() ? QStringLiteral("RecordDock") : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Record"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("RecordDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelRecord")));
  auto* panel = new RecordPanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireRecordPanel(dock, panel);
  frame_->layout_->configureSidebarDock(dock, Qt::RightDockWidgetArea);
  registerPanelDock(dock);
  if (dock_name == QLatin1String("RecordDock")) {
    record_dock_ = dock;
    record_panel_ = panel;
  }
  return dock;
}

void FramePanels::wireTfTreePanel(PanelDockWidget* dock, TfTreePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &TfTreePanel::panelRemoveRequested, dock, &QDockWidget::close);
  QObject::connect(panel, &TfTreePanel::panelExpandRequested, frame_, [this, dock]() { frame_->layout_->expandPanelDock(dock); });
  QObject::connect(panel, &TfTreePanel::panelSplitRequested, frame_, [this, dock](Qt::Orientation orientation) {
            frame_->layout_->onSplitActiveDock(dock, orientation);
          });
  QObject::connect(panel, &TfTreePanel::panelChangeRequested, frame_, [this, dock](const QString& object_name) {
            frame_->layout_->changePanelInDock(dock, object_name);
          });
  QObject::connect(panel, &TfTreePanel::configChanged, frame_,
                   [this]() { frame_->session_->markConfigModified(); });
}

PanelDockWidget* FramePanels::createTfTreePanelDock(const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("TfTreeDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Transform Tree"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("TfTreeDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelTransformTree")));
  auto* panel = new TfTreePanel(frame_->manager_->tfBuffer(), frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireTfTreePanel(dock, panel);
  frame_->layout_->configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void FramePanels::updatePublishDockTitle(
    PanelDockWidget* dock, publish_panel::PublishPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Publish") : title);
}

void FramePanels::wirePublishPanel(PanelDockWidget* dock,
                                          publish_panel::PublishPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &publish_panel::PublishPanel::activated, frame_, [this, panel]() { setActivePublishPanel(panel); });
  QObject::connect(panel, &publish_panel::PublishPanel::settingsToggled, frame_, [this, panel](bool visible) {
            setActivePublishPanel(panel);
            frame_->layout_->showPropertyInspector(visible);
            frame_->session_->markConfigModified();
          });
  QObject::connect(panel, &publish_panel::PublishPanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  QObject::connect(panel, &publish_panel::PublishPanel::panelExpandRequested, frame_, [this, dock]() { frame_->layout_->expandPanelDock(dock); });
  QObject::connect(panel, &publish_panel::PublishPanel::panelSplitRequested, frame_, [this, dock](Qt::Orientation orientation) {
            frame_->layout_->onSplitActiveDock(dock, orientation);
          });
  QObject::connect(panel, &publish_panel::PublishPanel::panelChangeRequested, frame_, [this, dock](const QString& object_name) {
            frame_->layout_->changePanelInDock(dock, object_name);
          });
  QObject::connect(panel, &publish_panel::PublishPanel::configChanged, frame_, [this, dock, panel]() {
            updatePublishDockTitle(dock, panel);
            if (panel == active_publish_panel_ &&
                property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? frame_->tr("Publish") : title);
            }
            frame_->session_->markConfigModified();
          });
  QObject::connect(dock, &QDockWidget::visibilityChanged, frame_, [this, dock, panel](bool visible) {
            if (visible || panel != active_publish_panel_) {
              return;
            }
            publish_panel::PublishPanel* fallback = nullptr;
            for (PanelDockWidget* candidate : frame_->layout_->orderedDockWidgets()) {
              if (candidate == nullptr || candidate == dock ||
                  panelTypeId(candidate) != QLatin1String("PublishDock") ||
                  !candidate->isVisible()) {
                continue;
              }
              fallback =
                  qobject_cast<publish_panel::PublishPanel*>(candidate->widget());
              if (fallback != nullptr) {
                break;
              }
            }
            setActivePublishPanel(fallback);
          });
}

PanelDockWidget* FramePanels::createPublishPanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("PublishDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Publish"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("PublishDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelPublish")));
  auto* panel = new publish_panel::PublishPanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wirePublishPanel(dock, panel);
  updatePublishDockTitle(dock, panel);
  frame_->layout_->configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void FramePanels::updateServiceDockTitle(PanelDockWidget* dock,
                                              service_panel::ServicePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Service Call") : title);
}

void FramePanels::wireServicePanel(PanelDockWidget* dock,
                                          service_panel::ServicePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &service_panel::ServicePanel::activated, frame_, [this, panel]() { setActiveServicePanel(panel); });
  QObject::connect(panel, &service_panel::ServicePanel::settingsToggled, frame_, [this, panel](bool visible) {
            setActiveServicePanel(panel);
            frame_->layout_->showPropertyInspector(visible);
            frame_->session_->markConfigModified();
          });
  QObject::connect(panel, &service_panel::ServicePanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  QObject::connect(panel, &service_panel::ServicePanel::panelExpandRequested, frame_, [this, dock]() { frame_->layout_->expandPanelDock(dock); });
  QObject::connect(panel, &service_panel::ServicePanel::panelSplitRequested, frame_, [this, dock](Qt::Orientation orientation) {
            frame_->layout_->onSplitActiveDock(dock, orientation);
          });
  QObject::connect(panel, &service_panel::ServicePanel::panelChangeRequested, frame_, [this, dock](const QString& object_name) {
            frame_->layout_->changePanelInDock(dock, object_name);
          });
  QObject::connect(panel, &service_panel::ServicePanel::configChanged, frame_, [this, dock, panel]() {
            updateServiceDockTitle(dock, panel);
            if (panel == active_service_panel_ &&
                property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? frame_->tr("Service Call") : title);
            }
            frame_->session_->markConfigModified();
          });
  QObject::connect(dock, &QDockWidget::visibilityChanged, frame_, [this, dock, panel](bool visible) {
            if (visible || panel != active_service_panel_) {
              return;
            }
            service_panel::ServicePanel* fallback = nullptr;
            for (PanelDockWidget* candidate : frame_->layout_->orderedDockWidgets()) {
              if (candidate == nullptr || candidate == dock ||
                  panelTypeId(candidate) != QLatin1String("ServiceDock") ||
                  !candidate->isVisible()) {
                continue;
              }
              fallback =
                  qobject_cast<service_panel::ServicePanel*>(candidate->widget());
              if (fallback != nullptr) {
                break;
              }
            }
            setActiveServicePanel(fallback);
          });
}

PanelDockWidget* FramePanels::createServicePanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("ServiceDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Service Call"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("ServiceDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelService")));
  auto* panel = new service_panel::ServicePanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireServicePanel(dock, panel);
  updateServiceDockTitle(dock, panel);
  frame_->layout_->configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void FramePanels::updateMapDockTitle(PanelDockWidget* dock,
                                            map::MapPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Map") : title);
}

void FramePanels::wireMapPanel(PanelDockWidget* dock, map::MapPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &map::MapPanel::activated, frame_, [this, panel]() { setActiveMapPanel(panel); });
  QObject::connect(panel, &map::MapPanel::settingsToggled, frame_, [this, panel](bool visible) {
            setActiveMapPanel(panel);
            frame_->layout_->showPropertyInspector(visible);
            frame_->session_->markConfigModified();
          });
  QObject::connect(panel, &map::MapPanel::panelRemoveRequested, dock, &QDockWidget::close);
  QObject::connect(panel, &map::MapPanel::panelExpandRequested, frame_, [this, dock]() { frame_->layout_->expandPanelDock(dock); });
  QObject::connect(panel, &map::MapPanel::panelSplitRequested, frame_, [this, dock](Qt::Orientation orientation) {
            frame_->layout_->onSplitActiveDock(dock, orientation);
          });
  QObject::connect(panel, &map::MapPanel::panelChangeRequested, frame_, [this, dock](const QString& object_name) {
            frame_->layout_->changePanelInDock(dock, object_name);
          });
  QObject::connect(panel, &map::MapPanel::configChanged, frame_, [this, dock, panel]() {
            updateMapDockTitle(dock, panel);
            if (panel == active_map_panel_ && property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? frame_->tr("Map") : title);
            }
            frame_->session_->markConfigModified();
          });
  QObject::connect(dock, &QDockWidget::visibilityChanged, frame_, [this, dock, panel](bool visible) {
            if (visible || panel != active_map_panel_) {
              return;
            }
            map::MapPanel* fallback = nullptr;
            for (PanelDockWidget* candidate : frame_->layout_->orderedDockWidgets()) {
              if (candidate == nullptr || candidate == dock ||
                  panelTypeId(candidate) != QLatin1String("MapDock") ||
                  !candidate->isVisible()) {
                continue;
              }
              fallback = qobject_cast<map::MapPanel*>(candidate->widget());
              if (fallback != nullptr) {
                break;
              }
            }
            setActiveMapPanel(fallback);
          });
}

PanelDockWidget* FramePanels::createMapPanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("MapDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Map"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("MapDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelMap")));
  auto* panel = new map::MapPanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireMapPanel(dock, panel);
  updateMapDockTitle(dock, panel);
  frame_->layout_->configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void FramePanels::wireChannelGraphPanel(
    PanelDockWidget* dock, channel_graph::ChannelGraphPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &channel_graph::ChannelGraphPanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  QObject::connect(panel, &channel_graph::ChannelGraphPanel::panelExpandRequested, frame_, [this, dock]() { frame_->layout_->expandPanelDock(dock); });
  QObject::connect(panel, &channel_graph::ChannelGraphPanel::panelSplitRequested, frame_, [this, dock](Qt::Orientation orientation) {
            frame_->layout_->onSplitActiveDock(dock, orientation);
          });
  QObject::connect(panel, &channel_graph::ChannelGraphPanel::panelChangeRequested, frame_, [this, dock](const QString& object_name) {
            frame_->layout_->changePanelInDock(dock, object_name);
          });
  QObject::connect(panel, &channel_graph::ChannelGraphPanel::configChanged, frame_, [this]() { frame_->session_->markConfigModified(); });
  QObject::connect(
      panel, &channel_graph::ChannelGraphPanel::openInRawMessagesRequested, frame_,
      [this](const QString& channel) {
        if (channel_dock_ != nullptr) {
          channel_dock_->show();
          channel_dock_->raise();
        }
        if (raw_messages_panel_ != nullptr) {
          raw_messages_panel_->selectChannel(channel);
        }
      });
  QObject::connect(
      panel, &channel_graph::ChannelGraphPanel::addToPlotRequested, frame_,
      [this](const QString& channel, const QString& field_path) {
        if (plot_dock_ != nullptr) {
          plot_dock_->show();
          plot_dock_->raise();
        }
        plot::PlotPanel* plot =
            active_plot_panel_ != nullptr ? active_plot_panel_ : plot_panel_;
        if (plot != nullptr) {
          plot->addSeriesFromTopic(channel, field_path);
        }
      });
  QObject::connect(
      panel, &channel_graph::ChannelGraphPanel::openInTableRequested, frame_,
      [this](const QString& channel, const QString& field_path) {
        PanelDockWidget* dock = nullptr;
        for (PanelDockWidget* candidate : frame_->layout_->orderedDockWidgets()) {
          if (candidate != nullptr &&
              panelTypeId(candidate) == QLatin1String("TableDock")) {
            dock = candidate;
            break;
          }
        }
        if (dock == nullptr) {
          dock = createTablePanelDock();
          frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
        }
        dock->show();
        dock->raise();
        auto* table = qobject_cast<table_panel::TablePanel*>(dock->widget());
        if (table != nullptr) {
          table->setSource(channel, field_path);
          updateTableDockTitle(dock, table);
        }
      });
}

PanelDockWidget* FramePanels::createChannelGraphPanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("ChannelGraphDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Channel Graph"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("ChannelGraphDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelChannelGraph")));
  auto* panel = new channel_graph::ChannelGraphPanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireChannelGraphPanel(dock, panel);
  frame_->layout_->configureMainPanelDock(dock);
  registerPanelDock(dock);
  if (dock_name == QLatin1String("ChannelGraphDock")) {
    channel_graph_dock_ = dock;
  }
  return dock;
}

PanelDockWidget* FramePanels::duplicatePanelDock(
    PanelDockWidget* source) {
  if (source == nullptr) {
    return nullptr;
  }

  const QString type = panelTypeId(source);
  if (type == QLatin1String("ViewportDock")) {
    PanelDockWidget* dock = frame_->viewport_->createViewportPanelDock();
    if (ViewportPanelEntry* src_entry = frame_->viewport_->viewportEntryForDock(source)) {
      if (ViewportPanelEntry* dst_entry = frame_->viewport_->viewportEntryForDock(dock)) {
        if (rendering::ViewController* src_vc = src_entry->viewController()) {
          if (rendering::ViewController* dst_vc = dst_entry->viewController()) {
            dst_vc->setState(src_vc->state());
          }
        }
      }
    }
    frame_->viewport_->setActiveViewportDock(dock);
    return dock;
  }
  if (type == QLatin1String("PlotDock")) {
    PanelDockWidget* dock = createPlotPanelDock();
    auto* src_panel = qobject_cast<plot::PlotPanel*>(source->widget());
    auto* dst_panel = qobject_cast<plot::PlotPanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updatePlotDockTitle(dock, dst_panel);
      setActivePlotPanel(dst_panel);
    }
    return dock;
  }
  if (type == QLatin1String("TableDock")) {
    PanelDockWidget* dock = createTablePanelDock();
    auto* src_panel = qobject_cast<table_panel::TablePanel*>(source->widget());
    auto* dst_panel = qobject_cast<table_panel::TablePanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updateTableDockTitle(dock, dst_panel);
    }
    return dock;
  }
  if (type == QLatin1String("ImageDock")) {
    PanelDockWidget* dock = createImagePanelDock();
    auto* src_panel = qobject_cast<image::ImagePanel*>(source->widget());
    auto* dst_panel = qobject_cast<image::ImagePanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updateImageDockTitle(dock, dst_panel);
      setActiveImagePanel(dst_panel);
    }
    return dock;
  }
  if (type == QLatin1String("TeleopDock")) {
    PanelDockWidget* dock = createTeleopPanelDock();
    auto* src_panel = qobject_cast<teleop::TeleopPanel*>(source->widget());
    auto* dst_panel = qobject_cast<teleop::TeleopPanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updateTeleopDockTitle(dock, dst_panel);
      setActiveTeleopPanel(dst_panel);
    }
    return dock;
  }
  if (type == QLatin1String("TfTreeDock")) {
    return createTfTreePanelDock();
  }
  if (type == QLatin1String("PublishDock")) {
    PanelDockWidget* dock = createPublishPanelDock();
    auto* src_panel = qobject_cast<publish_panel::PublishPanel*>(source->widget());
    auto* dst_panel = qobject_cast<publish_panel::PublishPanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updatePublishDockTitle(dock, dst_panel);
      setActivePublishPanel(dst_panel);
    }
    return dock;
  }
  if (type == QLatin1String("MapDock")) {
    PanelDockWidget* dock = createMapPanelDock();
    auto* src_panel = qobject_cast<map::MapPanel*>(source->widget());
    auto* dst_panel = qobject_cast<map::MapPanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updateMapDockTitle(dock, dst_panel);
      setActiveMapPanel(dst_panel);
    }
    return dock;
  }
  if (type == QLatin1String("ServiceDock")) {
    PanelDockWidget* dock = createServicePanelDock();
    auto* src_panel = qobject_cast<service_panel::ServicePanel*>(source->widget());
    auto* dst_panel = qobject_cast<service_panel::ServicePanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updateServiceDockTitle(dock, dst_panel);
      setActiveServicePanel(dst_panel);
    }
    return dock;
  }
  if (type == QLatin1String("ChannelGraphDock")) {
    PanelDockWidget* dock = createChannelGraphPanelDock();
    auto* src_panel =
        qobject_cast<channel_graph::ChannelGraphPanel*>(source->widget());
    auto* dst_panel =
        qobject_cast<channel_graph::ChannelGraphPanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
    }
    return dock;
  }
  return nullptr;
}

void FramePanels::onAddPanel() {
  // Catalog types that are hidden or not yet created (e.g. Graph / Map).
  QStringList available;
  for (const PanelCatalogEntry& entry : PanelCatalog()) {
    if (!entry.isImplemented()) {
      continue;
    }
    const QString object_name = QString::fromLatin1(entry.object_name);
    PanelDockWidget* dock = frame_->findChild<PanelDockWidget*>(object_name);
    if (dock == nullptr && object_name == QLatin1String("TopicsDock")) {
      dock = channels_dock_;
    }
    if (dock == nullptr || !dock->isVisible()) {
      available.push_back(object_name);
    }
  }
  if (available.isEmpty()) {
    QMessageBox::information(frame_, frame_->tr("Add New Panel"),
                             frame_->tr("All panels are already visible."));
    return;
  }
  AddPanelDialog dialog(available, frame_);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  frame_->layout_->showPanelByObjectName(dialog.selectedPanelObjectName());
}

void FramePanels::registerDeletePanelAction(PanelDockWidget* dock) {
  if (dock == nullptr || frame_->chrome_->delete_panel_menu_ == nullptr ||
      delete_panel_actions_.contains(dock)) {
    return;
  }
  const QString title =
      detail::PanelsMenuDisplayTitle(panelTypeId(dock), dock->windowTitle());
  auto* action = frame_->chrome_->delete_panel_menu_->addAction(
      IconLoader::panelsMenuDockIcon(panelTypeId(dock)), title, frame_, &VisualizationFrame::onDeletePanel);
  action->setData(QVariant::fromValue(static_cast<QObject*>(dock)));
  action->setToolTip(frame_->tr("Close the %1 panel").arg(title));
  delete_panel_actions_.insert(dock, action);
  frame_->chrome_->delete_panel_menu_->setEnabled(true);
}

void FramePanels::unregisterDeletePanelAction(PanelDockWidget* dock) {
  if (dock == nullptr || frame_->chrome_->delete_panel_menu_ == nullptr) {
    return;
  }
  QAction* action = delete_panel_actions_.take(dock);
  if (action == nullptr) {
    return;
  }
  frame_->chrome_->delete_panel_menu_->removeAction(action);
  action->deleteLater();
  frame_->chrome_->delete_panel_menu_->setEnabled(!frame_->chrome_->delete_panel_menu_->actions().isEmpty());
}

void FramePanels::onDeletePanel() {
  auto* action = qobject_cast<QAction*>(frame_->sender());
  if (action == nullptr) {
    return;
  }
  auto* dock = qobject_cast<PanelDockWidget*>(action->data().value<QObject*>());
  if (dock != nullptr) {
    // Prefer frame_->close() so PanelDockWidget::closed runs (menu uncheck / Split
    // duplicate deleteLater + rebuildPanelsMenuToggles).
    dock->close();
  }
}

}  // namespace autoviz
