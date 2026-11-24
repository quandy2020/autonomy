/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_panels.hpp"
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/ui/frame_detail.hpp"
#include "autoviz/common/selection.hpp"
#include "autoviz/plugins/displays/display_group.hpp"
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
#include "autoviz/ui/image/image_view_widget.hpp"
#include "autoviz/ui/panel_host.hpp"
#include "autoviz/ui/panel/role.hpp"
#include "autoviz/ui/inspector/property_panel.hpp"
#include "autoviz/ui/dialog/import_record.hpp"
#include "autoviz/ui/dialog/record_open.hpp"
#include "autoviz/ui/plot/io/plot_config_io.hpp"
#include "autoviz/ui/publish/publish_config_io.hpp"
#include "autoviz/ui/service/service_config_io.hpp"
#include "autoviz/ui/teleop/teleop_config_io.hpp"
#include "autoviz/ui/map/map_config_io.hpp"
#include "autoviz/ui/image/image_config_io.hpp"
#include "autoviz/ui/plot/panel/plot_panel.hpp"
#include "autoviz/ui/table/table_config_io.hpp"
#include "autoviz/ui/table/table_panel.hpp"
#include "autoviz/ui/teleop/teleop_panel.hpp"
#include "autoviz/ui/navigation/navigation_panel.hpp"
#include "autoviz/ui/raw/panel.hpp"
#include "autoviz/ui/channels/channels_panel.hpp"
#include "autoviz/ui/record/record_panel.hpp"
#include "autoviz/ui/evidence/evidence_panel.hpp"
#include "autoviz/ui/inspector/selection_panel.hpp"
#include "autoviz/ui/inspector/tool_properties_panel.hpp"
#include "autoviz/ui/tf_tree/panel.hpp"
#include "autoviz/ui/panel/operator_tools_panel.hpp"
#include "autoviz/ui/publish/publish_panel.hpp"
#include "autoviz/ui/map/map_panel.hpp"
#include "autoviz/ui/channel_graph/channel_graph_panel.hpp"
#include "autoviz/ui/channel_graph/channel_graph_config_io.hpp"
#include "autoviz/ui/service/service_panel.hpp"
#include "autoviz/ui/time/panel.hpp"
#include "autoviz/ui/panel/name_map.hpp"
#include "autoviz/ui/viewport_hud.hpp"
#include "autoviz/ui/views/panel.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"


namespace autoviz {

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

}  // namespace autoviz
