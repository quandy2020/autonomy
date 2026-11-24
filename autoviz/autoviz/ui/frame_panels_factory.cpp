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

void FramePanels::ensurePlotDockExists(const QString& object_name) {
  if (object_name.isEmpty() ||
      frame_->findChild<PanelDockWidget*>(object_name) != nullptr) {
    return;
  }
  PanelDockWidget* dock = createPlotPanelDock(object_name);
  frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
}

void FramePanels::ensureImageDockExists(const QString& object_name) {
  if (object_name.isEmpty() ||
      frame_->findChild<PanelDockWidget*>(object_name) != nullptr) {
    return;
  }
  PanelDockWidget* dock = createImagePanelDock(object_name);
  frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
}

void FramePanels::ensurePublishDockExists(const QString& object_name) {
  if (object_name.isEmpty() ||
      frame_->findChild<PanelDockWidget*>(object_name) != nullptr) {
    return;
  }
  PanelDockWidget* dock = createPublishPanelDock(object_name);
  frame_->layout_->addMainPanelDock(dock, Qt::LeftDockWidgetArea);
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

PanelDockWidget* FramePanels::createNavigationPanelDock(
    const QString& object_name) {
  if (navigation_dock_ != nullptr &&
      (object_name.isEmpty() ||
       object_name == QLatin1String("NavigationDock"))) {
    return navigation_dock_;
  }
  const QString dock_name = object_name.isEmpty()
                                ? QStringLiteral("NavigationDock")
                                : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Navigation"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("NavigationDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelNavigation")));
  auto* panel = new navigation::NavigationPanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  frame_->layout_->configureSidebarDock(dock, Qt::RightDockWidgetArea);
  registerPanelDock(dock);
  if (dock_name == QLatin1String("NavigationDock")) {
    navigation_dock_ = dock;
  }
  return dock;
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

PanelDockWidget* FramePanels::createEvidencePanelDock(const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty()
          ? uniquePanelObjectName(QStringLiteral("EvidenceDock"))
          : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Evidence"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("EvidenceDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelEvidence")));
  auto* panel = new EvidencePanel(frame_->manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireEvidencePanel(dock, panel);
  frame_->layout_->configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
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

PanelDockWidget* FramePanels::createOperatorToolsPanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("OperatorToolsDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(frame_->tr("Operator Tools"), frame_);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("OperatorToolsDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelOperatorTools")));
  auto* panel = new OperatorToolsPanel(frame_->manager_.get(), dock);
  dock->setContentWidget(panel);
  frame_->layout_->configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
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

}  // namespace autoviz
