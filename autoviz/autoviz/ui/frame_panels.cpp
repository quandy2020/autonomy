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
    if (dock != nullptr && dock->isVisible() &&
        !dock->property("mosaicPlaceholder").toBool()) {
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
         panel_type_id == QLatin1String("OperatorToolsDock") ||
         panel_type_id == QLatin1String("PublishDock") ||
         panel_type_id == QLatin1String("MapDock") ||
         panel_type_id == QLatin1String("ChannelGraphDock") ||
         panel_type_id == QLatin1String("ServiceDock") ||
         panel_type_id == QLatin1String("TableDock") ||
         panel_type_id == QLatin1String("EvidenceDock") ||
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

void FramePanels::updatePlotDockTitle(PanelDockWidget* dock,
                                             plot::PlotPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Plot") : title);
}

void FramePanels::tickVisibleImagePanels() {
  const auto tick_dock = [](PanelDockWidget* dock) {
    if (dock == nullptr || !dock->isVisible()) {
      return;
    }
    if (auto* panel = qobject_cast<image::ImagePanel*>(dock->widget())) {
      panel->tick();
    }
  };
  tick_dock(image_dock_);
  for (auto it = image_display_docks_.cbegin(); it != image_display_docks_.cend();
       ++it) {
    tick_dock(it.value().data());
  }
}

void FramePanels::refreshAllPlotSettingsChannels() {
  // Use orderedDockWidgets (mosaic + direct children) — recursive findChildren
  // has SEGV'd in QMetaObject::cast from the refresh timer under live Argus.
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
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
  for (auto it = image_display_docks_.cbegin(); it != image_display_docks_.cend();
       ++it) {
    if (auto* panel =
            qobject_cast<image::ImagePanel*>(it.value().isNull()
                                                 ? nullptr
                                                 : it.value()->widget())) {
      panel->refreshSettingsChannels();
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

void FramePanels::updateTableDockTitle(PanelDockWidget* dock,
                                       table_panel::TablePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Table") : title);
}

void FramePanels::updateImageDockTitle(PanelDockWidget* dock,
                                              image::ImagePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Image") : title);
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

void FramePanels::updatePublishDockTitle(
    PanelDockWidget* dock, publish_panel::PublishPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Publish") : title);
}

void FramePanels::updateServiceDockTitle(PanelDockWidget* dock,
                                              service_panel::ServicePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Service Call") : title);
}

void FramePanels::updateMapDockTitle(PanelDockWidget* dock,
                                            map::MapPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? frame_->tr("Map") : title);
}

PanelDockWidget* FramePanels::duplicatePanelDock(
    PanelDockWidget* source) {
  if (source == nullptr) {
    return nullptr;
  }

  const QString type = panelTypeId(source);
  if (type == QLatin1String("ViewportDock")) {
    // Defer Ogre/GLX until after mosaic split reparent — creating a live native
    // window here and then moving the dock SIGSEGVs (same as session restore).
    // Caller (@ref FrameLayout::onSplitActiveDock) parks/reinstalls and calls
    // ensureViewportPanelReady, then copies the source camera state.
    return frame_->viewport_->createViewportPanelDock(
        QString(), /*create_render_window=*/false);
  }
  if (type == QLatin1String("PlotDock")) {
    // Split opens a blank Plot — do not clone the parent's series / samples.
    PanelDockWidget* dock = createPlotPanelDock();
    if (auto* dst_panel = qobject_cast<plot::PlotPanel*>(dock->widget())) {
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
  if (type == QLatin1String("EvidenceDock")) {
    return createEvidencePanelDock();
  }
  if (type == QLatin1String("OperatorToolsDock")) {
    return createOperatorToolsPanelDock();
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
