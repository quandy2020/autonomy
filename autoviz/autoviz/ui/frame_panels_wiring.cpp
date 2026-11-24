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
  QObject::connect(panel, &image::ImagePanel::panelFloatRequested, frame_,
                   [this, dock]() { frame_->layout_->toggleFloatMainPanel(dock); });
  QObject::connect(
      panel, &image::ImagePanel::panelContentOnlyRequested, frame_,
      [this, dock, panel]() {
        if (dock == nullptr || panel == nullptr) {
          return;
        }
        if (!dock->property("intentionalFloat").toBool()) {
          return;
        }
        const QString title = panel->config().title.trimmed().isEmpty()
                                  ? panel->config().image_channel
                                  : panel->config().title.trimmed();
        if (IsContentOnlyFloat(dock)) {
          frame_->layout_->setContentOnlyFloat(dock, false);
        } else {
          frame_->layout_->setContentOnlyFloat(dock, true, title);
        }
      });
  QObject::connect(
      panel, &image::ImagePanel::panelContentOnlySetRequested, frame_,
      [this, dock, panel](bool enabled) {
        if (dock == nullptr || panel == nullptr) {
          return;
        }
        if (!dock->property("intentionalFloat").toBool()) {
          return;
        }
        const QString title = panel->config().title.trimmed().isEmpty()
                                  ? panel->config().image_channel
                                  : panel->config().title.trimmed();
        frame_->layout_->setContentOnlyFloat(dock, enabled, title);
      });
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

void FramePanels::wireEvidencePanel(PanelDockWidget* dock, EvidencePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  QObject::connect(panel, &EvidencePanel::panelRemoveRequested, dock,
                   &QDockWidget::close);
  QObject::connect(panel, &EvidencePanel::panelExpandRequested, frame_,
                   [this, dock]() { frame_->layout_->expandPanelDock(dock); });
  QObject::connect(
      panel, &EvidencePanel::panelSplitRequested, frame_,
      [this, dock](Qt::Orientation orientation) {
        frame_->layout_->onSplitActiveDock(dock, orientation);
      });
  QObject::connect(panel, &EvidencePanel::panelChangeRequested, frame_,
                   [this, dock](const QString& object_name) {
                     frame_->layout_->changePanelInDock(dock, object_name);
                   });
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

}  // namespace autoviz
