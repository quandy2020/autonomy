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

void FrameViewport::syncToolContext() {
  ViewportPanelEntry* active_entry = activeViewportEntry();
  if (frame_->manager_ != nullptr) {
    frame_->manager_->displayContext().ogre_scene_host =
        active_entry != nullptr && active_entry->ogre_viewport != nullptr
            ? active_entry->ogre_viewport->ogreSceneHost()
            : nullptr;
  }
  common::ToolContext context;
  context.view_controller = activeViewController();
  if (active_entry != nullptr && active_entry->scene_overlay != nullptr) {
    context.scene_overlay = active_entry->scene_overlay.get();
  } else if (frame_->manager_ != nullptr) {
    context.scene_overlay = &frame_->manager_->sceneOverlay();
  }
  if (active_entry != nullptr) {
    // Never call QObject::objectName on a possibly-disposed dock pointer.
    if (!active_entry->object_name.isEmpty()) {
      context.viewport_key = active_entry->object_name.toStdString();
    } else if (active_entry->dock != nullptr &&
               viewport_panels_.contains(active_entry->dock)) {
      active_entry->object_name = active_entry->dock->objectName();
      context.viewport_key = active_entry->object_name.toStdString();
    }
  }
  if (active_entry != nullptr && active_entry->widget != nullptr) {
    context.viewport_width = active_entry->widget->width();
    context.viewport_height = active_entry->widget->height();
  }
  if (frame_->manager_ != nullptr) {
    auto& display_context = frame_->manager_->displayContext();
    display_context.view_controller = context.view_controller;
    display_context.viewport_width = context.viewport_width;
    display_context.viewport_height = context.viewport_height;
    if (context.view_controller != nullptr && context.viewport_width > 0 &&
        context.viewport_height > 0) {
      const float aspect =
          static_cast<float>(context.viewport_width) /
          static_cast<float>(std::max(1, context.viewport_height));
      display_context.view_matrix = context.view_controller->viewMatrix();
      display_context.projection_matrix =
          context.view_controller->projectionMatrix(aspect);
      display_context.has_view_matrices = true;
    } else {
      display_context.has_view_matrices = false;
    }
  }
  context.gpu_picking_enabled =
      rendering::GpuCapabilities::instance().hasHardwareGpu();
  if (context.gpu_picking_enabled && active_entry != nullptr) {
    if (active_entry->ogre_viewport != nullptr) {
      rendering::OgreRenderWindow* viewport = active_entry->ogre_viewport;
      context.gpu_depth_pick = [viewport](int x, int y, QVector3D* world) {
        if (viewport == nullptr || world == nullptr) {
          return false;
        }
        return viewport->readDepthPick(
            x, y, viewport->viewController().viewMatrix(),
            viewport->viewController().projectionMatrix(
                static_cast<float>(viewport->width()) /
                static_cast<float>(std::max(1, viewport->height()))),
            world);
      };
      context.gpu_pick_id_read = [viewport](int x, int y) {
        if (viewport == nullptr) {
          return common::kInvalidPickHandle;
        }
        return viewport->readPickHandleAt(x, y);
      };
    }
  }
  context.request_redraw = [this]() { requestViewportUpdate(); };
  context.sync_ogre_host = [this]() {
    if (ViewportPanelEntry* entry = activeViewportEntry()) {
      if (frame_->manager_ != nullptr && entry->ogre_viewport != nullptr) {
        frame_->manager_->displayContext().ogre_scene_host =
            entry->ogre_viewport->ogreSceneHost();
      }
    }
  };
  context.set_status = [this](const QString& text) {
    frame_->chrome_->status_hint_ = text.trimmed();
    frame_->chrome_->updateStatusBar();
  };
  context.autolink_node = frame_->manager_->autolinkNode();
  context.fixed_frame = frame_->manager_->fixedFrame();
  // Keep the active entry overlay set above — do not reset to the shared
  // manager overlay (that would make tools/draw inherit the wrong pane).
  if (context.scene_overlay == nullptr) {
    context.scene_overlay = &frame_->manager_->sceneOverlay();
  }
  context.display_context = &frame_->manager_->displayContext();
  context.selection_manager = &frame_->manager_->selectionManager();
  context.pick_registry = &frame_->manager_->pickRegistry();
  context.handler_manager = &frame_->manager_->handlerManager();
  context.interactive_markers = &frame_->manager_->interactiveMarkerRegistry();
  context.selections_changed = [this](
                                    const std::vector<common::SelectionEntry>&
                                        entries) { frame_->session_->updateSelectionPanel(entries); };
  context.revert_to_default_tool = [this]() {
    frame_->chrome_->applyActiveTool(frame_->manager_->tools().defaultToolId());
  };
  frame_->manager_->tools().setContext(std::move(context));
  forEachViewportPanel([this](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->setToolManager(&frame_->manager_->tools());
    }
  });
}

void FrameViewport::updateViewportCursor() {
  forEachViewportPanel([this](ViewportPanelEntry& entry) {
    if (entry.host == nullptr) {
      return;
    }
    common::Tool* tool = frame_->manager_->tools().toolById(entry.local_tool_id);
    // Shape cursors only on the host widget. Custom pixmap cursors have SEGV'd
    // in QCursor copy during the first mouse move after RenderSystem ready
    // (heap already soft-corrupted); keep pixmap cursors off the QWidget path.
    QCursor cursor = IconLoader::defaultCursor();
    if (entry.local_tool_id == "Interact") {
      cursor = QCursor(Qt::PointingHandCursor);
    } else if (entry.local_tool_id == "Select" ||
               entry.local_tool_id == "Measure" ||
               entry.local_tool_id == "PublishPoint") {
      cursor = QCursor(Qt::CrossCursor);
    }
    if (tool != nullptr) {
      tool->setCursor(cursor);
    }
    entry.host->setCursor(cursor);
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->setToolCursor(cursor);
    }
  });
}

void FrameViewport::applyViewController(const QString& name) {
  rendering::ViewController* controller = activeViewController();
  if (controller == nullptr) {
    return;
  }
  controller->setTypeByName(name);
  if (name == QLatin1String("FPS")) {
    if (ViewportPanelEntry* entry = activeViewportEntry()) {
      if (entry->widget != nullptr) {
        entry->widget->setFocus();
      }
    }
  }
  // Avoid frame_->manager_->update() here: during startup callbacks frame_ runs on the
  // UI thread before the splash finishes and can stall on TF / message queues.
  if (frame_->panels_->views_panel_ != nullptr) {
    frame_->panels_->views_panel_->refreshFromController();
  }
  syncViewportTitleBarTools();
  forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.widget != nullptr) {
      entry.widget->update();
    }
  });
}

void FrameViewport::applyRenderBackend(const QString& /*name*/) {
  rendering::GpuCapabilities::instance().ensureProbed();
  if (frame_->manager_->renderBackendName() != "Ogre") {
    frame_->manager_->setRenderBackendName("Ogre");
  }
  if (!rendering::GpuCapabilities::instance().hasHardwareGpu() &&
      frame_->chrome_->status_label_ != nullptr) {
    frame_->chrome_->status_label_->setText(
        frame_->tr("No hardware GPU detected; Ogre may use a software path."));
  }
  createViewport(QStringLiteral("Ogre"));
  forEachViewportPanel([this](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->setToolManager(&frame_->manager_->tools());
    }
  });
  applyViewController(QString::fromStdString(frame_->manager_->viewControllerName()));
  syncToolContext();
  updateViewportCursor();
  // Widget repaint only — do not drain display message queues here (startup).
  forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.widget != nullptr) {
      entry.widget->update();
    }
  });
}

void FrameViewport::syncViewportTitleBarToolsForEntry(
    const ViewportPanelEntry& entry) {
  if (entry.expand_button != nullptr) {
    entry.expand_button->blockSignals(true);
    entry.expand_button->setChecked(frame_->layout_->expanded_main_panel_dock_ == entry.dock);
    entry.expand_button->blockSignals(false);
  }
  if (entry.settings_button != nullptr && frame_->panels_->views_dock_ != nullptr) {
    entry.settings_button->blockSignals(true);
    const Qt::DockWidgetArea views_area = frame_->dockWidgetArea(frame_->panels_->views_dock_);
    const bool views_in_sidebar =
        views_area == Qt::LeftDockWidgetArea ||
        views_area == Qt::RightDockWidgetArea;
    entry.settings_button->setChecked(views_in_sidebar && frame_->panels_->views_dock_->isVisible());
    entry.settings_button->blockSignals(false);
  }
  if (entry.float_button != nullptr && entry.dock != nullptr) {
    const bool floating =
        entry.dock->isFloating() ||
        entry.dock->property("intentionalFloat").toBool();
    entry.float_button->blockSignals(true);
    entry.float_button->setChecked(floating);
    entry.float_button->setToolTip(floating ? frame_->tr("Dock panel")
                                            : frame_->tr("Float panel"));
    entry.float_button->blockSignals(false);
    if (entry.content_only_button != nullptr) {
      entry.content_only_button->setEnabled(floating);
      if (!floating) {
        entry.content_only_button->blockSignals(true);
        entry.content_only_button->setChecked(false);
        entry.content_only_button->blockSignals(false);
      }
    }
  }
}

void FrameViewport::setViewportFloatButtonChecked(PanelDockWidget* dock,
                                                  bool checked) {
  ViewportPanelEntry* entry = viewportEntryForDock(dock);
  if (entry == nullptr || entry->float_button == nullptr) {
    return;
  }
  entry->float_button->blockSignals(true);
  entry->float_button->setChecked(checked);
  entry->float_button->setToolTip(checked ? frame_->tr("Dock panel")
                                          : frame_->tr("Float panel"));
  entry->float_button->blockSignals(false);
  if (entry->content_only_button != nullptr) {
    entry->content_only_button->setEnabled(checked);
    if (!checked) {
      entry->content_only_button->blockSignals(true);
      entry->content_only_button->setChecked(false);
      entry->content_only_button->blockSignals(false);
    }
  }
}

void FrameViewport::setViewportContentOnlyButtonChecked(PanelDockWidget* dock,
                                                        bool enabled) {
  ViewportPanelEntry* entry = viewportEntryForDock(dock);
  if (entry == nullptr || entry->content_only_button == nullptr) {
    return;
  }
  entry->content_only_button->blockSignals(true);
  entry->content_only_button->setChecked(enabled);
  entry->content_only_button->setToolTip(
      enabled ? frame_->tr("Show title bar")
              : frame_->tr("Content-only (hide chrome)"));
  entry->content_only_button->blockSignals(false);
}

void FrameViewport::pushViewportLocalTool(ViewportPanelEntry& entry) {
  const std::string viewport_key =
      entry.dock != nullptr ? entry.dock->objectName().toStdString()
                            : std::string();
  if (entry.ogre_viewport != nullptr) {
    entry.ogre_viewport->setViewportToolId(entry.local_tool_id);
  }
  if (common::Tool* tool = frame_->manager_->tools().toolById(entry.local_tool_id)) {
    tool->setCursor(IconLoader::toolCursor(
        QString::fromStdString(entry.local_tool_id)));
    const QCursor cursor = tool->cursor();
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->setToolCursor(cursor);
    }
  }
}

void FrameViewport::setViewportLocalTool(PanelDockWidget* dock,
                                              const std::string& tool_id) {
  ViewportPanelEntry* entry = viewportEntryForDock(dock);
  if (entry == nullptr || frame_->manager_->tools().toolById(tool_id) == nullptr) {
    return;
  }
  const std::string previous_tool = entry->local_tool_id;
  const std::string viewport_key =
      dock != nullptr ? dock->objectName().toStdString() : std::string();
  entry->local_tool_id = tool_id;
  pushViewportLocalTool(*entry);
  if (previous_tool == "Measure" && tool_id != "Measure") {
    // clearViewportSession must hit frame_ dock's host, not the active one.
    if (entry->ogre_viewport != nullptr) {
      frame_->manager_->displayContext().ogre_scene_host =
          entry->ogre_viewport->ogreSceneHost();
    }
    frame_->manager_->tools().clearToolViewportSession("Measure", viewport_key);
  }
  if (dock == active_viewport_dock_) {
    frame_->manager_->tools().setActiveTool(tool_id);
    frame_->chrome_->syncToolbarToActiveTool();
    syncToolContext();
    if (frame_->panels_->tool_properties_panel_ != nullptr) {
      frame_->panels_->tool_properties_panel_->refresh();
    }
    frame_->chrome_->updateStatusBar();
  }
  requestViewportUpdate();
}





void FrameViewport::installViewportHudOverlay(ViewportPanelEntry& entry) {
  // Glass SPEED/ANG/ODOM dials (WA_AlwaysStackOnTop + translucent) float as a
  // see-through strip over the desktop during mosaic drag. Flat theme keeps
  // the 3D plate clean — tear down any leftover overlay and do not reinstall.
  if (entry.hud_overlay != nullptr) {
    if (entry.layout != nullptr) {
      entry.layout->removeWidget(entry.hud_overlay);
    }
    entry.hud_overlay->setAttribute(Qt::WA_AlwaysStackOnTop, false);
    entry.hud_overlay->setHudVisible(false);
    entry.hud_overlay->hide();
    entry.hud_overlay->deleteLater();
    entry.hud_overlay = nullptr;
  }
  if (entry.host != nullptr) {
    const QList<ViewportHudOverlay*> orphans =
        entry.host->findChildren<ViewportHudOverlay*>();
    for (ViewportHudOverlay* hud : orphans) {
      if (hud == nullptr) {
        continue;
      }
      hud->setAttribute(Qt::WA_AlwaysStackOnTop, false);
      hud->setHudVisible(false);
      hud->hide();
      hud->deleteLater();
    }
  }
  if (entry.layout != nullptr) {
    entry.layout->setContentsMargins(0, 0, 0, 0);
  }
}

void FrameViewport::syncViewportTitleBarTools() {
  forEachViewportPanel([this](ViewportPanelEntry& entry) {
    syncViewportTitleBarToolsForEntry(entry);
  });
}


}  // namespace autoviz
