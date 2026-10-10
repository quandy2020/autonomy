/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_session.hpp"
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/common/selection.hpp"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
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
#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QIODevice>
#include <QTextStream>
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
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QQuaternion>
#include <QVBoxLayout>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/tool_manager.hpp"
#include "autoviz/integration/message_queue.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"
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
#include "autoviz/ui/record/record_panel.hpp"
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
#include "autoviz/ui/record_drop_overlay.hpp"

namespace autoviz {

FrameSession::FrameSession(VisualizationFrame* frame) : frame_(frame) {}

void FrameSession::updateWindowTitle() {
  QString title = QStringLiteral("Autoviz");
  if (!config_path_.isEmpty()) {
    title += QStringLiteral(" - %1").arg(QFileInfo(config_path_).fileName());
  }
  if (config_modified_) {
    title += QLatin1Char('*');
  }
  frame_->setWindowTitle(title);
}

void FrameSession::markConfigModified() {
  if (suppress_config_modified_ || config_modified_) {
    return;
  }
  config_modified_ = true;
  updateWindowTitle();
}

void FrameSession::clearConfigModified() {
  config_modified_ = false;
  updateWindowTitle();
}

void FrameSession::connectConfigModifiedSignals() {
  QObject::connect(frame_->panels_->displays_panel_, &DisplaysPanel::displaysChanged, frame_, &VisualizationFrame::markConfigModified);
  QObject::connect(frame_->panels_->displays_panel_, &DisplaysPanel::fixedFrameChanged, frame_, &VisualizationFrame::markConfigModified);
  QObject::connect(frame_->panels_->displays_panel_, &DisplaysPanel::backgroundColorChanged, frame_, &VisualizationFrame::markConfigModified);

  if (frame_->panels_->views_panel_ != nullptr) {
    QObject::connect(frame_->panels_->views_panel_, &ViewsPanel::viewsChanged, frame_, &VisualizationFrame::markConfigModified);
    QObject::connect(frame_->panels_->views_panel_, &ViewsPanel::viewChanged, frame_, &VisualizationFrame::markConfigModified);
  }
  if (frame_->panels_->tool_properties_panel_ != nullptr) {
    QObject::connect(frame_->panels_->tool_properties_panel_, &ToolPropertiesPanel::propertiesChanged, frame_, &VisualizationFrame::markConfigModified);
  }

  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    QObject::connect(dock, &QDockWidget::dockLocationChanged, frame_, &VisualizationFrame::markConfigModified);
    QObject::connect(dock, &QDockWidget::topLevelChanged, frame_, &VisualizationFrame::markConfigModified);
    QObject::connect(dock, &PanelDockWidget::closed, frame_, &VisualizationFrame::markConfigModified);
  }
}

void FrameSession::updateSelectionPanel(
    const std::vector<common::SelectionEntry>& entries) {
  if (frame_->panels_->selection_panel_ != nullptr) {
    frame_->panels_->selection_panel_->setSelections(entries);
  }
}

void FrameSession::onScreenshot() {
  ViewportPanelEntry* entry = frame_->viewport_->activeViewportEntry();
  if (entry == nullptr || entry->widget == nullptr) {
    return;
  }
  const QPixmap pixmap = entry->widget->grab();
  const QString path = QFileDialog::getSaveFileName(
      frame_, frame_->tr("Save Image"),
      QStringLiteral("autoviz_%1.png")
          .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_hhmmss"))),
      frame_->tr("PNG Image (*.png)"));
  if (!path.isEmpty()) {
    pixmap.save(path);
  }
}

void FrameSession::setFullScreen(bool full_screen) {
  const auto state = frame_->windowState();
  if (full_screen == state.testFlag(Qt::WindowFullScreen)) {
    return;
  }
  if (frame_->chrome_->fullscreen_action_ != nullptr) {
    frame_->chrome_->fullscreen_action_->setChecked(full_screen);
  }
  if (full_screen) {
    frame_->chrome_->toolbar_visible_ = frame_->chrome_->tool_bar_ == nullptr || frame_->chrome_->tool_bar_->isVisible();
  }
  emit frame_->fullScreenChange(full_screen);
  frame_->menuBar()->setVisible(!full_screen);
  if (frame_->chrome_->tool_bar_ != nullptr) {
    frame_->chrome_->tool_bar_->setVisible(!full_screen && frame_->chrome_->toolbar_visible_);
  }
  frame_->statusBar()->setVisible(!full_screen);
  if (full_screen) {
    frame_->setWindowState(state | Qt::WindowFullScreen);
  } else {
    frame_->setWindowState(state & ~Qt::WindowFullScreen);
  }
  frame_->show();
}

void FrameSession::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape && frame_->isFullScreen()) {
    setFullScreen(false);
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_R && !event->modifiers().testFlag(Qt::ControlModifier) &&
      !event->modifiers().testFlag(Qt::AltModifier)) {
    frame_->onReset();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Z && frame_->panels_->views_panel_ != nullptr &&
      !event->modifiers().testFlag(Qt::ControlModifier) &&
      !event->modifiers().testFlag(Qt::AltModifier)) {
    frame_->panels_->views_panel_->zeroView();
    frame_->viewport_->requestViewportUpdate();
    event->accept();
    return;
  }
  if (!event->modifiers()) {
    if (frame_->manager_->tools().handleShortcutKey(event->key())) {
      frame_->chrome_->syncActiveToolUi();
      event->accept();
      return;
    }
  }
}

bool FrameSession::loadConfig(const QString& path) {
  suppress_config_modified_ = true;
  if (!frame_->manager_->loadSession(path.toStdString())) {
    suppress_config_modified_ = false;
    return false;
  }
  config_path_ = path;
  frame_->panels_->displays_panel_->refresh();
  frame_->panels_->syncImageDisplayWindows();
  syncViewsFromManager();
  // Do not call applyRenderBackend() here: Frame ctor already created the
  // Ogre window, and recreating it before mosaic restore (destroy + reparent)
  // SIGSEGVs. GL is torn down in restoreWindowLayout and recreated in
  // activateRestoredViewportAndView() after docks are in final panes.
  if (frame_->manager_->renderBackendName() != "Ogre") {
    frame_->manager_->setRenderBackendName("Ogre");
  }
  frame_->chrome_->rebuildToolbar();
  // Create / reconfigure panel docks BEFORE restoreWindowLayout so
  // QMainWindow::restoreState and the center mosaic can find objectNames.
  frame_->panels_->restorePlotPanelConfigs();
  frame_->panels_->restoreTablePanelConfigs();
  frame_->panels_->restoreChannelGraphPanelConfigs();
  frame_->panels_->restoreTfTreePanelConfigs();
  frame_->panels_->restoreImagePanelConfigs();
  frame_->panels_->restorePublishPanelConfigs();
  frame_->panels_->restoreServicePanelConfigs();
  frame_->panels_->restoreTeleopPanelConfigs();
  frame_->panels_->restoreMapPanelConfigs();
  // Recreate split 3D Views (ViewportDock_2, …) before mosaic restore.
  ensureSessionMainPanels();
  restoreWindowLayout();
  activateRestoredViewportAndView();
  if (frame_->panels_->channels_panel_ != nullptr) {
    frame_->panels_->channels_panel_->setConfig(
        frame_->manager_->channelsBrowser());
  }
  if (frame_->panels_->raw_messages_panel_ != nullptr) {
    frame_->panels_->raw_messages_panel_->setConfig(
        frame_->manager_->rawMessages());
  }
  if (frame_->panels_->time_panel_ != nullptr) {
    const int sync_mode = static_cast<int>(frame_->manager_->timeSyncMode());
    const QString sync_source =
        QString::fromStdString(frame_->manager_->timeSyncSource());
    if (sync_mode != 0 || !sync_source.isEmpty()) {
      frame_->panels_->time_panel_->setExperimental(true);
    }
    frame_->panels_->time_panel_->setSyncMode(sync_mode);
    frame_->panels_->time_panel_->setSyncSource(sync_source);
  }
  frame_->panels_->applyPlotSettingsVisibilityFromSession();
  frame_->chrome_->applyActiveTool(frame_->manager_->tools().activeToolId());
  // Config window-state may have hidden ImageDock; keep it available when a
  // default image topic is configured (tutorial path).
  if (frame_->panels_->image_dock_ != nullptr && frame_->panels_->image_panel_ != nullptr &&
      !frame_->panels_->image_panel_->config().image_channel.isEmpty()) {
    frame_->panels_->image_dock_->show();
  }
  frame_->layout_->ensureTimeDockAtBottom();
  frame_->panels_->syncDeletePanelMenu();
  frame_->chrome_->updateStatusBar();
  markRecentConfig(path);
  suppress_config_modified_ = false;
  clearConfigModified();
  return true;
}

bool FrameSession::saveConfig(const QString& path) {
  frame_->syncViewsToManager();
  captureCurrentView();
  captureWindowLayout();
  if (!frame_->manager_->saveSession(path.toStdString())) {
    return false;
  }
  config_path_ = path;
  return true;
}

void FrameSession::syncViewsFromManager() {
  if (frame_->panels_->views_panel_ != nullptr) {
    frame_->panels_->views_panel_->setSavedViews(frame_->manager_->savedViews());
  }
}

void FrameSession::syncViewsToManager() {
  if (frame_->panels_->views_panel_ != nullptr) {
    frame_->manager_->setSavedViews(frame_->panels_->views_panel_->savedViews());
  }
}

void FrameSession::captureCurrentView() {
  if (rendering::ViewController* controller = frame_->viewport_->activeViewController()) {
    frame_->manager_->setCurrentView(
        common::ToSavedViewConfig("Current", controller->state()));
  }
}

void FrameSession::applyCurrentView() {
  if (!frame_->manager_->hasCurrentView()) {
    frame_->viewport_->applyViewController(QString::fromStdString(frame_->manager_->viewControllerName()));
    return;
  }
  const common::SavedViewConfig& view = frame_->manager_->currentView();
  if (!view.type.empty()) {
    frame_->viewport_->applyViewController(QString::fromStdString(view.type));
  }
  if (rendering::ViewController* controller = frame_->viewport_->activeViewController()) {
    controller->setState(common::ToViewState(view));
    if (frame_->panels_->views_panel_ != nullptr) {
      frame_->panels_->views_panel_->refreshFromController();
    }
    frame_->viewport_->requestViewportUpdate();
  }
}

namespace {

void CollectMosaicLeafNames(const QJsonObject& node, QSet<QString>* names) {
  if (names == nullptr || node.isEmpty()) {
    return;
  }
  const QString type = node.value(QStringLiteral("type")).toString();
  if (type == QLatin1String("leaf")) {
    const QString name = node.value(QStringLiteral("name")).toString();
    if (!name.isEmpty()) {
      names->insert(name);
    }
    return;
  }
  if (type != QLatin1String("split")) {
    return;
  }
  const QJsonArray children = node.value(QStringLiteral("children")).toArray();
  for (const QJsonValue& child : children) {
    if (child.isObject()) {
      CollectMosaicLeafNames(child.toObject(), names);
    }
  }
}

bool IsViewportDockObjectName(const QString& object_name) {
  return object_name == QLatin1String("ViewportDock") ||
         object_name.startsWith(QLatin1String("ViewportDock_"));
}

}  // namespace

void FrameSession::ensureSessionMainPanels() {
  QSet<QString> needed;
  for (const std::string& panel : frame_->manager_->visiblePanels()) {
    const QString name =
        QString::fromStdString(NormalizePanelObjectName(panel));
    if (!name.isEmpty()) {
      needed.insert(name);
    }
  }
  for (const auto& entry : frame_->manager_->panelLayouts()) {
    const QString name =
        QString::fromStdString(NormalizePanelObjectName(entry.object_name));
    if (!name.isEmpty()) {
      needed.insert(name);
    }
  }
  const QByteArray mosaic = QByteArray::fromBase64(
      QByteArray::fromStdString(frame_->manager_->mainPanelStateBase64()));
  static constexpr char kMosaicPrefix[] = "mosaic:v1:";
  if (mosaic.startsWith(kMosaicPrefix)) {
    const QByteArray json =
        mosaic.mid(static_cast<int>(sizeof(kMosaicPrefix) - 1));
    const QJsonDocument document = QJsonDocument::fromJson(json);
    if (document.isObject()) {
      CollectMosaicLeafNames(
          document.object().value(QStringLiteral("root")).toObject(), &needed);
    }
  }

  for (const QString& name : needed) {
    if (name.isEmpty() ||
        frame_->findChild<PanelDockWidget*>(name) != nullptr) {
      continue;
    }
    if (IsViewportDockObjectName(name)) {
      // Defer Ogre/native window until after mosaic restore + show.
      PanelDockWidget* dock = frame_->viewport_->createViewportPanelDock(
          name, /*create_render_window=*/false);
      if (dock != nullptr) {
        dock->hide();
      }
    }
  }
}

void FrameSession::activateRestoredViewportAndView() {
  PanelDockWidget* visible_viewport = nullptr;
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || !dock->requestedVisible() ||
        frame_->panels_->panelTypeId(dock) != QLatin1String("ViewportDock")) {
      continue;
    }
    visible_viewport = dock;
    break;
  }

  const bool host_has_viewport = [&]() {
    if (frame_->layout_->main_panel_host_ == nullptr) {
      return false;
    }
    for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
      if (dock == nullptr ||
          frame_->panels_->panelTypeId(dock) !=
              QLatin1String("ViewportDock")) {
        continue;
      }
      if (frame_->layout_->main_panel_host_->hostsPanel(dock)) {
        return true;
      }
    }
    return false;
  }();

  // Center column must host a 3D View by default. Cover: closed-all, mosaic
  // that parked ViewportDock, or restore that left only Image/Plot in the host.
  if ((visible_viewport == nullptr || !host_has_viewport) &&
      frame_->viewport_->viewport_dock_ != nullptr) {
    PanelDockWidget* primary = frame_->viewport_->viewport_dock_;
    frame_->layout_->center_manual_layout_ = false;
    frame_->layout_->suppress_center_tile_ = false;
    frame_->layout_->ensureMainPanelDockAttached(primary);
    primary->show();
    if (QAction* toggle = primary->toggleViewAction()) {
      toggle->blockSignals(true);
      toggle->setChecked(true);
      toggle->blockSignals(false);
    }
    frame_->layout_->last_active_dock_ = primary;
    if (!host_has_viewport) {
      // Empty / Image-only center → retile so ViewportDock is in the mosaic.
      frame_->layout_->tileCenterPanels();
    } else {
      frame_->layout_->scheduleTileCenterPanels();
    }
    visible_viewport = primary;
  }
  if (visible_viewport != nullptr) {
    frame_->viewport_->ensureViewportPanelReady(visible_viewport);
    frame_->viewport_->setActiveViewportDock(visible_viewport);
  }
  applyCurrentView();
}

void FrameSession::captureWindowLayout() {
  frame_->chrome_->rebuildToolbar();
  const QByteArray window_state = frame_->saveState().toBase64();
  const QByteArray window_geometry = frame_->saveGeometry().toBase64();
  frame_->manager_->setWindowLayout(window_state.toStdString(),
                                    window_geometry.toStdString());
  if (frame_->layout_->main_panel_host_ != nullptr) {
    // Persist the QSplitter mosaic (not QMainWindow::saveState — center panels
    // are not QDockWidget children of the host).
    const QByteArray mosaic =
        frame_->layout_->main_panel_host_->saveMosaicLayout().toBase64();
    frame_->manager_->setMainPanelLayout(mosaic.toStdString());
  }
  const bool hide_left =
      frame_->chrome_->toolbar_toggle_left_dock_action_ != nullptr
          ? !frame_->chrome_->toolbar_toggle_left_dock_action_->isChecked()
          : frame_->manager_->hideLeftDock();
  const bool hide_right =
      frame_->chrome_->toolbar_toggle_right_dock_action_ != nullptr
          ? !frame_->chrome_->toolbar_toggle_right_dock_action_->isChecked()
          : frame_->manager_->hideRightDock();
  frame_->manager_->setDockHideState(hide_left, hide_right);
  std::vector<common::PanelLayoutConfig> layouts;
  layouts.reserve(frame_->layout_->orderedDockWidgets().size());
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    common::PanelLayoutConfig entry;
    entry.object_name = dock->objectName().toStdString();
    entry.collapsed = dock->isCollapsed();
    layouts.push_back(std::move(entry));
  }
  frame_->manager_->setPanelLayouts(layouts);
  std::vector<std::string> visible_panels;
  visible_panels.reserve(frame_->layout_->orderedDockWidgets().size());
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    if (dock->isVisible()) {
      visible_panels.push_back(dock->objectName().toStdString());
    }
  }
  bool plot_settings_visible = false;
  if (frame_->panels_->active_plot_panel_ != nullptr) {
    plot_settings_visible = frame_->panels_->active_plot_panel_->settingsVisible();
  } else if (frame_->panels_->plot_panel_ != nullptr) {
    plot_settings_visible = frame_->panels_->plot_panel_->settingsVisible();
  }
  frame_->manager_->setPlotSettingsVisible(plot_settings_visible);
  frame_->manager_->setVisiblePanels(visible_panels);
  frame_->panels_->capturePlotPanelConfigs();
  frame_->panels_->captureTablePanelConfigs();
  frame_->panels_->captureChannelGraphPanelConfigs();
  frame_->panels_->captureTfTreePanelConfigs();
  frame_->panels_->captureImagePanelConfigs();
  frame_->panels_->capturePublishPanelConfigs();
  frame_->panels_->captureServicePanelConfigs();
  frame_->panels_->captureTeleopPanelConfigs();
  frame_->panels_->captureMapPanelConfigs();
  if (frame_->panels_->channels_panel_ != nullptr) {
    frame_->manager_->setChannelsBrowser(
        frame_->panels_->channels_panel_->config());
  }
  if (frame_->panels_->raw_messages_panel_ != nullptr) {
    frame_->manager_->setRawMessages(
        frame_->panels_->raw_messages_panel_->config());
  }
  const QRect frame_geometry = frame_->geometry();
  frame_->manager_->setWindowFrame(frame_geometry.x(), frame_geometry.y(),
                           frame_geometry.width(), frame_geometry.height());
}

void FrameSession::applyPanelVisibility() {
  const std::vector<std::string>& visible = frame_->manager_->visiblePanels();
  if (visible.empty()) {
    return;
  }
  const auto is_visible = [&visible](const std::string& object_name) {
    for (const std::string& panel : visible) {
      if (NormalizePanelObjectName(panel) == object_name || panel == object_name) {
        return true;
      }
    }
    return false;
  };
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    if (is_visible(dock->objectName().toStdString())) {
      if (frame_->layout_->isMainPanel(dock)) {
        frame_->layout_->ensureMainPanelDockAttached(dock);
      } else {
        frame_->layout_->ensureSidebarDockAttached(dock);
      }
      dock->show();
    } else {
      dock->hide();
    }
  }
  frame_->layout_->ensureTimeDockAtBottom();
  frame_->layout_->scheduleTileCenterPanels();
  frame_->panels_->syncDeletePanelMenu();
}

void FrameSession::applyStartupWindowState() {
  const AppUiPreferences prefs = LoadAppUiPreferences();
  if (!prefs.start_maximized) {
    if (!frame_->isVisible()) {
      frame_->show();
    }
  } else {
    frame_->setWindowState(Qt::WindowMaximized);
    frame_->show();

    const auto ensure_maximized = [this]() {
      if (!LoadAppUiPreferences().start_maximized) {
        return;
      }
      if (frame_->isMaximized()) {
        frame_->raise();
        frame_->activateWindow();
        return;
      }
      frame_->showMaximized();
      frame_->raise();
      frame_->activateWindow();
      if (frame_->isMaximized()) {
        return;
      }
      if (QScreen* screen = QGuiApplication::primaryScreen()) {
        frame_->setGeometry(screen->availableGeometry());
      }
    };

    QTimer::singleShot(0, frame_, ensure_maximized);
    QTimer::singleShot(150, frame_, ensure_maximized);
  }

  // Fresh start (no loadConfig) never hits activateRestoredViewportAndView.
  // After the window is shown, ensure the center mosaic hosts 3D View.
  QTimer::singleShot(0, frame_, [this]() {
    if (frame_->viewport_->viewport_dock_ == nullptr ||
        frame_->layout_->main_panel_host_ == nullptr) {
      return;
    }
    const bool host_has_viewport =
        frame_->layout_->main_panel_host_->hostsPanel(
            frame_->viewport_->viewport_dock_);
    if (host_has_viewport &&
        frame_->viewport_->viewport_dock_->requestedVisible()) {
      frame_->viewport_->ensureViewportPanelReady(
          frame_->viewport_->viewport_dock_);
    } else {
      frame_->layout_->applyMainPanelDefaultLayout();
    }

    // Optional CI/smoke: Split Right the active 3D View after layout settles.
    const int split_ms = qEnvironmentVariableIntValue("AUTOVIZ_SMOKE_SPLIT_MS");
    if (split_ms > 0 && frame_->viewport_->viewport_dock_ != nullptr) {
      QTimer::singleShot(split_ms, frame_, [this]() {
        if (frame_->viewport_->viewport_dock_ == nullptr) {
          return;
        }
        frame_->layout_->onSplitActiveDock(frame_->viewport_->viewport_dock_,
                                           Qt::Horizontal);
        // Dump mosaic state after deferred Split finish settles.
        QTimer::singleShot(250, frame_, [this]() {
          QFile diag(QStringLiteral("/tmp/autoviz_split_diag.txt"));
          if (!diag.open(QIODevice::WriteOnly | QIODevice::Truncate |
                         QIODevice::Text)) {
            return;
          }
          QTextStream out(&diag);
          int hosted_count = 0;
          for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
            if (dock == nullptr || frame_->layout_->main_panel_host_ == nullptr ||
                !frame_->layout_->main_panel_host_->hostsPanel(dock)) {
              continue;
            }
            ++hosted_count;
            out << "dock name=" << dock->objectName()
                << " title=" << dock->windowTitle()
                << " visible=" << (dock->isVisible() ? 1 : 0)
                << " geo=" << dock->width() << 'x' << dock->height()
                << " at=" << dock->x() << ',' << dock->y() << '\n';
          }
          out << "hosted=" << hosted_count << '\n';
          int vp_count = 0;
          frame_->viewport_->forEachViewportPanel(
              [&](ViewportPanelEntry& entry) {
                ++vp_count;
                out << "viewport name=" << entry.object_name
                    << " widget=" << (entry.widget != nullptr ? 1 : 0)
                    << " ogre=" << (entry.ogre_viewport != nullptr ? 1 : 0);
                if (entry.host != nullptr) {
                  out << " host=" << entry.host->width() << 'x'
                      << entry.host->height();
                }
                if (entry.widget != nullptr) {
                  out << " gl=" << entry.widget->width() << 'x'
                      << entry.widget->height()
                      << " gl_vis=" << (entry.widget->isVisible() ? 1 : 0);
                }
                out << '\n';
              });
          out << "viewport_entries=" << vp_count << '\n';
        });
      });
    }
  });
}

void FrameSession::restoreWindowLayout() {
  frame_->layout_->expanded_main_panel_dock_ = nullptr;
  frame_->layout_->pre_expand_visible_main_panels_.clear();
  frame_->layout_->syncMainPanelExpandUi(nullptr);
  const bool skip_window_geometry = LoadAppUiPreferences().start_maximized;
  const std::string geometry_b64 = frame_->manager_->windowGeometryBase64();
  if (!skip_window_geometry) {
    if (!geometry_b64.empty()) {
      frame_->restoreGeometry(QByteArray::fromBase64(
          QByteArray::fromStdString(geometry_b64)));
    } else if (frame_->manager_->windowWidth() > 0 && frame_->manager_->windowHeight() > 0) {
      const int x =
          frame_->manager_->windowX() >= 0 ? frame_->manager_->windowX() : frame_->geometry().x();
      const int y =
          frame_->manager_->windowY() >= 0 ? frame_->manager_->windowY() : frame_->geometry().y();
      frame_->setGeometry(x, y, frame_->manager_->windowWidth(), frame_->manager_->windowHeight());
    }
  }
  const std::string state_b64 = frame_->manager_->windowStateBase64();
  if (!state_b64.empty()) {
    frame_->restoreState(QByteArray::fromBase64(QByteArray::fromStdString(state_b64)));
  }
  // Saved layouts may still place Teleop on the left from older builds.
  if (frame_->panels_->teleop_dock_ != nullptr) {
    frame_->layout_->ensureSidebarDockAttached(frame_->panels_->teleop_dock_);
  }
  // Ensure main-column docks are owned by the center host before mosaic restore.
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || !frame_->layout_->isMainPanel(dock)) {
      continue;
    }
    frame_->layout_->ensureMainPanelDockAttached(dock);
  }
  frame_->chrome_->syncToolbarToActiveTool();
  frame_->viewport_->syncViewportTitleBarTools();

  const QByteArray mosaic = QByteArray::fromBase64(
      QByteArray::fromStdString(frame_->manager_->mainPanelStateBase64()));
  const bool have_mosaic = mosaic.startsWith("mosaic:v1:");
  // Mark manual BEFORE applyPanelVisibility so its scheduleTileCenterPanels
  // is a no-op; otherwise a queued tile destroys the mosaic and can SIGSEGV
  // while reparenting docks that host QListView title menus.
  if (have_mosaic) {
    frame_->layout_->center_manual_layout_ = true;
    ++frame_->layout_->center_tile_epoch_;
    frame_->layout_->center_tile_pending_ = false;
  }

  restorePanelLayouts();
  applyPanelVisibility();
  frame_->layout_->restoreDockHideState();

  ++frame_->layout_->center_tile_epoch_;
  frame_->layout_->center_tile_pending_ = false;

  bool restored_mosaic = false;
  if (have_mosaic && frame_->layout_->main_panel_host_ != nullptr) {
    frame_->layout_->center_manual_layout_ = true;
    // Park native Ogre widgets off the docks before mosaic reparent — do not
    // destroy/recreate (that races with NVIDIA GL teardown and PointCloud GPU).
    frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
      frame_->viewport_->parkRenderWindowInEntry(entry);
    });
    restored_mosaic = frame_->layout_->main_panel_host_->restoreMosaicLayout(
        mosaic, [this](const QString& object_name) -> QDockWidget* {
          return frame_->findChild<PanelDockWidget*>(object_name);
        });
    frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
      frame_->viewport_->reinstallRenderWindowInEntry(entry);
    });
  }
  if (restored_mosaic) {
    frame_->layout_->syncCenterLayout();
  } else {
    frame_->layout_->center_manual_layout_ = false;
    frame_->layout_->scheduleTileCenterPanels();
  }
}

void FrameSession::restorePanelLayouts() {
  for (const auto& entry : frame_->manager_->panelLayouts()) {
    auto* dock = frame_->findChild<PanelDockWidget*>(
        QString::fromStdString(NormalizePanelObjectName(entry.object_name)));
    if (dock != nullptr) {
      dock->setCollapsed(entry.collapsed);
    }
  }
}

void FrameSession::setupRecordDropOverlay() {
  record_drop_overlay_ = new RecordDropOverlay(frame_);
  layoutRecordDropOverlay();
}

void FrameSession::layoutRecordDropOverlay() {
  if (record_drop_overlay_ == nullptr) {
    return;
  }
  record_drop_overlay_->setGeometry(frame_->rect());
  record_drop_overlay_->raise();
}

void FrameSession::showRecordDropOverlay(bool visible) {
  if (record_drop_overlay_ == nullptr) {
    return;
  }
  if (visible) {
    layoutRecordDropOverlay();
    record_drop_overlay_->show();
    record_drop_overlay_->raise();
  } else {
    record_drop_overlay_->hide();
  }
}

bool FrameSession::handleRecordMime(const QMimeData* mime, bool drop) {
  const QStringList paths = LocalRecordSourcePaths(mime);
  if (paths.isEmpty()) {
    return false;
  }
  if (drop) {
    showRecordDropOverlay(false);
    openRecordFile(paths.front());
  } else {
    showRecordDropOverlay(true);
  }
  return true;
}

void FrameSession::onOpenRecord() {
  const QString path = QFileDialog::getOpenFileName(
      frame_, frame_->tr("Open Record"), QString(),
      frame_->tr("Autolink Record (*.record);;Legacy Bag (*.bag);;MCAP (*.mcap);;All Files (*)"));
  if (!path.isEmpty()) {
    openRecordFile(path);
  }
}

bool FrameSession::openRecordFile(const QString& path) {
  if (path.isEmpty() || frame_->manager_ == nullptr) {
    return false;
  }
  integration::PlaybackController& playback = frame_->manager_->playback();
  playback.stop();

  const RecordSourceKind kind = ClassifyRecordSource(path);
  OpenRecordResult result = OpenRecordSource(&playback, path);
  if (!result.ok &&
      (kind == RecordSourceKind::kBag || kind == RecordSourceKind::kMcap)) {
    ImportRecordDialog dialog(&playback, frame_);
    dialog.setSourcePath(path);
    if (dialog.exec() != QDialog::Accepted || !dialog.recordOpened()) {
      return false;
    }
    result.ok = true;
  }
  if (!result.ok) {
    QMessageBox::warning(frame_, frame_->tr("Open Record"), result.error);
    return false;
  }

  // Bind Displays to record channels and let the UI thread create readers
  // before the player writers start (avoids write-failed storms / races).
  const int added = frame_->manager_->ensureDisplaysForRecordChannels(
      playback.channelTypes());
  if (frame_->panels_ != nullptr &&
      frame_->panels_->displays_panel_ != nullptr) {
    frame_->panels_->displays_panel_->refreshStatus();
  }
  frame_->manager_->update();
  QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

  // Skip seekTo/preview before first play: previewAtLocked creates ephemeral
  // Writers then destroys them, racing topology callbacks into freed Writer*.
  if (!playback.play(1.0, false)) {
    QMessageBox::warning(
        frame_, frame_->tr("Open Record"),
        frame_->tr("Opened %1 but failed to start playback.")
            .arg(QFileInfo(result.record_path).fileName()));
    return false;
  }
  if (frame_->panels_ != nullptr) {
    if (frame_->panels_->record_dock_ == nullptr) {
      frame_->panels_->record_dock_ = frame_->panels_->createRecordPanelDock();
      frame_->layout_->addSidebarDock(frame_->panels_->record_dock_,
                                      Qt::RightDockWidgetArea);
      if (frame_->panels_->views_dock_ != nullptr) {
        frame_->tabifyDockWidget(frame_->panels_->views_dock_,
                                 frame_->panels_->record_dock_);
      }
    }
    frame_->layout_->ensureSidebarDockAttached(frame_->panels_->record_dock_);
    frame_->panels_->record_dock_->show();
    frame_->panels_->record_dock_->raise();
    if (frame_->panels_->record_panel_ != nullptr) {
      frame_->panels_->record_panel_->reloadFromPlayback();
    }
  }
  if (frame_->statusBar() != nullptr) {
    QString message =
        frame_->tr("Playing %1").arg(QFileInfo(result.record_path).fileName());
    if (added > 0) {
      message +=
          frame_->tr(" · added %1 display(s)").arg(added);
    }
    frame_->statusBar()->showMessage(message, 5000);
  }
  return true;
}

void FrameSession::dragEnterEvent(QDragEnterEvent* event) {
  if (handleRecordMime(event->mimeData(), false)) {
    event->acceptProposedAction();
    return;
  }
}

void FrameSession::dragMoveEvent(QDragMoveEvent* event) {
  if (handleRecordMime(event->mimeData(), false)) {
    event->acceptProposedAction();
    return;
  }
}

void FrameSession::dragLeaveEvent(QDragLeaveEvent* event) {
  showRecordDropOverlay(false);
}

void FrameSession::dropEvent(QDropEvent* event) {
  if (handleRecordMime(event->mimeData(), true)) {
    event->acceptProposedAction();
    return;
  }
}

bool FrameSession::eventFilter(QObject* watched, QEvent* event) {
  auto* widget = qobject_cast<QWidget*>(watched);
  if (widget == nullptr ||
      (widget != frame_ && widget != record_drop_overlay_ &&
       !frame_->isAncestorOf(widget))) {
    return false; // base handled by VisualizationFrame
  }

  switch (event->type()) {
    case QEvent::DragEnter:
    case QEvent::DragMove: {
      auto* drag = static_cast<QDragMoveEvent*>(event);
      if (handleRecordMime(drag->mimeData(), false)) {
        drag->acceptProposedAction();
        return true;
      }
      break;
    }
    case QEvent::Drop: {
      auto* drop = static_cast<QDropEvent*>(event);
      if (handleRecordMime(drop->mimeData(), true)) {
        drop->acceptProposedAction();
        return true;
      }
      break;
    }
    case QEvent::DragLeave:
      if (!frame_->rect().contains(frame_->mapFromGlobal(QCursor::pos()))) {
        showRecordDropOverlay(false);
      }
      break;
    default:
      break;
  }
  return false; // base handled by VisualizationFrame
}

void FrameSession::onOpenConfig() {
  const QString path = QFileDialog::getOpenFileName(
      frame_, frame_->tr("Open Config"), QString(),
      frame_->tr("Autoviz Config (*.autoviz *.yaml);;RViz Config (*.rviz);;All Files (*)"));
  if (path.isEmpty()) {
    return;
  }
  if (!loadConfig(path)) {
    QMessageBox::warning(frame_, frame_->tr("Open Config"),
                         frame_->tr("Failed to load config:\n%1").arg(path));
  }
}

void FrameSession::onSaveConfig() {
  if (config_path_.isEmpty()) {
    onSaveConfigAs();
    return;
  }
  if (!saveConfig(config_path_)) {
    QMessageBox::warning(frame_, frame_->tr("Save Config"),
                         frame_->tr("Failed to save config:\n%1").arg(config_path_));
    return;
  }
  clearConfigModified();
  markRecentConfig(config_path_);
}

void FrameSession::onSaveConfigAs() {
  const QString path = QFileDialog::getSaveFileName(
      frame_, frame_->tr("Save Autoviz Config"), config_path_, frame_->tr("Autoviz Config (*.autoviz)"));
  if (path.isEmpty()) {
    return;
  }
  if (!saveConfig(path)) {
    QMessageBox::warning(frame_, frame_->tr("Save Config"),
                         frame_->tr("Failed to save config:\n%1").arg(path));
    return;
  }
  clearConfigModified();
  markRecentConfig(path);
}

void FrameSession::onFixedFrameChanged(const QString& /*frame*/) {
  frame_->viewport_->syncToolContext();
  frame_->chrome_->updateStatusBar();
}

void FrameSession::onRenderTick() {
  if (app_inactive_) {
    return;
  }
  const qint64 elapsed_ms = render_elapsed_.restart();
  // After backgrounding, elapsed can be huge; clamp so FPS/view ticks don't
  // jump and we don't try to "catch up" expensive work in one frame.
  const float delta_seconds =
      static_cast<float>(std::min<qint64>(elapsed_ms, 100)) / 1000.f;
  frame_->viewport_->syncToolContext();
  frame_->manager_->update();
  frame_->viewport_->viewportTick(delta_seconds);
  if (rendering::ViewController* controller = frame_->viewport_->activeViewController()) {
    controller->appendFocalShape(&frame_->manager_->sceneOverlay());
  }
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || !dock->isVisible() ||
        frame_->panels_->panelTypeId(dock) != QLatin1String("ImageDock")) {
      continue;
    }
    if (auto* panel = qobject_cast<image::ImagePanel*>(dock->widget())) {
      panel->tick();
    }
  }
  frame_->chrome_->updateFps();
}

void FrameSession::onRefreshTick() {
  if (app_inactive_) {
    return;
  }
  frame_->manager_->refreshChannelList();
  // Remember channels already considered so a display the user removed is not
  // recreated on the next refresh. Kept out of FrameSession so the class
  // layout stays stable for the rest of the binary.
  static std::unordered_set<std::string> live_displays_seen;
  std::unordered_map<std::string, std::string> fresh_channels;
  for (const auto& channel : frame_->manager_->channels()) {
    if (channel.message_type.empty() ||
        live_displays_seen.count(channel.channel_name) != 0) {
      continue;
    }
    fresh_channels.emplace(channel.channel_name, channel.message_type);
  }
  if (!fresh_channels.empty()) {
    frame_->manager_->ensureDisplaysForRecordChannels(fresh_channels);
    for (const auto& entry : fresh_channels) {
      live_displays_seen.insert(entry.first);
    }
  }
  frame_->panels_->displays_panel_->refreshStatus();
  if (frame_->panels_->views_panel_ != nullptr) {
    frame_->panels_->views_panel_->refreshFrameList();
  }
  frame_->chrome_->updateChannelList();
  if (frame_->panels_->tf_tree_panel_ != nullptr) {
    frame_->panels_->tf_tree_panel_->refresh();
  }
  frame_->chrome_->updateStatusBar();
}

void FrameSession::onAboutToQuit() {
  if (quit_teardown_done_) {
    return;
  }
  quit_teardown_done_ = true;
  // Stop timers / playback, then release display GPU objects while Ogre
  // windows still exist. Do NOT manager_->shutdown() here (Autolink/docks).
  render_timer_.stop();
  refresh_timer_.stop();
  if (frame_->manager_ != nullptr) {
    frame_->manager_->playback().stop();
    frame_->manager_->detachDisplaysFromScene();
  }
  frame_->viewport_->forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->hideNativeSurface();
      if (auto* host = entry.ogre_viewport->ogreSceneHost()) {
        host->clear();
      }
    }
  });
  // Sync-destroy before ~VisualizationFrame walks dock children — deferred
  // deleteLater leaves GLX widgets for QObject teardown and SIGSEGVs.
  frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
    frame_->viewport_->destroyRenderWindowInEntry(entry, /*synchronous=*/true);
  });

  QSettings settings;
  settings.setValue(QStringLiteral("recent_configs"), recent_configs_);
}

void FrameSession::leaveEvent(QEvent* event) {
  // RViz2 clears the tool status when the cursor leaves the window.
  frame_->chrome_->status_hint_.clear();
  if (frame_->chrome_->status_label_ != nullptr) {
    frame_->chrome_->status_label_->clear();
  }
}

void FrameSession::onReset() {
  if (frame_->manager_ != nullptr) {
    frame_->manager_->resetTime();
  }
  // Persistent Ogre geometry is not rebuilt every frame; clear hosts now so
  // emptied Displays do not leave stale meshes/point clouds on screen.
  frame_->viewport_->forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport != nullptr) {
      if (auto* host = entry.ogre_viewport->ogreSceneHost()) {
        host->clear();
      }
    }
  });
  if (frame_->panels_->time_panel_ != nullptr) {
    frame_->panels_->time_panel_->syncAfterReset();
  }
  frame_->viewport_->requestViewportUpdate();
}

void FrameSession::resizeEvent(QResizeEvent* event) {
  Q_UNUSED(event);
  // Never retile on resize — that would destroy a directed mosaic.
  // Always scale-fill existing splitter ratios so 3D View + Map stay edge-to
  // edge when the window grows (manual layout used to skip this and left a gap).
  if (frame_->layout_->main_panel_host_ != nullptr &&
      !frame_->layout_->center_tiling_ &&
      !frame_->layout_->suppress_center_tile_) {
    frame_->layout_->main_panel_host_->syncHorizontalDockLayout();
  }
  layoutRecordDropOverlay();
}

void FrameSession::applyBackgroundColor(const QColor& color) {
  frame_->viewport_->forEachViewportPanel([color](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->setBackgroundColor(color);
    }
  });
  frame_->viewport_->requestViewportUpdate();
}

void FrameSession::updateRecentConfigMenu() {
  if (frame_->chrome_->recent_configs_menu_ == nullptr) {
    return;
  }
  frame_->chrome_->recent_configs_menu_->clear();
  const QString home = QDir::homePath();
  for (const QString& path : recent_configs_) {
    if (path.isEmpty()) {
      continue;
    }
    QString display_name = path;
    if (display_name.startsWith(home)) {
      display_name =
          QStringLiteral("~/") + display_name.mid(home.size() + 1);
    }
    auto* action = frame_->chrome_->recent_configs_menu_->addAction(display_name, frame_, &VisualizationFrame::onRecentConfigSelected);
    action->setData(path);
  }
}

void FrameSession::markRecentConfig(const QString& path) {
  if (path.isEmpty()) {
    return;
  }
  recent_configs_.removeAll(path);
  recent_configs_.prepend(path);
  while (recent_configs_.size() > kRecentConfigCount) {
    recent_configs_.removeLast();
  }
  updateRecentConfigMenu();
}

void FrameSession::onRecentConfigSelected() {
  auto* action = qobject_cast<QAction*>(frame_->sender());
  if (action == nullptr) {
    return;
  }
  const QString path = action->data().toString();
  if (path.isEmpty()) {
    return;
  }
  if (!loadConfig(path)) {
    QMessageBox::warning(frame_, frame_->tr("Open Config"),
                         frame_->tr("Failed to load config:\n%1").arg(path));
  }
}

void FrameSession::onResetDefaultLayout() {
  frame_->layout_->applyDefaultDockLayout();
  frame_->manager_->setToolbarTools({});
  frame_->manager_->tools().setActiveTool("Interact");
  frame_->chrome_->rebuildToolbar();
  frame_->chrome_->syncActiveToolUi();
  markConfigModified();
}

void FrameSession::applyShortcutPreferences(
    const QHash<QString, QKeySequence>& shortcuts) {
  const auto apply = [&](QAction* action, const QString& id,
                         const QKeySequence& fallback) {
    if (action == nullptr) {
      return;
    }
    const QKeySequence sequence =
        shortcuts.contains(id) ? shortcuts.value(id) : fallback;
    action->setShortcut(sequence);
  };

  apply(frame_->chrome_->open_config_action_, QStringLiteral("file.open"), QKeySequence::Open);
  apply(frame_->chrome_->open_record_action_, QStringLiteral("file.open_record"),
        QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_O));
  apply(frame_->chrome_->save_config_action_, QStringLiteral("file.save"), QKeySequence::Save);
  apply(frame_->chrome_->save_config_as_action_, QStringLiteral("file.save_as"),
        QKeySequence::SaveAs);
  apply(frame_->chrome_->quit_action_, QStringLiteral("file.quit"), QKeySequence::Quit);
  if (frame_->chrome_->add_panel_action_ != nullptr) {
    frame_->chrome_->add_panel_action_->setShortcut(QKeySequence());
  }
  if (frame_->chrome_->fullscreen_action_ != nullptr) {
    frame_->chrome_->fullscreen_action_->setShortcut(QKeySequence());
  }
  frame_->chrome_->setupToolShortcuts();
}

void FrameSession::applyUiPreferences(const AppSettingsResult& settings,
                                            const AppUiPreferences& previous_ui) {
  AppUiPreferences ui_preferences;
  ui_preferences.language_code = settings.language_code;
  ui_preferences.start_maximized = settings.start_maximized;
  ui_preferences.shortcuts = settings.shortcuts;
  SaveAppUiPreferences(ui_preferences);

  if (ui_preferences.start_maximized) {
    frame_->showMaximized();
  }

  if (QApplication* app = qobject_cast<QApplication*>(QApplication::instance())) {
    ApplyAppTheme(*app);
    if (ui_preferences.language_code != previous_ui.language_code) {
      InstallAppTranslations(*app, ui_preferences.language_code);
    }
  }
  applyShortcutPreferences(ui_preferences.shortcuts);
}

void FrameSession::onAppSettings() {
  AppSettingsDialog dialog(frame_->manager_.get(), frame_);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }

  const AppUiPreferences previous_ui = LoadAppUiPreferences();
  const AppSettingsResult settings = dialog.resultValues();
  applyUiPreferences(settings, previous_ui);

  if (!settings.fixed_frame.empty()) {
    frame_->manager_->setFixedFrame(settings.fixed_frame);
    frame_->onFixedFrameChanged(QString::fromStdString(settings.fixed_frame));
  }

  if (!settings.transformer_id.empty()) {
    frame_->manager_->transformationManager().setTransformer(settings.transformer_id);
  }

  if (!settings.render_backend.empty() &&
      settings.render_backend != frame_->manager_->renderBackendName()) {
    frame_->manager_->setRenderBackendName(settings.render_backend);
    frame_->viewport_->applyRenderBackend(QString::fromStdString(settings.render_backend));
    syncRenderBackendMenu(QString::fromStdString(settings.render_backend));
  }

  frame_->manager_->setTargetFrameRate(settings.frame_rate);
  frame_->viewport_->applyTargetFrameRate(settings.frame_rate);

  frame_->manager_->setBackgroundColor(settings.background_color);
  frame_->applyBackgroundColor(common::ParseColorProperty(settings.background_color,
                                                  QColor(48, 48, 48)));

  frame_->manager_->setTimeSyncMode(settings.time_sync_mode);
  frame_->manager_->setTimePaused(settings.time_paused);
  frame_->manager_->setPlotSettingsVisible(settings.plot_settings_visible);
  frame_->layout_->showPropertyInspector(settings.plot_settings_visible);

  frame_->manager_->setDockHideState(settings.hide_left_dock, settings.hide_right_dock);
  frame_->layout_->hideLeftDock(settings.hide_left_dock);
  frame_->layout_->hideRightDock(settings.hide_right_dock);
  frame_->chrome_->syncToolbarLayoutControls();

  if (frame_->panels_->displays_panel_ != nullptr) {
    frame_->panels_->displays_panel_->refresh();
  }
  frame_->viewport_->requestViewportUpdate();
  markConfigModified();
}

void FrameSession::onHelpAbout() {
  QString about_text =
      frame_->tr("This is Autoviz.\n\nCompiled against Qt version %1.")
          .arg(QLatin1String(QT_VERSION_STR));
  about_text += frame_->tr("\nViewport: Ogre 1.x (required).");
  QMessageBox::about(QApplication::activeWindow(), frame_->tr("About"), about_text);
}

void FrameSession::onToggleFullscreen() {
  setFullScreen(!frame_->windowState().testFlag(Qt::WindowFullScreen));
}

void FrameSession::onBackendOgre() {
  frame_->manager_->setRenderBackendName("Ogre");
  frame_->viewport_->applyRenderBackend(QStringLiteral("Ogre"));
  syncRenderBackendMenu(QStringLiteral("Ogre"));
  markConfigModified();
}

void FrameSession::syncRenderBackendMenu(const QString& /*name*/) {
  if (frame_->chrome_->backend_ogre_action_ != nullptr) {
    frame_->chrome_->backend_ogre_action_->setChecked(true);
    frame_->chrome_->backend_ogre_action_->setEnabled(true);
  }
}

void FrameSession::syncOffscreenPause() {
  const bool offscreen = frame_->isMinimized();
  if (offscreen == app_inactive_) {
    return;
  }
  app_inactive_ = offscreen;
  // Keep accepting live messages while Autoviz is open. Mouse leaving the
  // Image/3D native windows must not freeze Image, 3D, or TF.
  integration::MessageQueue::setAcceptIncoming(true);
  if (offscreen) {
    setRenderingPaused(true);
    return;
  }
  render_elapsed_.restart();
  if (frame_->panels_->displays_panel_ != nullptr) {
    frame_->panels_->displays_panel_->setLiveUpdatesPaused(false);
  }
  if (frame_->panels_->tf_tree_panel_ != nullptr) {
    frame_->panels_->tf_tree_panel_->setPaused(false);
  }
  setRenderingPaused(false);
}

void FrameSession::changeEvent(QEvent* event) {
  if (event != nullptr && (event->type() == QEvent::WindowStateChange ||
                           event->type() == QEvent::Hide ||
                           event->type() == QEvent::Show)) {
    syncOffscreenPause();
  }
}

void FrameSession::setRenderingPaused(bool paused) {
  if (paused) {
    render_timer_.stop();
    refresh_timer_.stop();
  } else {
    frame_->viewport_->applyTargetFrameRate(frame_->manager_->targetFrameRate());
    if (!refresh_timer_.isActive()) {
      refresh_timer_.setTimerType(Qt::PreciseTimer);
      refresh_timer_.start(1000);
    }
  }
}

}  // namespace autoviz
