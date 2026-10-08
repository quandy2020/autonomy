/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_session.hpp"
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
  frame_->viewport_->applyRenderBackend(QString::fromStdString(frame_->manager_->renderBackendName()));
  applyCurrentView();
  frame_->chrome_->rebuildToolbar();
  restoreWindowLayout();
  frame_->panels_->restorePlotPanelConfigs();
  frame_->panels_->restoreTablePanelConfigs();
  frame_->panels_->restoreChannelGraphPanelConfigs();
  frame_->panels_->restoreTfTreePanelConfigs();
  frame_->panels_->restoreImagePanelConfigs();
  frame_->panels_->restorePublishPanelConfigs();
  frame_->panels_->restoreServicePanelConfigs();
  frame_->panels_->restoreTeleopPanelConfigs();
  frame_->panels_->restoreMapPanelConfigs();
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
  if (rendering::ViewController* controller = frame_->viewport_->activeViewController()) {
    controller->setState(common::ToViewState(frame_->manager_->currentView()));
    if (frame_->panels_->views_panel_ != nullptr) {
      frame_->panels_->views_panel_->refreshFromController();
    }
    frame_->viewport_->requestViewportUpdate();
  }
}

void FrameSession::captureWindowLayout() {
  frame_->chrome_->rebuildToolbar();
  frame_->manager_->setWindowLayout(
      frame_->saveState().toBase64().constData(),
      frame_->saveGeometry().toBase64().constData());
  if (frame_->layout_->main_panel_host_ != nullptr) {
    frame_->manager_->setMainPanelLayout(
        frame_->layout_->main_panel_host_->saveState().toBase64().constData());
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
    return;
  }

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
  // Center column uses dynamic grid tiling — ignore legacy MainPanelState docks.
  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || !frame_->layout_->isMainPanel(dock)) {
      continue;
    }
    frame_->layout_->ensureMainPanelDockAttached(dock);
  }
  frame_->chrome_->syncToolbarToActiveTool();
  frame_->viewport_->syncViewportTitleBarTools();
  restorePanelLayouts();
  applyPanelVisibility();
  frame_->layout_->restoreDockHideState();
  frame_->layout_->scheduleTileCenterPanels();
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
  const RecordSourceKind kind = ClassifyRecordSource(path);
  OpenRecordResult result = OpenRecordSource(&frame_->manager_->playback(), path);
  if (!result.ok &&
      (kind == RecordSourceKind::kBag || kind == RecordSourceKind::kMcap)) {
    ImportRecordDialog dialog(&frame_->manager_->playback(), frame_);
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

  frame_->manager_->playback().play(1.0, false);
  if (frame_->statusBar() != nullptr) {
    frame_->statusBar()->showMessage(
        frame_->tr("Playing %1").arg(QFileInfo(path).fileName()), 4000);
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
  // Stop timers / playback only. Do NOT frame_->manager_->shutdown() here: Qt has not
  // destroyed docks/GL widgets yet, and they still touch displays / Autolink.
  render_timer_.stop();
  refresh_timer_.stop();
  if (frame_->manager_ != nullptr) {
    frame_->manager_->playback().stop();
  }

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
  // Never retile on resize — dock geometry changes would loop forever.
  // Also never equalize after directed Split: docks are not a regular grid.
  if (frame_->layout_->main_panel_host_ != nullptr && !frame_->layout_->center_tiling_ && !frame_->layout_->center_manual_layout_ &&
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
