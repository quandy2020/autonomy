/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_session.hpp"
#include "autoviz/ui/frame_session_internal.hpp"
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/common/selection.hpp"
#include <algorithm>
#include <cstdio>
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
#include <QElapsedTimer>
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
#include <QUrl>
#include <QPainter>
#include <QPen>
#include <QPointer>
#include <QResizeEvent>
#include <QScreen>
#include <QSettings>
#include <QWindow>
#include <QShortcut>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QSize>
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
#include "autoviz/common/interaction/message_queue.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/render_system.hpp"
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
#include "autoviz/ui/plot/io/plot_config_io.hpp"
#include "autoviz/ui/publish/publish_config_io.hpp"
#include "autoviz/ui/image/image_config_io.hpp"
#include "autoviz/ui/plot/panel/plot_panel.hpp"
#include "autoviz/ui/teleop/teleop_panel.hpp"
#include "autoviz/ui/navigation/navigation_panel.hpp"
#include "autoviz/ui/raw/panel.hpp"
#include "autoviz/ui/channels/channels_panel.hpp"
#include "autoviz/ui/record/record_panel.hpp"
#include "autoviz/ui/evidence/evidence_panel.hpp"
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
#include "autoviz/ui/record_drop_overlay.hpp"
#include "autoviz/ui/theme/tokens.hpp"

#include <QAbstractAnimation>
#include <QEasingCurve>
#include <QGraphicsOpacityEffect>
#include <QPropertyAnimation>


namespace autoviz {

using frame_session_detail::CollectMosaicLeafNames;
using frame_session_detail::IsViewportDockObjectName;
bool FrameSession::loadConfig(const QString& path) {
  suppress_config_modified_ = true;
  // Always freeze GLX before dock/mosaic churn — both first launch (config
  // applied while still hidden) and mid-session Open Config. Remapping while
  // docks shuffle flashes transparent background_pixel=0 3D panes.
  const bool live_reload = frame_->isVisible();
  if (frame_->viewport_ != nullptr) {
    frame_->viewport_->setViewportRevealHeld(true);
  }

  // Live reload: opaque cover + frozen paints so the user never sees dock
  // reparent / charcoal / GL remount tearing (双窗撕裂闪烁).
  QWidget* mosaic_host = (frame_->layout_ != nullptr)
                             ? static_cast<QWidget*>(frame_->layout_->main_panel_host_)
                             : nullptr;
  const bool freeze_ui = live_reload;
  if (freeze_ui) {
    showLayoutTransitionCover();
    frame_->setUpdatesEnabled(false);
    if (mosaic_host != nullptr) {
      mosaic_host->setUpdatesEnabled(false);
    }
  }
  const auto unfreeze_ui = [this, mosaic_host, freeze_ui]() {
    if (!freeze_ui) {
      return;
    }
    if (mosaic_host != nullptr) {
      mosaic_host->setUpdatesEnabled(true);
    }
    frame_->setUpdatesEnabled(true);
    layoutLayoutTransitionCover();
    if (layout_transition_cover_ != nullptr) {
      layout_transition_cover_->raise();
    }
  };

  if (!frame_->manager_->loadSession(path.toStdString())) {
    unfreeze_ui();
    dismissLayoutTransitionCover();
    if (frame_->viewport_ != nullptr) {
      scheduleViewportRevealAfterLayoutSettle();
    }
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
  // One paint of the final dock/mosaic chrome, then deferred GL reveal.
  unfreeze_ui();
  // Mid-session Open Config: reveal after mosaic settles. Startup leaves hold
  // in place — applyStartupWindowState() schedules the first reveal after map.
  if (live_reload) {
    scheduleViewportRevealAfterLayoutSettle();
  }
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

void FrameSession::ensureSessionMainPanels() {
  QSet<QString> needed;
  QSet<QString> mosaic_leaves;
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
          document.object().value(QStringLiteral("root")).toObject(),
          &mosaic_leaves);
      needed.unite(mosaic_leaves);
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
      continue;
    }
    // Sidebar-style panels (Map / Service / Teleop / Navigation …) can also be
    // saved as mosaic panes. Create only those mosaic leaves here so the
    // resolver does not drop them and collapse the restored split tree;
    // ordinary hidden/sidebar entries keep their previous lazy creation.
    if (!mosaic_leaves.contains(name)) {
      continue;
    }
    // showPanelByObjectName() shows the dock and clears the manual-layout
    // flags (Change semantics). Restore them so the queued tile cannot
    // overwrite the mosaic that is about to be built.
    const bool was_manual = frame_->layout_->center_manual_layout_;
    const bool was_suppressed = frame_->layout_->suppress_center_tile_;
    const int epoch = frame_->layout_->center_tile_epoch_;
    frame_->layout_->showPanelByObjectName(name);
    frame_->layout_->center_manual_layout_ = was_manual;
    frame_->layout_->suppress_center_tile_ = was_suppressed;
    frame_->layout_->center_tile_epoch_ = epoch;
    if (PanelDockWidget* dock = frame_->findChild<PanelDockWidget*>(name)) {
      dock->hide();
    }
  }
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

void FrameSession::restoreWindowLayout() {
  // Detach native GLX BEFORE restoreState / mosaic reparent. Remapping
  // WA_PaintOnScreen windows while docks shuffle produces overlapping holes
  // and the "frenzied" layout (duplicate titles, tear, purple garbage).
  if (frame_->viewport_ != nullptr) {
    if (!frame_->viewport_->isViewportRevealHeld()) {
      frame_->viewport_->setViewportRevealHeld(true);
    }
    frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
      frame_->viewport_->parkRenderWindowInEntry(entry);
    });
  }

  frame_->layout_->expanded_main_panel_dock_ = nullptr;
  frame_->layout_->pre_expand_visible_main_panel_docks_.clear();
  frame_->layout_->pre_expand_mosaic_layout_.clear();
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
  // restoreState can clear WindowMaximized; applyStartupWindowState re-applies
  // it after loadConfig. Also ignore session Width/Height when launching
  // maximized (already skipped above) so Docker does not open a 2200x1200 float.
  // Saved layouts may still place Teleop on the left from older builds.
  if (frame_->panels_->teleop_dock_ != nullptr) {
    frame_->layout_->ensureSidebarDockAttached(frame_->panels_->teleop_dock_);
  }
  if (frame_->panels_->navigation_dock_ != nullptr) {
    frame_->layout_->ensureSidebarDockAttached(frame_->panels_->navigation_dock_);
  }
  // Do not attach every main dock here. addPanel() always show()s, which
  // undoes applyDefaultDockLayout and grids Messages/Channels/Image/Plot
  // when the session has no mosaic / VisiblePanels (e.g. radar.autoviz).
  // Mosaic restore and applyPanelVisibility attach only the docks they need.
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
  // Record has no live preview — keep closed unless a record file is open
  // (CLI / Open Record / drop). Older sessions often left it tabbed on the
  // right; do not restore that empty panel as the default right sidebar.
  // Must run after applyPanelVisibility / restoreState so saved "visible"
  // lists cannot reopen it.
  if (frame_->panels_->record_dock_ != nullptr &&
      frame_->manager_->playback().currentFile().empty()) {
    frame_->layout_->ensureSidebarDockAttached(frame_->panels_->record_dock_);
    frame_->panels_->record_dock_->hide();
  }

  ++frame_->layout_->center_tile_epoch_;
  frame_->layout_->center_tile_pending_ = false;

  bool restored_mosaic = false;
  if (have_mosaic && frame_->layout_->main_panel_host_ != nullptr) {
    frame_->layout_->center_manual_layout_ = true;
    // Already parked at entry — keep surfaces off docks during mosaic reparent.
    restored_mosaic = frame_->layout_->main_panel_host_->restoreMosaicLayout(
        mosaic, [this](const QString& object_name) -> QDockWidget* {
          return frame_->findChild<PanelDockWidget*>(object_name);
        });
    LOG(INFO) << "mosaic: restore " << (restored_mosaic ? "ok" : "FAILED")
              << " for session mosaic (" << mosaic.size() << " bytes)";
    // Create deferred GL windows into final panes, but keep them parked.
    // reinstall must not recreate/map under reveal_held_ (loadConfig flicker).
    for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
      if (dock == nullptr || !dock->requestedVisible() ||
          frame_->panels_->panelTypeId(dock) !=
              QLatin1String("ViewportDock")) {
        continue;
      }
      frame_->viewport_->ensureViewportPanelReady(dock);
    }
    frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
      frame_->viewport_->reinstallRenderWindowInEntry(entry);
      frame_->viewport_->parkRenderWindowInEntry(entry);
    });
  }
  if (restored_mosaic) {
    frame_->layout_->syncCenterLayout();
  } else if (frame_->manager_->visiblePanels().empty() && !have_mosaic) {
    // radar.autoviz / other lean configs omit mosaic + VisiblePanels.
    // Keep 3D View in the center; do not grid Messages/Channels/Image/Plot.
    frame_->layout_->applyMainPanelDefaultLayout();
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


}  // namespace autoviz
