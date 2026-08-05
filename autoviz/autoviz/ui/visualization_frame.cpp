/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/visualization_frame.hpp"

#include "autoviz/common/selection.hpp"

#include <algorithm>
#include <QApplication>
#include <QFrame>
#include <QScrollArea>
#include <QStackedWidget>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QDateTime>
#include <QDir>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QResizeEvent>
#include <QSettings>
#include <QShortcut>
#include <QStatusBar>
#include <QSizePolicy>
#include <QTimer>
#include <QCursor>
#include <QVariant>
#include <QTabWidget>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/tool_manager.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"
#include "autoviz/rendering/render_window.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/common/view_state_io.hpp"
#include "autoviz/ui/add_panel_dialog.hpp"
#include "autoviz/ui/panel_catalog.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/displays_panel.hpp"
#include "autoviz/ui/help_panel.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/image/image_panel.hpp"
#include "autoviz/ui/main_panel_host.hpp"
#include "autoviz/ui/panel_role.hpp"
#include "autoviz/ui/property_inspector_panel.hpp"
#include "autoviz/ui/playback_panel.hpp"
#include "autoviz/ui/plot/plot_config_io.hpp"
#include "autoviz/ui/image/image_config_io.hpp"
#include "autoviz/ui/plot/plot_panel.hpp"
#include "autoviz/ui/teleop/teleop_panel.hpp"
#include "autoviz/ui/log/log_panel.hpp"
#include "autoviz/ui/raw_messages_panel.hpp"
#include "autoviz/ui/topics_panel.hpp"
#include "autoviz/ui/problems_panel.hpp"
#include "autoviz/ui/variables_panel.hpp"
#include "autoviz/ui/selection_panel.hpp"
#include "autoviz/ui/tool_properties_panel.hpp"
#include "autoviz/ui/tf_tree_panel.hpp"
#include "autoviz/ui/table/table_panel.hpp"
#include "autoviz/ui/publish/publish_panel.hpp"
#include "autoviz/ui/gauge/gauge_panel.hpp"
#include "autoviz/ui/map/map_panel.hpp"
#include "autoviz/ui/indicator/indicator_panel.hpp"
#include "autoviz/ui/transformation_panel.hpp"
#include "autoviz/ui/strata_floor_panel.hpp"
#include "autoviz/ui/time_panel.hpp"
#include "autoviz/ui/panel_rviz_map.hpp"
#include "autoviz/ui/views_panel.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#ifdef AUTOVIZ_USE_QML_DRONE
#include "autoviz/ui/vehicle_3d_panel.hpp"
#endif

namespace autoviz {

VisualizationFrame::VisualizationFrame(
    std::shared_ptr<common::VisualizationManager> manager, QWidget* parent)
    : QMainWindow(parent), manager_(std::move(manager)) {
  const QIcon app_icon = IconLoader::applicationIcon();
  if (!app_icon.isNull()) {
    setWindowIcon(app_icon);
  }
  setupUi();
  setDockNestingEnabled(true);
  setTabPosition(Qt::AllDockWidgetAreas, QTabWidget::North);
  setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowNestedDocks |
                 QMainWindow::AllowTabbedDocks);
  setupMenu();
  setupToolbar();

  manager_->setRedrawCallback([this]() { requestViewportUpdate(); });
  manager_->setGridVisibilityCallback([this](bool visible) {
    if (gl_viewport_ != nullptr) {
      gl_viewport_->setGridVisible(visible);
    }
    if (ogre_viewport_ != nullptr) {
      ogre_viewport_->setGridVisible(visible);
    }
    requestViewportUpdate();
  });
  manager_->setBackgroundColorCallback([this](const std::string& color) {
    applyBackgroundColor(
        common::ParseColorProperty(color, QColor(48, 48, 48)));
  });
  manager_->setViewControllerCallback([this](const std::string& name) {
    applyViewController(QString::fromStdString(name));
  });
  manager_->setRenderBackendCallback([this](const std::string& name) {
    applyRenderBackend(QString::fromStdString(name));
  });
  manager_->setFrameRateCallback([this](int rate) {
    applyTargetFrameRate(rate);
  });

  createViewport(QString::fromStdString(manager_->renderBackendName()));
  if (gl_viewport_ != nullptr) {
    gl_viewport_->setGridVisible(manager_->showGrid());
  }
  if (ogre_viewport_ != nullptr) {
    ogre_viewport_->setGridVisible(manager_->showGrid());
  }
  applyBackgroundColor(
      common::ParseColorProperty(manager_->backgroundColor(), QColor(48, 48, 48)));

  connect(&render_timer_, &QTimer::timeout, this,
          &VisualizationFrame::onRenderTick);
  applyTargetFrameRate(manager_->targetFrameRate());
  render_elapsed_.start();

  connect(&refresh_timer_, &QTimer::timeout, this,
          &VisualizationFrame::onRefreshTick);
  refresh_timer_.start(1000);

  connect(qApp, &QApplication::aboutToQuit, this,
          &VisualizationFrame::onAboutToQuit);

  connect(displays_panel_, &DisplaysPanel::fixedFrameChanged, this,
          &VisualizationFrame::onFixedFrameChanged);
  connect(displays_panel_, &DisplaysPanel::backgroundColorChanged, this,
          &VisualizationFrame::applyBackgroundColor);
  connect(displays_panel_, &DisplaysPanel::displaysChanged, this,
          [this]() { requestViewportUpdate(); });

  connectConfigModifiedSignals();

  if (manager_->windowStateBase64().empty()) {
    applyDefaultDockLayout();
  } else {
    applyMainPanelDefaultLayout();
  }
  restorePanelLayouts();
  restoreDockHideState();
  ensureTimeDockAtBottom();
  syncDeletePanelMenu();

  connect(time_panel_, &TimePanel::layoutChanged, this, [this]() {
    if (time_dock_ == nullptr || time_dock_->isFloating()) {
      return;
    }
    time_dock_->updateGeometry();
  });
  connect(time_panel_, &TimePanel::resetRequested, this,
          &VisualizationFrame::onReset);

  if (views_panel_ != nullptr) {
    connect(views_panel_, &ViewsPanel::viewsChanged, this,
            &VisualizationFrame::syncViewsToManager);
    connect(views_panel_, &ViewsPanel::viewChanged, this, [this]() {
      if (rendering::ViewController* controller = activeViewController()) {
        manager_->setViewControllerName(controller->typeName().toStdString());
      }
      requestViewportUpdate();
    });
  }

  updateChannelList();
  updateStatusBar();

  manager_->setSelectionChangedCallback(
      [this](const std::vector<common::SelectionEntry>& entries) {
        updateSelectionPanel(entries);
      });
  manager_->setSelectionFocusCallback([this](const QVector3D& target) {
    if (rendering::ViewController* controller = activeViewController()) {
      controller->setTarget(target);
      requestViewportUpdate();
    }
  });

  syncToolContext();
  applyActiveTool(manager_->tools().activeToolId());
}

void VisualizationFrame::updateWindowTitle() {
  QString title = QStringLiteral("Autoviz");
  if (!config_path_.isEmpty()) {
    title += QStringLiteral(" - %1").arg(QFileInfo(config_path_).fileName());
  }
  if (config_modified_) {
    title += QLatin1Char('*');
  }
  setWindowTitle(title);
}

void VisualizationFrame::markConfigModified() {
  if (suppress_config_modified_ || config_modified_) {
    return;
  }
  config_modified_ = true;
  updateWindowTitle();
}

void VisualizationFrame::clearConfigModified() {
  config_modified_ = false;
  updateWindowTitle();
}

void VisualizationFrame::connectConfigModifiedSignals() {
  connect(displays_panel_, &DisplaysPanel::displaysChanged, this,
          &VisualizationFrame::markConfigModified);
  connect(displays_panel_, &DisplaysPanel::fixedFrameChanged, this,
          &VisualizationFrame::markConfigModified);
  connect(displays_panel_, &DisplaysPanel::backgroundColorChanged, this,
          &VisualizationFrame::markConfigModified);

  if (views_panel_ != nullptr) {
    connect(views_panel_, &ViewsPanel::viewsChanged, this,
            &VisualizationFrame::markConfigModified);
    connect(views_panel_, &ViewsPanel::viewChanged, this,
            &VisualizationFrame::markConfigModified);
  }
  if (tool_properties_panel_ != nullptr) {
    connect(tool_properties_panel_, &ToolPropertiesPanel::propertiesChanged, this,
            &VisualizationFrame::markConfigModified);
  }
  if (transformation_panel_ != nullptr) {
    connect(transformation_panel_, &TransformationPanel::transformerChanged, this,
            &VisualizationFrame::markConfigModified);
  }

  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    connect(dock, &QDockWidget::dockLocationChanged, this,
            &VisualizationFrame::markConfigModified);
    connect(dock, &QDockWidget::topLevelChanged, this,
            &VisualizationFrame::markConfigModified);
    connect(dock, &PanelDockWidget::closed, this,
            &VisualizationFrame::markConfigModified);
  }
}

void VisualizationFrame::updateSelectionPanel(
    const std::vector<common::SelectionEntry>& entries) {
  if (selection_panel_ != nullptr) {
    selection_panel_->setSelections(entries);
  }
}

VisualizationFrame::~VisualizationFrame() = default;

void VisualizationFrame::createViewport(const QString& backend) {
  rendering::GpuCapabilities::instance().ensureProbed();

  if (viewport_widget_ != nullptr) {
    if (viewport_layout_ != nullptr) {
      viewport_layout_->removeWidget(viewport_widget_);
    }
    viewport_widget_->hide();
    viewport_widget_->deleteLater();
    viewport_widget_ = nullptr;
    gl_viewport_ = nullptr;
    ogre_viewport_ = nullptr;
  }

  if (backend == QLatin1String("Ogre")) {
#ifdef AUTOVIZ_USE_OGRE
    if (rendering::GpuCapabilities::instance().hasHardwareGpu()) {
      ogre_viewport_ = new rendering::OgreRenderWindow(this);
      ogre_viewport_->setSceneOverlay(&manager_->sceneOverlay());
      ogre_viewport_->setToolManager(&manager_->tools());
      viewport_widget_ = ogre_viewport_;
    } else {
      gl_viewport_ = new rendering::RenderWindow(this);
      gl_viewport_->setSceneOverlay(&manager_->sceneOverlay());
      gl_viewport_->setToolManager(&manager_->tools());
      viewport_widget_ = gl_viewport_;
    }
#else
    gl_viewport_ = new rendering::RenderWindow(this);
    gl_viewport_->setSceneOverlay(&manager_->sceneOverlay());
    gl_viewport_->setToolManager(&manager_->tools());
    viewport_widget_ = gl_viewport_;
#endif
  } else {
    gl_viewport_ = new rendering::RenderWindow(this);
    gl_viewport_->setSceneOverlay(&manager_->sceneOverlay());
    gl_viewport_->setToolManager(&manager_->tools());
    viewport_widget_ = gl_viewport_;
  }
  if (viewport_widget_ != nullptr && viewport_layout_ != nullptr) {
    viewport_layout_->addWidget(viewport_widget_);
  }
  if (gl_viewport_ != nullptr) {
    gl_viewport_->setGridVisible(manager_->showGrid());
    gl_viewport_->setBackgroundColor(
        common::ParseColorProperty(manager_->backgroundColor(),
                                   QColor(48, 48, 48)));
  }
  if (ogre_viewport_ != nullptr) {
    ogre_viewport_->setGridVisible(manager_->showGrid());
    ogre_viewport_->setBackgroundColor(
        common::ParseColorProperty(manager_->backgroundColor(),
                                   QColor(48, 48, 48)));
  }
  if (views_panel_ != nullptr) {
    views_panel_->setViewController(activeViewController());
  }
  if (gl_viewport_ != nullptr) {
    gl_viewport_->viewController().setFrameManager(&manager_->frameManager());
  }
  if (ogre_viewport_ != nullptr) {
    ogre_viewport_->viewController().setFrameManager(&manager_->frameManager());
  }
  connectViewportInteractions();
  setupToolShortcuts();
}

void VisualizationFrame::setupToolShortcuts() {
  qDeleteAll(tool_shortcuts_);
  tool_shortcuts_.clear();
  if (manager_ == nullptr) {
    return;
  }

  const auto shouldIgnoreShortcut = []() {
    QWidget* focus = QApplication::focusWidget();
    return focus != nullptr &&
           (focus->inherits("QLineEdit") || focus->inherits("QPlainTextEdit") ||
            focus->inherits("QTextEdit") || focus->inherits("QSpinBox") ||
            focus->inherits("QDoubleSpinBox"));
  };

  const common::ToolManager& tools = manager_->tools();
  for (const std::string& tool_id : tools.toolIds()) {
    const char letter = tools.shortcutKeyForTool(tool_id);
    if (letter == '\0') {
      continue;
    }
    auto* shortcut =
        new QShortcut(QKeySequence(QString(QChar::fromLatin1(letter))), this);
    shortcut->setContext(Qt::WindowShortcut);
    shortcut->setAutoRepeat(false);
    connect(shortcut, &QShortcut::activated, this, [this, tool_id, shouldIgnoreShortcut]() {
      if (shouldIgnoreShortcut()) {
        return;
      }
      if (manager_->tools().activeToolId() == tool_id) {
        applyActiveTool(manager_->tools().defaultToolId());
      } else {
        applyActiveTool(tool_id);
      }
    });
    tool_shortcuts_.push_back(shortcut);
  }

  auto* escape_shortcut = new QShortcut(QKeySequence(Qt::Key_Escape), this);
  escape_shortcut->setContext(Qt::WindowShortcut);
  escape_shortcut->setAutoRepeat(false);
  connect(escape_shortcut, &QShortcut::activated, this, [this, shouldIgnoreShortcut]() {
    if (shouldIgnoreShortcut()) {
      return;
    }
    if (manager_->tools().activeToolId() != manager_->tools().defaultToolId()) {
      applyActiveTool(manager_->tools().defaultToolId());
    }
  });
  tool_shortcuts_.push_back(escape_shortcut);
}

void VisualizationFrame::connectViewportInteractions() {
  const auto sync_views = [this]() {
    if (views_panel_ != nullptr) {
      views_panel_->refreshFromController();
    }
  };
  if (gl_viewport_ != nullptr) {
    connect(gl_viewport_, &rendering::RenderWindow::viewDragUpdated, this,
            sync_views, Qt::UniqueConnection);
    connect(gl_viewport_, &rendering::RenderWindow::viewDragEnded, this,
            sync_views, Qt::UniqueConnection);
  }
#ifdef AUTOVIZ_USE_OGRE
  if (ogre_viewport_ != nullptr) {
    connect(ogre_viewport_, &rendering::OgreRenderWindow::viewDragUpdated, this,
            sync_views, Qt::UniqueConnection);
    connect(ogre_viewport_, &rendering::OgreRenderWindow::viewDragEnded, this,
            sync_views, Qt::UniqueConnection);
    connect(ogre_viewport_, &rendering::OgreRenderWindow::toolShortcutTriggered,
            this,
            [this]() { syncActiveToolUi(); },
            Qt::UniqueConnection);
  }
#endif
}

void VisualizationFrame::applyTargetFrameRate(int fps) {
  const int clamped = std::clamp(fps, 1, 120);
  const int interval_ms = std::max(1, 1000 / clamped);
  render_timer_.setInterval(interval_ms);
  if (!render_timer_.isActive()) {
    render_timer_.start(interval_ms);
  }
}

rendering::ViewController* VisualizationFrame::activeViewController() {
  if (gl_viewport_ != nullptr) {
    return &gl_viewport_->viewController();
  }
  if (ogre_viewport_ != nullptr) {
    return &ogre_viewport_->viewController();
  }
  return nullptr;
}

void VisualizationFrame::requestViewportUpdate() {
  syncToolContext();
  if (manager_ != nullptr && !manager_->isUpdating()) {
    manager_->update();
  }
  if (viewport_widget_ != nullptr) {
    viewport_widget_->update();
  }
}

void VisualizationFrame::viewportTick(float delta_seconds) {
  syncToolContext();
  if (gl_viewport_ != nullptr) {
    gl_viewport_->tick(delta_seconds);
  }
  if (ogre_viewport_ != nullptr) {
    ogre_viewport_->tick(delta_seconds);
  }
}

void VisualizationFrame::syncToolContext() {
#ifdef AUTOVIZ_USE_OGRE
  if (manager_ != nullptr) {
    manager_->displayContext().ogre_scene_host =
        ogre_viewport_ != nullptr ? ogre_viewport_->ogreSceneHost() : nullptr;
  }
#endif
  common::ToolContext context;
  context.view_controller = activeViewController();
  context.scene_overlay = &manager_->sceneOverlay();
  if (viewport_widget_ != nullptr) {
    context.viewport_width = viewport_widget_->width();
    context.viewport_height = viewport_widget_->height();
  }
  if (manager_ != nullptr) {
    auto& display_context = manager_->displayContext();
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
  if (context.gpu_picking_enabled) {
    if (gl_viewport_ != nullptr) {
      rendering::RenderWindow* viewport = gl_viewport_;
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
    } else if (ogre_viewport_ != nullptr) {
      rendering::OgreRenderWindow* viewport = ogre_viewport_;
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
#ifdef AUTOVIZ_USE_OGRE
  context.sync_ogre_host = [this]() {
    if (manager_ != nullptr && ogre_viewport_ != nullptr) {
      manager_->displayContext().ogre_scene_host =
          ogre_viewport_->ogreSceneHost();
    }
  };
#endif
  context.set_status = [this](const QString& text) {
    if (status_label_ != nullptr) {
      status_label_->setText(text);
    }
  };
  context.autolink_node = manager_->autolinkNode();
  context.fixed_frame = manager_->fixedFrame();
  context.scene_overlay = &manager_->sceneOverlay();
  context.display_context = &manager_->displayContext();
  context.selection_manager = &manager_->selectionManager();
  context.pick_registry = &manager_->pickRegistry();
  context.handler_manager = &manager_->handlerManager();
  context.interactive_markers = &manager_->interactiveMarkerRegistry();
  context.selections_changed = [this](
                                    const std::vector<common::SelectionEntry>&
                                        entries) { updateSelectionPanel(entries); };
  context.revert_to_default_tool = [this]() {
    applyActiveTool(manager_->tools().defaultToolId());
  };
  manager_->tools().setContext(std::move(context));
  if (gl_viewport_ != nullptr) {
    gl_viewport_->setToolManager(&manager_->tools());
  }
  if (ogre_viewport_ != nullptr) {
    ogre_viewport_->setToolManager(&manager_->tools());
  }
}

void VisualizationFrame::setupCentralContainer() {
  main_panel_host_ = new MainPanelHost(this);
  main_panel_host_->setMinimumSize(0, 0);
  main_panel_host_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setCentralWidget(main_panel_host_);
  viewport_host_ = nullptr;
  viewport_layout_ = nullptr;
}

void VisualizationFrame::setupMainPanelHost() {}

QMainWindow* VisualizationFrame::dockHostForPanel(
    const PanelDockWidget* dock) const {
  if (dock != nullptr && isMainPanel(dock)) {
    return main_panel_host_;
  }
  return const_cast<VisualizationFrame*>(this);
}

bool VisualizationFrame::isMainPanel(const PanelDockWidget* dock) const {
  if (dock == nullptr) {
    return false;
  }
  return dock->property("panelRole").toString() == PanelRoleMain();
}

void VisualizationFrame::configureMainPanelDock(PanelDockWidget* dock) {
  if (dock == nullptr || dock == time_dock_) {
    return;
  }
  dock->setProperty("panelRole", PanelRoleMain());
  dock->setAllowedAreas(Qt::AllDockWidgetAreas);
  dock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                    QDockWidget::DockWidgetFloatable);
}

void VisualizationFrame::configureSidebarDock(PanelDockWidget* dock,
                                              Qt::DockWidgetArea area) {
  if (dock == nullptr || dock == time_dock_) {
    return;
  }
  dock->setProperty("panelRole", PanelRoleSidebar());
  dock->setAllowedAreas(area);
  dock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                    QDockWidget::DockWidgetFloatable);
}

void VisualizationFrame::addMainPanelDock(PanelDockWidget* dock,
                                          Qt::DockWidgetArea area) {
  if (dock == nullptr || main_panel_host_ == nullptr) {
    return;
  }
  configureMainPanelDock(dock);
  main_panel_host_->addDockWidget(area, dock);
  wireMainPanelExpandTracking(dock);
}

void VisualizationFrame::ensureMainPanelDockAttached(
    PanelDockWidget* dock, Qt::DockWidgetArea area) {
  if (dock == nullptr || main_panel_host_ == nullptr || !isMainPanel(dock)) {
    return;
  }
  configureMainPanelDock(dock);
  if (main_panel_host_->dockWidgetArea(dock) == Qt::NoDockWidgetArea) {
    main_panel_host_->addDockWidget(area, dock);
  }
  dock->setCollapsed(false);
}

Qt::DockWidgetArea VisualizationFrame::defaultSidebarArea(
    const PanelDockWidget* dock) const {
  if (dock == nullptr) {
    return Qt::RightDockWidgetArea;
  }
  if (dock == topics_dock_ || dock == displays_dock_ || dock == channel_dock_ ||
      dock == problems_dock_ || dock == strata_floor_dock_) {
    return Qt::LeftDockWidgetArea;
  }
  return Qt::RightDockWidgetArea;
}

void VisualizationFrame::ensureSidebarDockAttached(PanelDockWidget* dock) {
  if (dock == nullptr || isMainPanel(dock)) {
    return;
  }
  Qt::DockWidgetArea area = dockWidgetArea(dock);
  if (area == Qt::NoDockWidgetArea) {
    area = defaultSidebarArea(dock);
  }
  configureSidebarDock(dock, area);
  if (dock->objectName() == QLatin1String("PropertyInspectorDock")) {
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  } else {
    dock->setAllowedAreas(area);
  }
  if (dockWidgetArea(dock) == Qt::NoDockWidgetArea) {
    addDockWidget(area, dock);
  }
  dock->setCollapsed(false);

  if (area == Qt::LeftDockWidgetArea &&
      toolbar_toggle_left_dock_action_ != nullptr &&
      !toolbar_toggle_left_dock_action_->isChecked()) {
    toolbar_toggle_left_dock_action_->setChecked(true);
  }
  if (area == Qt::RightDockWidgetArea &&
      toolbar_toggle_right_dock_action_ != nullptr &&
      !toolbar_toggle_right_dock_action_->isChecked()) {
    toolbar_toggle_right_dock_action_->setChecked(true);
  }
}

void VisualizationFrame::syncCenterLayout() {
  if (main_panel_host_ == nullptr) {
    return;
  }
  if (QWidget* central = centralWidget()) {
    central->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    central->setMinimumSize(0, 0);
    central->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
  }
  main_panel_host_->setMinimumSize(0, 0);
  main_panel_host_->updateGeometry();

  QTimer::singleShot(0, main_panel_host_, [host = main_panel_host_]() {
    if (host != nullptr) {
      host->syncHorizontalDockLayout();
    }
  });

  updateGeometry();
}

void VisualizationFrame::addSidebarDock(PanelDockWidget* dock,
                                        Qt::DockWidgetArea area) {
  if (dock == nullptr) {
    return;
  }
  configureSidebarDock(dock, area);
  if (dock->objectName() == QLatin1String("PropertyInspectorDock")) {
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  }
  addDockWidget(area, dock);
}

void VisualizationFrame::showPropertyInspector(bool visible) {
  if (property_inspector_dock_ == nullptr) {
    return;
  }
  if (visible) {
    if (active_log_panel_ != nullptr) {
      bindLogToPropertyInspector(active_log_panel_);
    } else if (active_teleop_panel_ != nullptr) {
      bindTeleopToPropertyInspector(active_teleop_panel_);
    } else if (active_image_panel_ != nullptr) {
      bindImageToPropertyInspector(active_image_panel_);
    } else if (active_plot_panel_ != nullptr) {
      bindPlotToPropertyInspector(active_plot_panel_);
    }
    property_inspector_dock_->show();
    property_inspector_dock_->raise();
  } else {
    property_inspector_dock_->hide();
  }
  if (active_plot_panel_ != nullptr) {
    active_plot_panel_->setSettingsButtonChecked(visible);
  }
  if (active_image_panel_ != nullptr) {
    active_image_panel_->setSettingsButtonChecked(visible);
  }
  if (active_teleop_panel_ != nullptr) {
    active_teleop_panel_->setSettingsButtonChecked(visible);
  }
  if (active_log_panel_ != nullptr) {
    active_log_panel_->setSettingsButtonChecked(visible);
  }
  manager_->setPlotSettingsVisible(visible);
}

void VisualizationFrame::bindPlotToPropertyInspector(plot::PlotPanel* panel) {
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
  if (inspector_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(inspector_log_panel_);
  }
  inspector_plot_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(panel->settingsWidgetForInspector(),
                                                title.isEmpty() ? tr("Plot") : title);
  if (property_inspector_dock_ != nullptr) {
    panel->setSettingsButtonChecked(property_inspector_dock_->isVisible());
  }
}

void VisualizationFrame::clearPropertyInspectorForPlot(plot::PlotPanel* panel) {
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

void VisualizationFrame::bindImageToPropertyInspector(image::ImagePanel* panel) {
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
  if (inspector_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(inspector_log_panel_);
  }
  inspector_image_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(panel->settingsWidgetForInspector(),
                                              title.isEmpty() ? tr("Image") : title);
  if (property_inspector_dock_ != nullptr) {
    panel->setSettingsButtonChecked(property_inspector_dock_->isVisible());
  }
}

void VisualizationFrame::clearPropertyInspectorForImage(image::ImagePanel* panel) {
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

void VisualizationFrame::bindTeleopToPropertyInspector(teleop::TeleopPanel* panel) {
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
                                              title.isEmpty() ? tr("Teleop") : title);
  if (property_inspector_dock_ != nullptr) {
    panel->setSettingsButtonChecked(property_inspector_dock_->isVisible());
  }
}

void VisualizationFrame::clearPropertyInspectorForTeleop(teleop::TeleopPanel* panel) {
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

void VisualizationFrame::bindLogToPropertyInspector(log_panel::LogPanel* panel) {
  if (panel == nullptr || property_inspector_panel_ == nullptr) {
    return;
  }
  if (inspector_log_panel_ != nullptr && inspector_log_panel_ != panel) {
    inspector_log_panel_->recallSettingsWidget();
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
  inspector_log_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(panel->settingsWidgetForInspector(),
                                              title.isEmpty() ? tr("Log") : title);
  if (property_inspector_dock_ != nullptr) {
    panel->setSettingsButtonChecked(property_inspector_dock_->isVisible());
  }
}

void VisualizationFrame::clearPropertyInspectorForLog(log_panel::LogPanel* panel) {
  if (panel == nullptr) {
    return;
  }
  panel->recallSettingsWidget();
  if (inspector_log_panel_ == panel) {
    inspector_log_panel_ = nullptr;
    if (property_inspector_panel_ != nullptr) {
      property_inspector_panel_->clearContent();
    }
  }
}

void VisualizationFrame::bindTableToPropertyInspector(table::TablePanel* panel) {
  if (panel == nullptr || property_inspector_panel_ == nullptr) {
    return;
  }
  if (inspector_table_panel_ != nullptr && inspector_table_panel_ != panel) {
    inspector_table_panel_->recallSettingsWidget();
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
  if (inspector_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(inspector_log_panel_);
  }
  inspector_table_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(
      panel->settingsWidgetForInspector(),
      title.isEmpty() ? tr("Table") : title);
  if (property_inspector_dock_ != nullptr) {
    panel->setSettingsButtonChecked(property_inspector_dock_->isVisible());
  }
}

void VisualizationFrame::clearPropertyInspectorForTable(table::TablePanel* panel) {
  if (panel == nullptr) {
    return;
  }
  panel->recallSettingsWidget();
  if (inspector_table_panel_ == panel) {
    inspector_table_panel_ = nullptr;
    if (property_inspector_panel_ != nullptr) {
      property_inspector_panel_->clearContent();
    }
  }
}

void VisualizationFrame::bindPublishToPropertyInspector(
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
  if (inspector_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(inspector_log_panel_);
  }
  if (inspector_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(inspector_table_panel_);
  }
  if (inspector_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(inspector_gauge_panel_);
  }
  inspector_publish_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(
      panel->settingsWidgetForInspector(),
      title.isEmpty() ? tr("Publish") : title);
  if (property_inspector_dock_ != nullptr) {
    panel->setSettingsButtonChecked(property_inspector_dock_->isVisible());
  }
}

void VisualizationFrame::clearPropertyInspectorForPublish(
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

void VisualizationFrame::bindGaugeToPropertyInspector(gauge::GaugePanel* panel) {
  if (panel == nullptr || property_inspector_panel_ == nullptr) {
    return;
  }
  if (inspector_gauge_panel_ != nullptr && inspector_gauge_panel_ != panel) {
    inspector_gauge_panel_->recallSettingsWidget();
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
  if (inspector_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(inspector_log_panel_);
  }
  if (inspector_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(inspector_table_panel_);
  }
  if (inspector_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(inspector_publish_panel_);
  }
  if (inspector_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(inspector_map_panel_);
  }
  if (inspector_indicator_panel_ != nullptr) {
    clearPropertyInspectorForIndicator(inspector_indicator_panel_);
  }
  inspector_gauge_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(
      panel->settingsWidgetForInspector(),
      title.isEmpty() ? tr("Gauge") : title);
  if (property_inspector_dock_ != nullptr) {
    panel->setSettingsButtonChecked(property_inspector_dock_->isVisible());
  }
}

void VisualizationFrame::clearPropertyInspectorForGauge(gauge::GaugePanel* panel) {
  if (panel == nullptr) {
    return;
  }
  panel->recallSettingsWidget();
  if (inspector_gauge_panel_ == panel) {
    inspector_gauge_panel_ = nullptr;
    if (property_inspector_panel_ != nullptr) {
      property_inspector_panel_->clearContent();
    }
  }
}

void VisualizationFrame::bindMapToPropertyInspector(map::MapPanel* panel) {
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
  if (inspector_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(inspector_log_panel_);
  }
  if (inspector_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(inspector_table_panel_);
  }
  if (inspector_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(inspector_publish_panel_);
  }
  if (inspector_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(inspector_gauge_panel_);
  }
  if (inspector_indicator_panel_ != nullptr) {
    clearPropertyInspectorForIndicator(inspector_indicator_panel_);
  }
  inspector_map_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(
      panel->settingsWidgetForInspector(),
      title.isEmpty() ? tr("Map") : title);
  if (property_inspector_dock_ != nullptr) {
    panel->setSettingsButtonChecked(property_inspector_dock_->isVisible());
  }
}

void VisualizationFrame::clearPropertyInspectorForMap(map::MapPanel* panel) {
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

void VisualizationFrame::bindIndicatorToPropertyInspector(
    indicator::IndicatorPanel* panel) {
  if (panel == nullptr || property_inspector_panel_ == nullptr) {
    return;
  }
  if (inspector_indicator_panel_ != nullptr && inspector_indicator_panel_ != panel) {
    inspector_indicator_panel_->recallSettingsWidget();
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
  if (inspector_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(inspector_log_panel_);
  }
  if (inspector_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(inspector_table_panel_);
  }
  if (inspector_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(inspector_publish_panel_);
  }
  if (inspector_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(inspector_gauge_panel_);
  }
  if (inspector_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(inspector_map_panel_);
  }
  inspector_indicator_panel_ = panel;
  const QString title = panel->config().title.trimmed();
  property_inspector_panel_->setContentWidget(
      panel->settingsWidgetForInspector(),
      title.isEmpty() ? tr("Indicator") : title);
  if (property_inspector_dock_ != nullptr) {
    panel->setSettingsButtonChecked(property_inspector_dock_->isVisible());
  }
}

void VisualizationFrame::clearPropertyInspectorForIndicator(
    indicator::IndicatorPanel* panel) {
  if (panel == nullptr) {
    return;
  }
  panel->recallSettingsWidget();
  if (inspector_indicator_panel_ == panel) {
    inspector_indicator_panel_ = nullptr;
    if (property_inspector_panel_ != nullptr) {
      property_inspector_panel_->clearContent();
    }
  }
}

void VisualizationFrame::applyMainPanelDefaultLayout() {
  restoreExpandedMainPanel();
  if (main_panel_host_ == nullptr || plot_dock_ == nullptr ||
      viewport_dock_ == nullptr) {
    return;
  }
  main_panel_host_->removeDockWidget(plot_dock_);
  main_panel_host_->removeDockWidget(viewport_dock_);
  plot_dock_->show();
  viewport_dock_->show();
  addMainPanelDock(plot_dock_, Qt::LeftDockWidgetArea);
  addMainPanelDock(viewport_dock_, Qt::RightDockWidgetArea);
  main_panel_host_->splitDockWidget(plot_dock_, viewport_dock_, Qt::Horizontal);
  main_panel_host_->resizeDocks({plot_dock_, viewport_dock_}, {420, 620},
                                Qt::Horizontal);
  last_active_dock_ = viewport_dock_;
}

void VisualizationFrame::configureFlexibleDock(PanelDockWidget* dock) {
  if (dock == nullptr || dock == time_dock_) {
    return;
  }
  dock->setAllowedAreas(Qt::AllDockWidgetAreas);
  dock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                    QDockWidget::DockWidgetFloatable);
}

void VisualizationFrame::setupUi() {
  updateWindowTitle();
  resize(1280, 800);
  setupCentralContainer();

  viewport_dock_ = new PanelDockWidget(tr("3D"), this);
  viewport_dock_->setObjectName(QStringLiteral("ViewportDock"));
  viewport_dock_->setProperty("panelTypeId", QStringLiteral("ViewportDock"));
  viewport_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Panel3D")));
  viewport_host_ = new QWidget(viewport_dock_);
  viewport_host_->setMinimumSize(240, 180);
  viewport_layout_ = new QVBoxLayout(viewport_host_);
  viewport_layout_->setContentsMargins(0, 0, 0, 0);
  viewport_layout_->setSpacing(0);
  viewport_dock_->setContentWidget(viewport_host_);
  configureMainPanelDock(viewport_dock_);
  addMainPanelDock(viewport_dock_, Qt::RightDockWidgetArea);
  installViewportTitleBarTools();

  channel_dock_ = new PanelDockWidget(tr("Raw Messages"), this);
  channel_dock_->setObjectName(QStringLiteral("ChannelsDock"));
  channel_dock_->setPanelIcon(
      IconLoader::panelIcon(QStringLiteral("PanelRawMessages")));
  raw_messages_panel_ = new RawMessagesPanel(manager_.get(), channel_dock_);
  channel_dock_->setContentWidget(raw_messages_panel_);
  addSidebarDock(channel_dock_, Qt::LeftDockWidgetArea);

  topics_dock_ = new PanelDockWidget(tr("Topics"), this);
  topics_dock_->setObjectName(QStringLiteral("TopicsDock"));
  topics_dock_->setPanelIcon(
      IconLoader::panelIcon(QStringLiteral("PanelChannelGraph")));
  topics_panel_ = new TopicsPanel(manager_.get(), topics_dock_);
  topics_dock_->setContentWidget(topics_panel_);
  addSidebarDock(topics_dock_, Qt::LeftDockWidgetArea);
  tabifyDockWidget(channel_dock_, topics_dock_);

  problems_dock_ = new PanelDockWidget(tr("Problems"), this);
  problems_dock_->setObjectName(QStringLiteral("ProblemsDock"));
  problems_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelLog")));
  problems_panel_ = new ProblemsPanel(problems_dock_);
  problems_dock_->setContentWidget(problems_panel_);
  addSidebarDock(problems_dock_, Qt::LeftDockWidgetArea);

  log_dock_ = createLogPanelDock(QStringLiteral("LogDock"));
  log_panel_ = qobject_cast<log_panel::LogPanel*>(log_dock_->widget());
  addMainPanelDock(log_dock_, Qt::LeftDockWidgetArea);

  displays_dock_ = new PanelDockWidget(tr("Displays"), this);
  displays_dock_->setObjectName(QStringLiteral("DisplaysDock"));
  displays_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Displays")));
  displays_panel_ = new DisplaysPanel(manager_, displays_dock_);
  displays_dock_->setContentWidget(displays_panel_);
  addSidebarDock(displays_dock_, Qt::LeftDockWidgetArea);

  strata_floor_dock_ = new PanelDockWidget(tr("Strata Floors"), this);
  strata_floor_dock_->setObjectName(QStringLiteral("StrataFloorDock"));
  strata_floor_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Displays")));
  strata_floor_panel_ = new StrataFloorPanel(manager_.get(), strata_floor_dock_);
  strata_floor_dock_->setContentWidget(strata_floor_panel_);
  addSidebarDock(strata_floor_dock_, Qt::LeftDockWidgetArea);

  playback_dock_ = new PanelDockWidget(tr("Playback"), this);
  playback_dock_->setObjectName(QStringLiteral("PlaybackDock"));
  playback_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Playback")));
  playback_panel_ = new PlaybackPanel(&manager_->playback(), playback_dock_);
  playback_dock_->setContentWidget(playback_panel_);
  addMainPanelDock(playback_dock_, Qt::LeftDockWidgetArea);
  playback_dock_->hide();

  views_dock_ = new PanelDockWidget(tr("Views"), this);
  views_dock_->setObjectName(QStringLiteral("ViewsDock"));
  views_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Views")));
  views_panel_ = new ViewsPanel(nullptr, manager_.get(), views_dock_);
  views_dock_->setContentWidget(views_panel_);
  addSidebarDock(views_dock_, Qt::RightDockWidgetArea);

  tool_props_dock_ = new PanelDockWidget(tr("Tool Properties"), this);
  tool_props_dock_->setObjectName(QStringLiteral("ToolPropertiesDock"));
  tool_props_dock_->setPanelIcon(
      IconLoader::panelIcon(QStringLiteral("ToolProperties")));
  tool_properties_panel_ =
      new ToolPropertiesPanel(manager_, tool_props_dock_);
  tool_props_dock_->setContentWidget(tool_properties_panel_);
  addSidebarDock(tool_props_dock_, Qt::RightDockWidgetArea);

  selection_dock_ = new PanelDockWidget(tr("Selection"), this);
  selection_dock_->setObjectName(QStringLiteral("SelectionDock"));
  selection_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Selection")));
  selection_panel_ = new SelectionPanel(selection_dock_);
  selection_dock_->setContentWidget(selection_panel_);
  addSidebarDock(selection_dock_, Qt::RightDockWidgetArea);

  variables_dock_ = new PanelDockWidget(tr("Variables"), this);
  variables_dock_->setObjectName(QStringLiteral("VariablesDock"));
  variables_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelParameters")));
  variables_panel_ = new VariablesPanel(variables_dock_);
  variables_dock_->setContentWidget(variables_panel_);
  addSidebarDock(variables_dock_, Qt::RightDockWidgetArea);
  variables_dock_->hide();

  property_inspector_dock_ = new PanelDockWidget(tr("Panel"), this);
  property_inspector_dock_->setObjectName(QStringLiteral("PropertyInspectorDock"));
  property_inspector_dock_->setProperty("panelTypeId",
                                        QStringLiteral("PropertyInspectorDock"));
  property_inspector_dock_->setPanelIcon(
      IconLoader::panelIcon(QStringLiteral("PanelPlot")));
  property_inspector_panel_ = new PropertyInspectorPanel(property_inspector_dock_);
  property_inspector_dock_->setContentWidget(property_inspector_panel_);
  addSidebarDock(property_inspector_dock_, Qt::RightDockWidgetArea);

  tf_dock_ = createTfTreePanelDock(QStringLiteral("TfTreeDock"));
  tf_tree_panel_ = qobject_cast<TfTreePanel*>(tf_dock_->widget());
  addMainPanelDock(tf_dock_, Qt::LeftDockWidgetArea);
  tf_dock_->hide();

  transformation_dock_ = new PanelDockWidget(tr("Transformation"), this);
  transformation_dock_->setObjectName(QStringLiteral("TransformationDock"));
  transformation_dock_->setPanelIcon(
      IconLoader::panelIcon(QStringLiteral("Transformation")));
  transformation_panel_ = new TransformationPanel(
      &manager_->transformationManager(), transformation_dock_);
  transformation_dock_->setContentWidget(transformation_panel_);
  addSidebarDock(transformation_dock_, Qt::RightDockWidgetArea);
  connect(transformation_panel_, &TransformationPanel::transformerChanged,
          this, [this]() { requestViewportUpdate(); });

  image_dock_ = createImagePanelDock(QStringLiteral("ImageDock"));
  image_panel_ = qobject_cast<image::ImagePanel*>(image_dock_->widget());
  addMainPanelDock(image_dock_, Qt::LeftDockWidgetArea);
  image_dock_->hide();

  plot_dock_ = createPlotPanelDock(QStringLiteral("PlotDock"));
  plot_panel_ = qobject_cast<plot::PlotPanel*>(plot_dock_->widget());
  installPlotFocusTracking();
  addMainPanelDock(plot_dock_, Qt::LeftDockWidgetArea);
  setActivePlotPanel(plot_panel_);

  help_dock_ = new PanelDockWidget(tr("Help"), this);
  help_dock_->setObjectName(QStringLiteral("HelpDock"));
  help_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Help")));
  help_panel_ = new HelpPanel(help_dock_);
  help_dock_->setContentWidget(help_panel_);
  addSidebarDock(help_dock_, Qt::RightDockWidgetArea);
  help_dock_->hide();

#ifdef AUTOVIZ_USE_QML_DRONE
  if (qEnvironmentVariableIntValue("AUTOVIZ_DISABLE_QML") == 0) {
    drone_dock_ = new PanelDockWidget(tr("Vehicle 3D"), this);
    drone_dock_->setObjectName(QStringLiteral("Drone3DDock"));
    drone_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Views")));
    vehicle_panel_ = new Vehicle3DPanel(drone_dock_);
    drone_dock_->setContentWidget(vehicle_panel_);
    addMainPanelDock(drone_dock_, Qt::LeftDockWidgetArea);
    drone_dock_->hide();
  }
#endif

  time_dock_ = new PanelDockWidget(tr("Time"), this);
  time_dock_->setObjectName(QStringLiteral("TimeDock"));
  time_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Time")));
  time_dock_->setAllowedAreas(Qt::BottomDockWidgetArea);
  time_dock_->setFeatures(QDockWidget::DockWidgetClosable |
                          QDockWidget::DockWidgetMovable);
  time_panel_ = new TimePanel(manager_.get(), time_dock_);
  time_dock_->setContentWidget(time_panel_);
  addDockWidget(Qt::BottomDockWidgetArea, time_dock_);

  for (PanelDockWidget* dock : orderedDockWidgets()) {
    connect(dock, &QDockWidget::visibilityChanged, this,
            &VisualizationFrame::onDockPanelVisibilityChange);
    connect(this, &VisualizationFrame::fullScreenChange, dock,
            &PanelDockWidget::overrideVisibility);
  }

  // Bottom Time bar spans full window width; sidebars stack above it.
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::BottomDockWidgetArea);

  setupStatusBar();

  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr || dock == time_dock_ || dock == viewport_dock_) {
      continue;
    }
    if (panelTypeId(dock) == QLatin1String("PlotDock") ||
        panelTypeId(dock) == QLatin1String("ImageDock") ||
        panelTypeId(dock) == QLatin1String("TeleopDock") ||
        panelTypeId(dock) == QLatin1String("LogDock") ||
        panelTypeId(dock) == QLatin1String("TfTreeDock") ||
        panelTypeId(dock) == QLatin1String("TableDock") ||
        panelTypeId(dock) == QLatin1String("PublishDock") ||
        panelTypeId(dock) == QLatin1String("GaugeDock") ||
        panelTypeId(dock) == QLatin1String("MapDock") ||
        panelTypeId(dock) == QLatin1String("IndicatorDock")) {
      continue;
    }
    installStandardPanelTitleTools(dock);
  }
}

void VisualizationFrame::setupMenu() {
  auto* file_menu = menuBar()->addMenu(tr("&File"));
  auto* open_config_action =
      file_menu->addAction(tr("&Open Config..."), this,
                           &VisualizationFrame::onOpenConfig);
  open_config_action->setShortcut(QKeySequence::Open);
  auto* save_config_action =
      file_menu->addAction(tr("&Save Config"), this,
                           &VisualizationFrame::onSaveConfig);
  save_config_action->setShortcut(QKeySequence::Save);
  auto* save_config_as_action =
      file_menu->addAction(tr("Save Config &As..."), this,
                           &VisualizationFrame::onSaveConfigAs);
  save_config_as_action->setShortcut(QKeySequence::SaveAs);
  recent_configs_menu_ = file_menu->addMenu(tr("&Recent Configs"));
  file_menu->addAction(tr("Save &Image..."), this, &VisualizationFrame::onScreenshot);
  file_menu->addSeparator();
  auto* quit_action =
      file_menu->addAction(tr("&Quit"), qApp, &QApplication::quit);
  quit_action->setShortcut(QKeySequence::Quit);
  addAction(quit_action);

  auto* layout_menu = menuBar()->addMenu(tr("&Layout"));
  layout_menu->addAction(tr("&Open layout..."), this,
                         &VisualizationFrame::onOpenConfig);
  layout_menu->addAction(tr("&Save layout"), this,
                         &VisualizationFrame::onSaveConfig);
  layout_menu->addAction(tr("Save layout &as..."), this,
                         &VisualizationFrame::onSaveConfigAs);
  layout_menu->addSeparator();
  layout_menu->addAction(tr("&Reset to default layout"), this,
                         &VisualizationFrame::onResetDefaultLayout);

  panels_menu_ = menuBar()->addMenu(tr("&Panels"));
  panels_menu_->addAction(tr("Add &New Panel"), this,
                          &VisualizationFrame::onAddPanel);
  delete_panel_menu_ = panels_menu_->addMenu(tr("&Delete Panel"));
  delete_panel_menu_->setEnabled(false);
  fullscreen_action_ =
      panels_menu_->addAction(tr("&Fullscreen"), this,
                             &VisualizationFrame::onToggleFullscreen);
  fullscreen_action_->setShortcut(QKeySequence(Qt::Key_F11));
  fullscreen_action_->setCheckable(true);
  addAction(fullscreen_action_);
  connect(this, &VisualizationFrame::fullScreenChange, fullscreen_action_,
          &QAction::setChecked);
  panels_menu_->addSeparator();

  auto* view_menu = menuBar()->addMenu(tr("&View"));
  auto* view_group = new QActionGroup(this);
  view_group->setExclusive(true);

  view_orbit_action_ = view_menu->addAction(tr("&Orbit"));
  view_orbit_action_->setCheckable(true);
  view_orbit_action_->setChecked(true);
  view_group->addAction(view_orbit_action_);
  connect(view_orbit_action_, &QAction::triggered, this,
          &VisualizationFrame::onViewOrbit);

  view_xy_orbit_action_ = view_menu->addAction(tr("XY &Orbit"));
  view_xy_orbit_action_->setCheckable(true);
  view_group->addAction(view_xy_orbit_action_);
  connect(view_xy_orbit_action_, &QAction::triggered, this,
          &VisualizationFrame::onViewXyOrbit);

  view_topdown_action_ = view_menu->addAction(tr("&Top Down"));
  view_topdown_action_->setCheckable(true);
  view_group->addAction(view_topdown_action_);
  connect(view_topdown_action_, &QAction::triggered, this,
          &VisualizationFrame::onViewTopDown);

  view_topdown_ortho_action_ = view_menu->addAction(tr("Top Down &Ortho"));
  view_topdown_ortho_action_->setCheckable(true);
  view_group->addAction(view_topdown_ortho_action_);
  connect(view_topdown_ortho_action_, &QAction::triggered, this,
          &VisualizationFrame::onViewTopDownOrtho);

  view_third_person_action_ = view_menu->addAction(tr("&Third Person Follow"));
  view_third_person_action_->setCheckable(true);
  view_group->addAction(view_third_person_action_);
  connect(view_third_person_action_, &QAction::triggered, this,
          &VisualizationFrame::onViewThirdPersonFollow);

  view_fps_action_ = view_menu->addAction(tr("&FPS (WASD/QE)"));
  view_fps_action_->setCheckable(true);
  view_group->addAction(view_fps_action_);
  connect(view_fps_action_, &QAction::triggered, this,
          &VisualizationFrame::onViewFps);

  view_menu->addSeparator();
  auto* backend_group = new QActionGroup(this);
  backend_group->setExclusive(true);
  backend_opengl_action_ = view_menu->addAction(tr("Backend: &OpenGL"));
  backend_opengl_action_->setCheckable(true);
  backend_opengl_action_->setChecked(true);
  backend_group->addAction(backend_opengl_action_);
  connect(backend_opengl_action_, &QAction::triggered, this,
          &VisualizationFrame::onBackendOpenGl);

  backend_ogre_action_ = view_menu->addAction(tr("Backend: &Ogre"));
  backend_ogre_action_->setCheckable(true);
  backend_group->addAction(backend_ogre_action_);
#ifndef AUTOVIZ_USE_OGRE
  backend_ogre_action_->setEnabled(false);
  backend_ogre_action_->setToolTip(
      tr("Rebuild with -DAUTOVIZ_USE_OGRE=ON to enable"));
#endif
  connect(backend_ogre_action_, &QAction::triggered, this,
          &VisualizationFrame::onBackendOgre);

  syncViewControllerMenu(
      QString::fromStdString(manager_->viewControllerName()));
  syncRenderBackendMenu(QString::fromStdString(manager_->renderBackendName()));

  auto* help_menu = menuBar()->addMenu(tr("&Help"));
  help_menu->addAction(tr("Show &Help panel"), this,
                       &VisualizationFrame::showHelpPanel);
  help_menu->addSeparator();
  help_menu->addAction(tr("&About"), this, &VisualizationFrame::onHelpAbout);

  QSettings settings;
  recent_configs_ =
      settings.value(QStringLiteral("recent_configs")).toStringList();
  updateRecentConfigMenu();
}

void VisualizationFrame::setupToolbar() {
  tool_bar_ = addToolBar(tr("Tools"));
  tool_bar_->setObjectName(QStringLiteral("Tools"));
  tool_bar_->setMovable(false);
  tool_bar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  tool_bar_->setIconSize(QSize(28, 28));
  tool_action_group_ = new QActionGroup(this);
  tool_action_group_->setExclusive(true);

  add_tool_action_ = new QAction(
      IconLoader::load(QStringLiteral(":/autoviz/icons/plus.svg")),
      QString(), this);
  add_tool_action_->setToolTip(tr("Add a tool to the toolbar"));
  connect(add_tool_action_, &QAction::triggered, this,
          &VisualizationFrame::onAddToolTriggered);

  remove_tool_menu_ = new QMenu(this);
  connect(remove_tool_menu_, &QMenu::triggered, this,
          &VisualizationFrame::onRemoveToolTriggered);
  auto* remove_tool_button = new QToolButton(this);
  remove_tool_button->setMenu(remove_tool_menu_);
  remove_tool_button->setPopupMode(QToolButton::InstantPopup);
  remove_tool_button->setToolTip(tr("Remove a tool from the toolbar"));
  remove_tool_button->setIcon(
      IconLoader::load(QStringLiteral(":/autoviz/icons/minus.svg")));
  remove_tool_button->setAutoRaise(true);

  rebuildToolbar();
  tool_bar_->addAction(add_tool_action_);
  tool_bar_->addWidget(remove_tool_button);

  if (panels_menu_ != nullptr) {
    panels_menu_->addAction(tool_bar_->toggleViewAction());
  }
  registerPanelToggleActions();
  setupToolbarLayoutControls();

  auto* app_menu = new QMenu(tr("&App"), this);
  if (toolbar_toggle_left_dock_action_ != nullptr) {
    app_menu->addAction(toolbar_toggle_left_dock_action_);
  }
  if (toolbar_toggle_right_dock_action_ != nullptr) {
    app_menu->addAction(toolbar_toggle_right_dock_action_);
  }
  if (!menuBar()->actions().isEmpty()) {
    menuBar()->insertMenu(menuBar()->actions().constFirst(), app_menu);
  } else {
    menuBar()->addMenu(app_menu);
  }
}

void VisualizationFrame::setupToolbarLayoutControls() {
  if (tool_bar_ == nullptr) {
    return;
  }

  toolbar_spacer_ = new QWidget(tool_bar_);
  toolbar_spacer_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  tool_bar_->addWidget(toolbar_spacer_);

  toolbar_add_panel_button_ = new QToolButton(tool_bar_);
  toolbar_add_panel_button_->setIcon(
      IconLoader::load(QStringLiteral(":/autoviz/icons/add_panel.svg")));
  toolbar_add_panel_button_->setAutoRaise(true);
  toolbar_add_panel_button_->setPopupMode(QToolButton::InstantPopup);
  toolbar_add_panel_button_->setToolTip(tr("Add"));
  add_panel_menu_ = new QMenu(toolbar_add_panel_button_);
  toolbar_add_panel_button_->setMenu(add_panel_menu_);
  connect(add_panel_menu_, &QMenu::aboutToShow, this,
          &VisualizationFrame::populateAddPanelMenu);
  connect(add_panel_menu_, &QMenu::triggered, this,
          &VisualizationFrame::onAddPanelMenuTriggered);
  tool_bar_->addWidget(toolbar_add_panel_button_);

  toolbar_toggle_left_dock_action_ = tool_bar_->addAction(
      IconLoader::load(QStringLiteral(":/autoviz/icons/sidebar_left.svg")),
      QString());
  toolbar_toggle_left_dock_action_->setCheckable(true);
  toolbar_toggle_left_dock_action_->setChecked(true);
  toolbar_toggle_left_dock_action_->setToolTip(tr("Hide Left"));
  connect(toolbar_toggle_left_dock_action_, &QAction::toggled, this,
          [this](bool show_left) {
            hideLeftDock(!show_left);
            toolbar_toggle_left_dock_action_->setToolTip(
                show_left ? tr("Hide Left") : tr("Show Left"));
            markConfigModified();
          });

  toolbar_toggle_right_dock_action_ = tool_bar_->addAction(
      IconLoader::load(QStringLiteral(":/autoviz/icons/sidebar_right.svg")),
      QString());
  toolbar_toggle_right_dock_action_->setCheckable(true);
  toolbar_toggle_right_dock_action_->setChecked(true);
  toolbar_toggle_right_dock_action_->setToolTip(tr("Hide Right"));
  connect(toolbar_toggle_right_dock_action_, &QAction::toggled, this,
          [this](bool show_right) {
            hideRightDock(!show_right);
            toolbar_toggle_right_dock_action_->setToolTip(
                show_right ? tr("Hide Right") : tr("Show Right"));
            markConfigModified();
          });

  syncToolbarLayoutControls();
}

void VisualizationFrame::syncToolbarLayoutControls() {
  if (toolbar_toggle_left_dock_action_ != nullptr) {
    toolbar_toggle_left_dock_action_->blockSignals(true);
    const bool show_left = !manager_->hideLeftDock();
    toolbar_toggle_left_dock_action_->setChecked(show_left);
    toolbar_toggle_left_dock_action_->setToolTip(
        show_left ? tr("Hide Left") : tr("Show Left"));
    toolbar_toggle_left_dock_action_->blockSignals(false);
  }
  if (toolbar_toggle_right_dock_action_ != nullptr) {
    toolbar_toggle_right_dock_action_->blockSignals(true);
    const bool show_right = !manager_->hideRightDock();
    toolbar_toggle_right_dock_action_->setChecked(show_right);
    toolbar_toggle_right_dock_action_->setToolTip(
        show_right ? tr("Hide Right") : tr("Show Right"));
    toolbar_toggle_right_dock_action_->blockSignals(false);
  }
}

void VisualizationFrame::rebuildToolbar() {
  if (tool_bar_ == nullptr || tool_action_group_ == nullptr) {
    return;
  }

  if (manager_->toolbarTools().empty()) {
    manager_->setToolbarTools({});
  }

  for (QAction* action : toolbar_tool_actions_) {
    tool_action_group_->removeAction(action);
    tool_bar_->removeAction(action);
    delete action;
  }
  toolbar_tool_actions_.clear();
  if (remove_tool_menu_ != nullptr) {
    remove_tool_menu_->clear();
  }

  int shortcut_index = 1;
  const common::ToolManager& tools = manager_->tools();
  for (const std::string& tool_id : manager_->toolbarTools()) {
    const QString tool_q = QString::fromStdString(tool_id);
    const QString label = tools.toolLabel(tool_id);
    auto* action = new QAction(IconLoader::toolIcon(tool_q), label, this);
    action->setObjectName(tool_q);
    action->setCheckable(true);
    action->setData(tool_q);
    if (shortcut_index <= 9) {
      action->setShortcut(QKeySequence(QString::number(shortcut_index)));
      ++shortcut_index;
    }
    const char letter_shortcut = tools.shortcutKeyForTool(tool_id);
    if (letter_shortcut != '\0') {
      action->setToolTip(
          tr("%1 (快捷键 %2)").arg(label).arg(QChar::fromLatin1(letter_shortcut)));
    } else {
      action->setToolTip(label);
    }
    if (tool_id == tools.activeToolId()) {
      action->setChecked(true);
    }
    connect(action, &QAction::triggered, this, [this, tool_id]() {
      applyActiveTool(tool_id);
    });
    tool_action_group_->addAction(action);
    if (add_tool_action_ != nullptr &&
        tool_bar_->actions().contains(add_tool_action_)) {
      tool_bar_->insertAction(add_tool_action_, action);
    } else {
      tool_bar_->addAction(action);
    }
    toolbar_tool_actions_.push_back(action);
    if (remove_tool_menu_ != nullptr) {
      QAction* remove_action = remove_tool_menu_->addAction(label);
      remove_action->setData(tool_q);
    }
  }

  if (add_tool_action_ != nullptr) {
    add_tool_action_->setEnabled(!tools.toolsNotInToolbar().empty());
  }
}

void VisualizationFrame::applyActiveTool(const std::string& tool_id) {
  if (!manager_->tools().setActiveTool(tool_id)) {
    return;
  }
  syncActiveToolUi();
}

void VisualizationFrame::syncActiveToolUi() {
  syncToolbarToActiveTool();
  syncViewportTitleBarTools();
  syncToolContext();
  if (tool_properties_panel_ != nullptr) {
    tool_properties_panel_->refresh();
  }
  updateStatusBar();
  updateViewportCursor();
  if (viewport_widget_ != nullptr) {
    viewport_widget_->setFocus(Qt::MouseFocusReason);
  }
  requestViewportUpdate();
}

void VisualizationFrame::syncToolbarToActiveTool() {
  const QString active =
      QString::fromStdString(manager_->tools().activeToolId());
  for (QAction* action : toolbar_tool_actions_) {
    if (action == nullptr) {
      continue;
    }
    action->setChecked(action->data().toString() == active);
  }
}

void VisualizationFrame::updateViewportCursor() {
  if (common::Tool* tool = manager_->tools().activeTool()) {
    tool->setCursor(IconLoader::toolCursor(
        QString::fromStdString(manager_->tools().activeToolId())));
  }
  const QCursor cursor =
      manager_->tools().activeTool() != nullptr
          ? manager_->tools().activeTool()->cursor()
          : IconLoader::defaultCursor();
  if (viewport_host_ != nullptr) {
    viewport_host_->setCursor(cursor);
  }
  if (gl_viewport_ != nullptr) {
    gl_viewport_->setToolCursor(cursor);
  }
  if (ogre_viewport_ != nullptr) {
    ogre_viewport_->setToolCursor(cursor);
  }
}

void VisualizationFrame::onAddToolTriggered() {
  QMenu menu(this);
  for (const std::string& id : manager_->tools().toolsNotInToolbar()) {
    const QString tool_q = QString::fromStdString(id);
    QAction* action = menu.addAction(manager_->tools().toolLabel(id));
    action->setData(tool_q);
    action->setIcon(IconLoader::toolIcon(tool_q));
  }
  if (menu.isEmpty()) {
    return;
  }
  QAction* picked = menu.exec(QCursor::pos());
  if (picked == nullptr) {
    return;
  }
  if (manager_->tools().addToolToToolbar(
          picked->data().toString().toStdString())) {
    rebuildToolbar();
    markConfigModified();
  }
}

void VisualizationFrame::onRemoveToolTriggered(QAction* action) {
  if (action == nullptr) {
    return;
  }
  if (manager_->tools().removeToolFromToolbar(
          action->data().toString().toStdString())) {
    rebuildToolbar();
    if (tool_properties_panel_ != nullptr) {
      tool_properties_panel_->refresh();
    }
    updateStatusBar();
    markConfigModified();
  }
}

void VisualizationFrame::registerPanelToggleActions() {
  if (panels_menu_ == nullptr) {
    return;
  }
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    panels_menu_->addAction(dock->toggleViewAction());
  }
}

void VisualizationFrame::onToolTriggered(QAction* action) {
  if (action == nullptr) {
    return;
  }
  applyActiveTool(action->data().toString().toStdString());
  markConfigModified();
}

void VisualizationFrame::applyViewController(const QString& name) {
  rendering::ViewController* controller = activeViewController();
  if (controller == nullptr) {
    return;
  }
  controller->setTypeByName(name);
  if (name == QLatin1String("FPS") && viewport_widget_ != nullptr) {
    viewport_widget_->setFocus();
  }
  requestViewportUpdate();
}

void VisualizationFrame::onScreenshot() {
  if (viewport_widget_ == nullptr) {
    return;
  }
  const QPixmap pixmap = viewport_widget_->grab();
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Save Image"),
      QStringLiteral("autoviz_%1.png")
          .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_hhmmss"))),
      tr("PNG Image (*.png)"));
  if (!path.isEmpty()) {
    pixmap.save(path);
  }
}

void VisualizationFrame::setFullScreen(bool full_screen) {
  const auto state = windowState();
  if (full_screen == state.testFlag(Qt::WindowFullScreen)) {
    return;
  }
  if (fullscreen_action_ != nullptr) {
    fullscreen_action_->setChecked(full_screen);
  }
  if (full_screen) {
    toolbar_visible_ = tool_bar_ == nullptr || tool_bar_->isVisible();
  }
  emit fullScreenChange(full_screen);
  menuBar()->setVisible(!full_screen);
  if (tool_bar_ != nullptr) {
    tool_bar_->setVisible(!full_screen && toolbar_visible_);
  }
  statusBar()->setVisible(!full_screen);
  if (full_screen) {
    setWindowState(state | Qt::WindowFullScreen);
  } else {
    setWindowState(state & ~Qt::WindowFullScreen);
  }
  show();
}

void VisualizationFrame::applyRenderBackend(const QString& name) {
  rendering::GpuCapabilities::instance().ensureProbed();
  QString effective = name;
  if (effective == QLatin1String("Ogre")) {
#ifndef AUTOVIZ_USE_OGRE
    effective = QStringLiteral("OpenGL");
#else
    if (!rendering::GpuCapabilities::instance().hasHardwareGpu()) {
      effective = QStringLiteral("OpenGL");
      manager_->setRenderBackendName("OpenGL");
      if (status_label_ != nullptr) {
        status_label_->setText(
            tr("No hardware GPU detected; using OpenGL backend."));
      }
    }
#endif
  }
  createViewport(effective);
  if (gl_viewport_ != nullptr) {
    gl_viewport_->setGridVisible(manager_->showGrid());
    gl_viewport_->setToolManager(&manager_->tools());
  }
  if (ogre_viewport_ != nullptr) {
    ogre_viewport_->setGridVisible(manager_->showGrid());
    ogre_viewport_->setToolManager(&manager_->tools());
  }
  applyViewController(QString::fromStdString(manager_->viewControllerName()));
  syncToolContext();
  updateViewportCursor();
  requestViewportUpdate();
}

void VisualizationFrame::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape && isFullScreen()) {
    setFullScreen(false);
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_R && !event->modifiers().testFlag(Qt::ControlModifier) &&
      !event->modifiers().testFlag(Qt::AltModifier)) {
    onReset();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Z && views_panel_ != nullptr &&
      !event->modifiers().testFlag(Qt::ControlModifier) &&
      !event->modifiers().testFlag(Qt::AltModifier)) {
    views_panel_->zeroView();
    requestViewportUpdate();
    event->accept();
    return;
  }
  if (!event->modifiers()) {
    if (manager_->tools().handleShortcutKey(event->key())) {
      syncActiveToolUi();
      event->accept();
      return;
    }
  }
  QMainWindow::keyPressEvent(event);
}

bool VisualizationFrame::loadConfig(const QString& path) {
  suppress_config_modified_ = true;
  if (!manager_->loadSession(path.toStdString())) {
    suppress_config_modified_ = false;
    return false;
  }
  config_path_ = path;
  displays_panel_->refresh();
  syncViewsFromManager();
  applyRenderBackend(QString::fromStdString(manager_->renderBackendName()));
  applyCurrentView();
  rebuildToolbar();
  restoreWindowLayout();
  restorePlotPanelConfigs();
  restoreImagePanelConfigs();
  applyPlotSettingsVisibilityFromSession();
  applyActiveTool(manager_->tools().activeToolId());
  ensureTimeDockAtBottom();
  syncDeletePanelMenu();
  updateStatusBar();
  markRecentConfig(path);
  suppress_config_modified_ = false;
  clearConfigModified();
  return true;
}

bool VisualizationFrame::saveConfig(const QString& path) {
  syncViewsToManager();
  captureCurrentView();
  captureWindowLayout();
  if (!manager_->saveSession(path.toStdString())) {
    return false;
  }
  config_path_ = path;
  return true;
}

void VisualizationFrame::syncViewsFromManager() {
  if (views_panel_ != nullptr) {
    views_panel_->setSavedViews(manager_->savedViews());
  }
}

void VisualizationFrame::syncViewsToManager() {
  if (views_panel_ != nullptr) {
    manager_->setSavedViews(views_panel_->savedViews());
  }
}

void VisualizationFrame::captureCurrentView() {
  if (rendering::ViewController* controller = activeViewController()) {
    manager_->setCurrentView(
        common::ToSavedViewConfig("Current", controller->state()));
  }
}

void VisualizationFrame::applyCurrentView() {
  if (!manager_->hasCurrentView()) {
    applyViewController(QString::fromStdString(manager_->viewControllerName()));
    return;
  }
  if (rendering::ViewController* controller = activeViewController()) {
    controller->setState(common::ToViewState(manager_->currentView()));
    if (views_panel_ != nullptr) {
      views_panel_->refreshFromController();
    }
    requestViewportUpdate();
  }
}

void VisualizationFrame::captureWindowLayout() {
  rebuildToolbar();
  manager_->setWindowLayout(
      saveState().toBase64().constData(),
      saveGeometry().toBase64().constData());
  if (main_panel_host_ != nullptr) {
    manager_->setMainPanelLayout(
        main_panel_host_->saveState().toBase64().constData());
  }
  const bool hide_left =
      toolbar_toggle_left_dock_action_ != nullptr
          ? !toolbar_toggle_left_dock_action_->isChecked()
          : manager_->hideLeftDock();
  const bool hide_right =
      toolbar_toggle_right_dock_action_ != nullptr
          ? !toolbar_toggle_right_dock_action_->isChecked()
          : manager_->hideRightDock();
  manager_->setDockHideState(hide_left, hide_right);
  std::vector<common::PanelLayoutConfig> layouts;
  layouts.reserve(orderedDockWidgets().size());
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    common::PanelLayoutConfig entry;
    entry.object_name = dock->objectName().toStdString();
    entry.collapsed = dock->isCollapsed();
    layouts.push_back(std::move(entry));
  }
  manager_->setPanelLayouts(layouts);
  std::vector<std::string> visible_panels;
  visible_panels.reserve(orderedDockWidgets().size());
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    if (dock->isVisible()) {
      visible_panels.push_back(dock->objectName().toStdString());
    }
  }
  bool plot_settings_visible = false;
  if (active_plot_panel_ != nullptr) {
    plot_settings_visible = active_plot_panel_->settingsVisible();
  } else if (plot_panel_ != nullptr) {
    plot_settings_visible = plot_panel_->settingsVisible();
  }
  manager_->setPlotSettingsVisible(plot_settings_visible);
  manager_->setVisiblePanels(visible_panels);
  capturePlotPanelConfigs();
  captureImagePanelConfigs();
  const QRect frame_geometry = geometry();
  manager_->setWindowFrame(frame_geometry.x(), frame_geometry.y(),
                           frame_geometry.width(), frame_geometry.height());
}

void VisualizationFrame::applyPanelVisibility() {
  const std::vector<std::string>& visible = manager_->visiblePanels();
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
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    if (is_visible(dock->objectName().toStdString())) {
      if (isMainPanel(dock)) {
        ensureMainPanelDockAttached(dock);
      } else {
        ensureSidebarDockAttached(dock);
      }
      dock->show();
    } else {
      dock->hide();
    }
  }
  ensureTimeDockAtBottom();
  syncCenterLayout();
  syncDeletePanelMenu();
}

void VisualizationFrame::restoreWindowLayout() {
  expanded_main_panel_dock_ = nullptr;
  pre_expand_main_panel_state_.clear();
  syncMainPanelExpandUi(nullptr);
  const std::string geometry_b64 = manager_->windowGeometryBase64();
  if (!geometry_b64.empty()) {
    restoreGeometry(QByteArray::fromBase64(
        QByteArray::fromStdString(geometry_b64)));
  } else if (manager_->windowWidth() > 0 && manager_->windowHeight() > 0) {
    const int x =
        manager_->windowX() >= 0 ? manager_->windowX() : geometry().x();
    const int y =
        manager_->windowY() >= 0 ? manager_->windowY() : geometry().y();
    setGeometry(x, y, manager_->windowWidth(), manager_->windowHeight());
  }
  const std::string state_b64 = manager_->windowStateBase64();
  if (!state_b64.empty()) {
    restoreState(QByteArray::fromBase64(QByteArray::fromStdString(state_b64)));
  }
  const std::string main_state_b64 = manager_->mainPanelStateBase64();
  if (!main_state_b64.empty() && main_panel_host_ != nullptr) {
    main_panel_host_->restoreState(
        QByteArray::fromBase64(QByteArray::fromStdString(main_state_b64)));
  } else {
    applyMainPanelDefaultLayout();
  }
  syncToolbarToActiveTool();
  syncViewportTitleBarTools();
  restorePanelLayouts();
  applyPanelVisibility();
  restoreDockHideState();
  syncCenterLayout();
}

void VisualizationFrame::restorePanelLayouts() {
  for (const auto& entry : manager_->panelLayouts()) {
    auto* dock = findChild<PanelDockWidget*>(
        QString::fromStdString(NormalizePanelObjectName(entry.object_name)));
    if (dock != nullptr) {
      dock->setCollapsed(entry.collapsed);
    }
  }
}

void VisualizationFrame::onOpenConfig() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Open Config"), QString(),
      tr("Autoviz Config (*.autoviz *.yaml);;RViz Config (*.rviz);;All Files (*)"));
  if (path.isEmpty()) {
    return;
  }
  if (!loadConfig(path)) {
    QMessageBox::warning(this, tr("Open Config"),
                         tr("Failed to load config:\n%1").arg(path));
  }
}

void VisualizationFrame::onSaveConfig() {
  if (config_path_.isEmpty()) {
    onSaveConfigAs();
    return;
  }
  if (!saveConfig(config_path_)) {
    QMessageBox::warning(this, tr("Save Config"),
                         tr("Failed to save config:\n%1").arg(config_path_));
    return;
  }
  clearConfigModified();
  markRecentConfig(config_path_);
}

void VisualizationFrame::onSaveConfigAs() {
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Save Autoviz Config"), config_path_, tr("Autoviz Config (*.autoviz)"));
  if (path.isEmpty()) {
    return;
  }
  if (!saveConfig(path)) {
    QMessageBox::warning(this, tr("Save Config"),
                         tr("Failed to save config:\n%1").arg(path));
    return;
  }
  clearConfigModified();
  markRecentConfig(path);
}

void VisualizationFrame::onFixedFrameChanged(const QString& /*frame*/) {
  syncToolContext();
  updateStatusBar();
}

void VisualizationFrame::onRenderTick() {
  const qint64 elapsed_ms = render_elapsed_.restart();
  const float delta_seconds = static_cast<float>(elapsed_ms) / 1000.f;
  manager_->update();
  viewportTick(delta_seconds);
  const rendering::ReferenceGridSettings& grid_settings =
      manager_->referenceGridSettings();
  if (gl_viewport_ != nullptr) {
    gl_viewport_->setReferenceGridSettings(grid_settings);
  }
  if (ogre_viewport_ != nullptr) {
    ogre_viewport_->setReferenceGridSettings(grid_settings);
  }
  if (rendering::ViewController* controller = activeViewController()) {
    controller->appendFocalShape(&manager_->sceneOverlay());
  }
#ifdef AUTOVIZ_USE_QML_DRONE
  if (vehicle_panel_ != nullptr && drone_dock_ != nullptr && drone_dock_->isVisible()) {
    vehicle_panel_->setFixedFrame(manager_->fixedFrame());
    vehicle_panel_->setTfBuffer(manager_->tfBuffer());
    vehicle_panel_->updateFromTf();
  }
#endif
  updateFps();
}

void VisualizationFrame::onRefreshTick() {
  manager_->refreshChannelList();
  displays_panel_->refreshStatus();
  if (views_panel_ != nullptr) {
    views_panel_->refreshFrameList();
  }
  updateChannelList();
  if (tf_tree_panel_ != nullptr) {
    tf_tree_panel_->refresh();
  }
  updateStatusBar();
}

void VisualizationFrame::onAboutToQuit() {
  render_timer_.stop();
  refresh_timer_.stop();
  manager_->playback().stop();
  manager_->shutdown();

  QSettings settings;
  settings.setValue(QStringLiteral("recent_configs"), recent_configs_);
}

void VisualizationFrame::updateChannelList() {
  if (raw_messages_panel_ != nullptr) {
    raw_messages_panel_->refreshChannels();
  }
  if (topics_panel_ != nullptr) {
    topics_panel_->refreshChannels();
  }
  refreshAllPlotSettingsChannels();
}

void VisualizationFrame::setupStatusBar() {
  status_label_ = new QLabel(this);
  statusBar()->addPermanentWidget(status_label_, 1);

  last_fps_calc_ = std::chrono::steady_clock::now();
  updateStatusBar();
}

void VisualizationFrame::onReset() {
  if (manager_ != nullptr) {
    manager_->resetTime();
  }
  requestViewportUpdate();
}

void VisualizationFrame::updateFps() {
  ++frame_count_;
  const auto now = std::chrono::steady_clock::now();
  if (now - last_fps_calc_ <= std::chrono::seconds(1)) {
    return;
  }
  const double seconds =
      std::chrono::duration<double>(now - last_fps_calc_).count();
  const int fps = seconds > 0.0 ? static_cast<int>(frame_count_ / seconds) : 0;
  frame_count_ = 0;
  last_fps_calc_ = now;
  const QString fps_text = QStringLiteral("%1 fps").arg(fps);
  if (time_panel_ != nullptr) {
    time_panel_->setFpsText(fps_text);
  }
}

void VisualizationFrame::updateStatusBar() {
  const QString tool_text = manager_->tools().activeStatusText();
  const QString base =
      tr("Autolink OK | channels: %1 | Fixed Frame: %2 | View: %3 | Tool: %4")
          .arg(static_cast<int>(manager_->channels().size()))
          .arg(QString::fromStdString(manager_->fixedFrame()))
          .arg(QString::fromStdString(manager_->viewControllerName()))
          .arg(QString::fromStdString(manager_->tools().activeToolId()));
  if (status_label_ != nullptr) {
    status_label_->setText(tool_text.isEmpty() ? base : tool_text + QStringLiteral(" | ") + base);
  }
}

QList<PanelDockWidget*> VisualizationFrame::orderedDockWidgets() const {
  QList<PanelDockWidget*> docks = {viewport_dock_,   displays_dock_,   strata_floor_dock_,
                                   selection_dock_,  tool_props_dock_, views_dock_,
                                   time_dock_,       playback_dock_,   channel_dock_,
                                   topics_dock_,     problems_dock_,   variables_dock_,
                                   property_inspector_dock_,
                                   transformation_dock_, tf_dock_, image_dock_,
                                   plot_dock_};
#ifdef AUTOVIZ_USE_QML_DRONE
  docks.push_back(drone_dock_);
#endif
  docks.push_back(help_dock_);
  for (PanelDockWidget* dock : findChildren<PanelDockWidget*>()) {
    if (dock != nullptr && !docks.contains(dock)) {
      docks.push_back(dock);
    }
  }
  return docks;
}

QStringList VisualizationFrame::hiddenPanels() const {
  QStringList hidden;
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr) {
      continue;
    }
    if (!dock->isVisible()) {
      hidden.push_back(dock->objectName());
    }
  }
  return hidden;
}

void VisualizationFrame::syncDeletePanelMenu() {
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    unregisterDeletePanelAction(dock);
  }
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock != nullptr && dock->isVisible()) {
      registerDeletePanelAction(dock);
    }
  }
}

void VisualizationFrame::restoreExpandedMainPanel() {
  if (expanded_main_panel_dock_ == nullptr || main_panel_host_ == nullptr) {
    return;
  }
  if (!pre_expand_main_panel_state_.isEmpty()) {
    main_panel_host_->restoreState(pre_expand_main_panel_state_);
    pre_expand_main_panel_state_.clear();
  }
  expanded_main_panel_dock_ = nullptr;
  syncMainPanelExpandUi(nullptr);
  syncCenterLayout();
}

void VisualizationFrame::syncMainPanelExpandUi(PanelDockWidget* expanded_dock) {
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr || !isMainPanel(dock)) {
      continue;
    }
    if (auto* plot = qobject_cast<plot::PlotPanel*>(dock->widget())) {
      plot->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
      image->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* log = qobject_cast<log_panel::LogPanel*>(dock->widget())) {
      log->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* tf = qobject_cast<TfTreePanel*>(dock->widget())) {
      tf->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* table = qobject_cast<table::TablePanel*>(dock->widget())) {
      table->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* publish = qobject_cast<publish_panel::PublishPanel*>(dock->widget())) {
      publish->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* gauge = qobject_cast<gauge::GaugePanel*>(dock->widget())) {
      gauge->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* map_panel = qobject_cast<map::MapPanel*>(dock->widget())) {
      map_panel->setExpandButtonChecked(dock == expanded_dock);
    }
    if (auto* indicator = qobject_cast<indicator::IndicatorPanel*>(dock->widget())) {
      indicator->setExpandButtonChecked(dock == expanded_dock);
    }
  }
  if (viewport_expand_button_ != nullptr) {
    viewport_expand_button_->blockSignals(true);
    viewport_expand_button_->setChecked(expanded_dock == viewport_dock_);
    viewport_expand_button_->blockSignals(false);
  }
}

void VisualizationFrame::wireMainPanelExpandTracking(PanelDockWidget* dock) {
  if (dock == nullptr || !isMainPanel(dock) ||
      dock->property("mainPanelExpandTracked").toBool()) {
    return;
  }
  dock->setProperty("mainPanelExpandTracked", true);
  connect(dock, &PanelDockWidget::closed, this, [this, dock]() {
    if (expanded_main_panel_dock_ == dock) {
      restoreExpandedMainPanel();
    }
  });
}

void VisualizationFrame::expandPanelDock(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  last_active_dock_ = dock;

  if (!isMainPanel(dock) || main_panel_host_ == nullptr) {
    if (!dock->isFloating()) {
      dock->setFloating(true);
      const QRect area = geometry();
      dock->resize(area.width() * 3 / 4, area.height() * 3 / 4);
      dock->move(area.center() - dock->rect().center());
    } else {
      dock->showMaximized();
    }
    dock->raise();
    return;
  }

  if (expanded_main_panel_dock_ == dock) {
    restoreExpandedMainPanel();
    dock->raise();
    return;
  }

  if (expanded_main_panel_dock_ != nullptr) {
    restoreExpandedMainPanel();
  }

  pre_expand_main_panel_state_ = main_panel_host_->saveState();
  expanded_main_panel_dock_ = dock;

  ensureMainPanelDockAttached(dock);
  for (PanelDockWidget* candidate : orderedDockWidgets()) {
    if (candidate == nullptr || candidate == dock || !isMainPanel(candidate)) {
      continue;
    }
    if (main_panel_host_->dockWidgetArea(candidate) != Qt::NoDockWidgetArea &&
        candidate->isVisible()) {
      candidate->hide();
    }
  }

  dock->show();
  dock->raise();
  main_panel_host_->syncHorizontalDockLayout();
  syncMainPanelExpandUi(dock);
  syncCenterLayout();
}

void VisualizationFrame::changePanelInDock(PanelDockWidget* source,
                                           const QString& target_object_name) {
  if (source == nullptr || target_object_name.isEmpty()) {
    return;
  }
  auto* target = findChild<PanelDockWidget*>(target_object_name);
  if (target == nullptr || target == source) {
    return;
  }

  const Qt::DockWidgetArea area = dockWidgetArea(source);
  const bool was_floating = source->isFloating();
  const QRect float_geometry = source->geometry();

  PanelDockWidget* tab_anchor = nullptr;
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr || dock == source) {
      continue;
    }
    const QList<QDockWidget*> tabified = tabifiedDockWidgets(dock);
    if (tabified.contains(source)) {
      tab_anchor = dock;
      break;
    }
  }
  if (tab_anchor == nullptr) {
    const QList<QDockWidget*> tabified = tabifiedDockWidgets(source);
    if (!tabified.isEmpty()) {
      tab_anchor = qobject_cast<PanelDockWidget*>(tabified.first());
    }
  }

  source->hide();

  if (was_floating) {
    target->setFloating(true);
    target->setGeometry(float_geometry);
  } else if (area != Qt::NoDockWidgetArea) {
    removeDockWidget(target);
    addDockWidget(area, target);
    if (tab_anchor != nullptr) {
      tabifyDockWidget(tab_anchor, target);
    }
  }

  target->show();
  target->raise();
  last_active_dock_ = target;
  markConfigModified();
}

PanelContextMenuCallbacks VisualizationFrame::makePanelContextMenuCallbacks(
    PanelDockWidget* dock) {
  PanelContextMenuCallbacks callbacks;
  if (dock == nullptr) {
    return callbacks;
  }
  callbacks.current_object_name = panelTypeId(dock);
  callbacks.change_panel = [this, dock](const QString& object_name) {
    changePanelInDock(dock, object_name);
  };
  callbacks.split = [this, dock](Qt::Orientation orientation) {
    onSplitActiveDock(dock, orientation);
  };
  callbacks.expand = [this, dock]() { expandPanelDock(dock); };
  callbacks.remove = [dock]() { dock->close(); };
  return callbacks;
}

void VisualizationFrame::installStandardPanelTitleTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  dock->setTitleBarTools(
      CreateStandardPanelTitleTools(dock, makePanelContextMenuCallbacks(dock)));
}

void VisualizationFrame::syncViewportTitleBarTools() {
  if (viewport_select_tool_ == nullptr || viewport_pan_tool_ == nullptr) {
    return;
  }
  const QString active =
      QString::fromStdString(manager_->tools().activeToolId());
  viewport_select_tool_->blockSignals(true);
  viewport_pan_tool_->blockSignals(true);
  viewport_select_tool_->setChecked(active == QLatin1String("Interact"));
  viewport_pan_tool_->setChecked(active == QLatin1String("MoveCamera"));
  viewport_select_tool_->blockSignals(false);
  viewport_pan_tool_->blockSignals(false);
}

void VisualizationFrame::installViewportTitleBarTools() {
  if (viewport_dock_ == nullptr) {
    return;
  }

  auto* tools = new QWidget(viewport_dock_);
  tools->setStyleSheet(PanelTitleToolsStyleSheet());
  auto* layout = new QHBoxLayout(tools);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  viewport_select_tool_ = CreateTitleToolButton(
      tools, IconLoader::load(QStringLiteral(":/autoviz/icons/cursor.svg")),
      tr("Select objects"), true);
  viewport_select_tool_->setChecked(true);
  layout->addWidget(viewport_select_tool_);
  connect(viewport_select_tool_, &QToolButton::clicked, this, [this]() {
    applyActiveTool("Interact");
  });

  auto* zoom_button = CreateTitleToolButton(
      tools, IconLoader::load(QStringLiteral(":/autoviz/icons/zoom.svg")),
      tr("Reset view"));
  layout->addWidget(zoom_button);
  connect(zoom_button, &QToolButton::clicked, this, [this]() {
    if (rendering::ViewController* controller = activeViewController()) {
      controller->reset();
      if (views_panel_ != nullptr) {
        views_panel_->refreshFromController();
      }
      requestViewportUpdate();
      markConfigModified();
    }
  });

  viewport_pan_tool_ = CreateTitleToolButton(
      tools, IconLoader::load(QStringLiteral(":/autoviz/icons/move2d.svg")),
      tr("Move camera"), true);
  layout->addWidget(viewport_pan_tool_);
  connect(viewport_pan_tool_, &QToolButton::clicked, this, [this]() {
    applyActiveTool("MoveCamera");
  });

  layout->addWidget(CreateTitleSeparator(tools));

  auto* settings_button = CreateTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_settings.svg")),
      tr("View settings"));
  layout->addWidget(settings_button);
  connect(settings_button, &QToolButton::clicked, this, [this]() {
    if (views_dock_ == nullptr) {
      return;
    }
    views_dock_->show();
    if (viewport_dock_ != nullptr && !viewport_dock_->isFloating()) {
      tabifyDockWidget(viewport_dock_, views_dock_);
    }
    views_dock_->raise();
    last_active_dock_ = views_dock_;
  });

  auto* expand_button = CreateTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_fullscreen.svg")),
      tr("Expand panel"), true);
  viewport_expand_button_ = expand_button;
  layout->addWidget(expand_button);
  connect(expand_button, &QToolButton::clicked, this,
          [this]() { expandPanelDock(viewport_dock_); });

  auto* more_button = CreateTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_more.svg")),
      tr("Panel actions"));
  more_button->setPopupMode(QToolButton::InstantPopup);
  more_button->setMenu(
      CreatePanelContextMenu(more_button, makePanelContextMenuCallbacks(viewport_dock_)));
  layout->addWidget(more_button);

  viewport_dock_->setTitleBarTools(tools);
  syncViewportTitleBarTools();
}

void VisualizationFrame::applyFoxgloveDefaultLayout() {
  if (displays_dock_ == nullptr || plot_dock_ == nullptr ||
      viewport_dock_ == nullptr) {
    return;
  }

  if (QMainWindow* outer_host = dockHostForPanel(displays_dock_)) {
    outer_host->removeDockWidget(displays_dock_);
  }
  displays_dock_->show();
  addSidebarDock(displays_dock_, Qt::LeftDockWidgetArea);

  applyMainPanelDefaultLayout();
}

void VisualizationFrame::applyDefaultDockLayout() {
  channel_dock_->hide();
  topics_dock_->show();
  if (problems_dock_ != nullptr) {
    problems_dock_->show();
  }
  if (strata_floor_dock_ != nullptr) {
    strata_floor_dock_->hide();
  }
  setupLeftSidebarTabs();
  topics_dock_->raise();
  tf_dock_->hide();
  transformation_dock_->hide();
  image_dock_->hide();
#ifdef AUTOVIZ_USE_QML_DRONE
  if (drone_dock_ != nullptr) {
    drone_dock_->hide();
  }
#endif
  help_dock_->hide();
  playback_dock_->hide();

  applyFoxgloveDefaultLayout();

  tabifyDockWidget(property_inspector_dock_, selection_dock_);
  tabifyDockWidget(property_inspector_dock_, tool_props_dock_);
  tabifyDockWidget(property_inspector_dock_, views_dock_);
  if (variables_dock_ != nullptr) {
    tabifyDockWidget(property_inspector_dock_, variables_dock_);
  }
  property_inspector_dock_->show();
  property_inspector_dock_->raise();
  QTimer::singleShot(0, this, [this]() {
    if (plot_panel_ != nullptr) {
      bindPlotToPropertyInspector(plot_panel_);
      showPropertyInspector(true);
    }
  });

  ensureTimeDockAtBottom();
  syncCenterLayout();
}

void VisualizationFrame::ensureTimeDockAtBottom() {
  if (time_dock_ == nullptr) {
    return;
  }
  time_dock_->setAllowedAreas(Qt::BottomDockWidgetArea);
  if (dockWidgetArea(time_dock_) != Qt::BottomDockWidgetArea) {
    removeDockWidget(time_dock_);
    addDockWidget(Qt::BottomDockWidgetArea, time_dock_);
  }
  time_dock_->show();
  updateGeometry();
}

void VisualizationFrame::hideDockImpl(Qt::DockWidgetArea area, bool hide) {
  for (PanelDockWidget* dock : findChildren<PanelDockWidget*>()) {
    if (dockWidgetArea(dock) != area) {
      continue;
    }
    dock->setCollapsed(hide);
    if (hide) {
      dock->setAllowedAreas(dock->allowedAreas() & ~area);
    } else {
      dock->setAllowedAreas(dock->allowedAreas() | area);
    }
  }
}

void VisualizationFrame::hideLeftDock(bool hide) {
  hideDockImpl(Qt::LeftDockWidgetArea, hide);
  syncCenterLayout();
}

void VisualizationFrame::hideRightDock(bool hide) {
  hideDockImpl(Qt::RightDockWidgetArea, hide);
  syncCenterLayout();
}

void VisualizationFrame::onHideLeftDockToggled(bool hide) {
  hideLeftDock(hide);
  syncToolbarLayoutControls();
  markConfigModified();
}

void VisualizationFrame::onHideRightDockToggled(bool hide) {
  hideRightDock(hide);
  syncToolbarLayoutControls();
  markConfigModified();
}

void VisualizationFrame::onDockPanelVisibilityChange(bool visible) {
  auto* dock_widget = qobject_cast<PanelDockWidget*>(sender());
  if (dock_widget != nullptr) {
    if (visible) {
      registerDeletePanelAction(dock_widget);
      last_active_dock_ = dock_widget;
    } else {
      unregisterDeletePanelAction(dock_widget);
    }
  }
  if (!visible) {
    return;
  }
  if (dock_widget == nullptr) {
    return;
  }
  const Qt::DockWidgetArea area = dockWidgetArea(dock_widget);
  if (area == Qt::LeftDockWidgetArea && toolbar_toggle_left_dock_action_ != nullptr &&
      !toolbar_toggle_left_dock_action_->isChecked()) {
    toolbar_toggle_left_dock_action_->setChecked(true);
  }
  if (area == Qt::RightDockWidgetArea && toolbar_toggle_right_dock_action_ != nullptr &&
      !toolbar_toggle_right_dock_action_->isChecked()) {
    toolbar_toggle_right_dock_action_->setChecked(true);
  }
  syncToolbarLayoutControls();
  syncCenterLayout();
}

void VisualizationFrame::resizeEvent(QResizeEvent* event) {
  QMainWindow::resizeEvent(event);
  syncCenterLayout();
}

void VisualizationFrame::restoreDockHideState() {
  hideLeftDock(manager_->hideLeftDock());
  hideRightDock(manager_->hideRightDock());
  syncToolbarLayoutControls();
}

void VisualizationFrame::applyBackgroundColor(const QColor& color) {
  if (gl_viewport_ != nullptr) {
    gl_viewport_->setBackgroundColor(color);
  }
  if (ogre_viewport_ != nullptr) {
    ogre_viewport_->setBackgroundColor(color);
  }
  requestViewportUpdate();
}

void VisualizationFrame::populateAddPanelMenu() {
  if (add_panel_menu_ == nullptr) {
    return;
  }
  add_panel_menu_->clear();
  const QStringList hidden = hiddenPanels();
  for (const PanelCatalogEntry& entry : PanelCatalog()) {
    const QString label = tr(entry.label);
    const QString description = tr(entry.description);
    const QIcon icon =
        IconLoader::panelIcon(QString::fromLatin1(entry.icon_id));
    auto* action = add_panel_menu_->addAction(icon, label);
    action->setToolTip(description);
    if (!entry.isImplemented()) {
      action->setEnabled(false);
      continue;
    }
    const QString object_name = QLatin1String(entry.object_name);
    const bool multi_instance = panelTypeSupportsMultiInstance(object_name);
    if (!multi_instance && !hidden.contains(object_name)) {
      action->setEnabled(false);
      action->setToolTip(
          tr("%1\n\nThis panel is already visible.").arg(description));
      continue;
    }
    action->setData(object_name);
  }
}

void VisualizationFrame::onAddPanelMenuTriggered(QAction* action) {
  if (action == nullptr) {
    return;
  }
  showPanelByObjectName(action->data().toString());
}

void VisualizationFrame::showPanelByObjectName(const QString& object_name) {
  if (object_name.isEmpty()) {
    return;
  }

  if (panelTypeSupportsMultiInstance(object_name)) {
    PanelDockWidget* existing_dock = nullptr;
    for (PanelDockWidget* dock : findChildren<PanelDockWidget*>()) {
      if (dock != nullptr && panelTypeId(dock) == object_name) {
        existing_dock = dock;
        break;
      }
    }
    if (existing_dock != nullptr && !existing_dock->isVisible()) {
      ensureMainPanelDockAttached(existing_dock);
      existing_dock->show();
      existing_dock->raise();
      last_active_dock_ = existing_dock;
      activatePanelDock(existing_dock);
      syncDeletePanelMenu();
      markConfigModified();
      if (add_panel_menu_ != nullptr) {
        add_panel_menu_->close();
      }
      return;
    }
    PanelDockWidget* visible_dock = nullptr;
    for (PanelDockWidget* dock : findChildren<PanelDockWidget*>()) {
      if (dock != nullptr && panelTypeId(dock) == object_name && dock->isVisible()) {
        visible_dock = dock;
        break;
      }
    }
    if (visible_dock != nullptr) {
      PanelDockWidget* duplicate = duplicatePanelDock(visible_dock);
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
        activatePanelDock(duplicate);
        syncDeletePanelMenu();
        markConfigModified();
        if (add_panel_menu_ != nullptr) {
          add_panel_menu_->close();
        }
        return;
      }
    }
  }

  auto* dock = findChild<PanelDockWidget*>(object_name);
  if (dock == nullptr && object_name == QLatin1String("PlotDock")) {
    dock = createPlotPanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("ImageDock")) {
    dock = createImagePanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("TeleopDock")) {
    dock = createTeleopPanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("LogDock")) {
    dock = createLogPanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("TfTreeDock")) {
    dock = createTfTreePanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("TableDock")) {
    dock = createTablePanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("PublishDock")) {
    dock = createPublishPanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("GaugeDock")) {
    dock = createGaugePanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("MapDock")) {
    dock = createMapPanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
  }
  if (dock == nullptr && object_name == QLatin1String("IndicatorDock")) {
    dock = createIndicatorPanelDock(object_name);
    addMainPanelDock(dock, Qt::LeftDockWidgetArea);
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
  activatePanelDock(dock);
  syncCenterLayout();
  syncDeletePanelMenu();
  markConfigModified();
  if (add_panel_menu_ != nullptr) {
    add_panel_menu_->close();
  }
}

PanelDockWidget* VisualizationFrame::activeDockForSplit() const {
  if (last_active_dock_ != nullptr && last_active_dock_->isVisible() &&
      last_active_dock_ != time_dock_) {
    return last_active_dock_;
  }
  if (QWidget* focused = QApplication::focusWidget()) {
    for (PanelDockWidget* dock : orderedDockWidgets()) {
      if (dock != nullptr && dock->isVisible() && dock != time_dock_ &&
          dock->isAncestorOf(focused)) {
        return dock;
      }
    }
  }
  if (viewport_dock_ != nullptr && viewport_dock_->isVisible()) {
    return viewport_dock_;
  }
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock != nullptr && dock->isVisible() && dock != time_dock_) {
      return dock;
    }
  }
  return nullptr;
}

void VisualizationFrame::onSplitActiveDock(PanelDockWidget* source,
                                         Qt::Orientation orientation) {
  if (source == nullptr || source == time_dock_) {
    return;
  }
  if (expanded_main_panel_dock_ != nullptr) {
    restoreExpandedMainPanel();
  }
  last_active_dock_ = source;

  const bool is_plot = panelTypeId(source) == QLatin1String("PlotDock");
  const bool is_image = panelTypeId(source) == QLatin1String("ImageDock");
  const bool is_teleop = panelTypeId(source) == QLatin1String("TeleopDock");
  const bool is_log = panelTypeId(source) == QLatin1String("LogDock");
  const bool is_tf = panelTypeId(source) == QLatin1String("TfTreeDock");
  const bool is_table = panelTypeId(source) == QLatin1String("TableDock");
  const bool is_publish = panelTypeId(source) == QLatin1String("PublishDock");
  const bool is_gauge = panelTypeId(source) == QLatin1String("GaugeDock");
  const bool is_map = panelTypeId(source) == QLatin1String("MapDock");
  const bool is_indicator = panelTypeId(source) == QLatin1String("IndicatorDock");

  PanelDockWidget* duplicate = duplicatePanelDock(source);
  if (duplicate == nullptr) {
    QMessageBox::information(
        this, tr("Split Panel"),
        tr("This panel type cannot be duplicated yet."));
    return;
  }

  if (source->isFloating()) {
    source->setFloating(false);
  }
  source->raise();

  duplicate->show();
  QMainWindow* host = dockHostForPanel(source);
  if (host == nullptr) {
    return;
  }
  host->splitDockWidget(source, duplicate, orientation);

  const int primary_size =
      orientation == Qt::Horizontal ? std::max(source->width(), 120)
                                    : std::max(source->height(), 120);
  const int half = std::max(primary_size / 2, 60);
  host->resizeDocks({source, duplicate}, {half, half}, orientation);

  duplicate->raise();
  last_active_dock_ = duplicate;

  if (is_plot) {
    if (auto* panel = qobject_cast<plot::PlotPanel*>(duplicate->widget())) {
      setActivePlotPanel(panel);
    }
  } else if (is_image) {
    if (auto* panel = qobject_cast<image::ImagePanel*>(duplicate->widget())) {
      setActiveImagePanel(panel);
    }
  } else if (is_teleop) {
    if (auto* panel = qobject_cast<teleop::TeleopPanel*>(duplicate->widget())) {
      setActiveTeleopPanel(panel);
    }
  } else if (is_log) {
    if (auto* panel = qobject_cast<log_panel::LogPanel*>(duplicate->widget())) {
      setActiveLogPanel(panel);
    }
  } else if (is_tf) {
    if (auto* panel = qobject_cast<TfTreePanel*>(duplicate->widget())) {
      panel->refresh();
    }
  } else if (is_table) {
    if (auto* panel = qobject_cast<table::TablePanel*>(duplicate->widget())) {
      setActiveTablePanel(panel);
    }
  } else if (is_publish) {
    if (auto* panel = qobject_cast<publish_panel::PublishPanel*>(duplicate->widget())) {
      setActivePublishPanel(panel);
    }
  } else if (is_gauge) {
    if (auto* panel = qobject_cast<gauge::GaugePanel*>(duplicate->widget())) {
      setActiveGaugePanel(panel);
    }
  } else if (is_map) {
    if (auto* panel = qobject_cast<map::MapPanel*>(duplicate->widget())) {
      setActiveMapPanel(panel);
    }
  } else if (is_indicator) {
    if (auto* panel = qobject_cast<indicator::IndicatorPanel*>(duplicate->widget())) {
      setActiveIndicatorPanel(panel);
    }
  }

  syncDeletePanelMenu();
  markConfigModified();
}

bool VisualizationFrame::panelTypeSupportsMultiInstance(
    const QString& panel_type_id) const {
  return panel_type_id == QLatin1String("PlotDock") ||
         panel_type_id == QLatin1String("ImageDock") ||
         panel_type_id == QLatin1String("TeleopDock") ||
         panel_type_id == QLatin1String("LogDock") ||
         panel_type_id == QLatin1String("TfTreeDock") ||
         panel_type_id == QLatin1String("TableDock") ||
         panel_type_id == QLatin1String("PublishDock") ||
         panel_type_id == QLatin1String("GaugeDock") ||
         panel_type_id == QLatin1String("MapDock") ||
         panel_type_id == QLatin1String("IndicatorDock");
}

QString VisualizationFrame::panelTypeId(const PanelDockWidget* dock) const {
  if (dock == nullptr) {
    return {};
  }
  const QVariant type_id = dock->property("panelTypeId");
  if (type_id.isValid()) {
    return type_id.toString();
  }
  return dock->objectName();
}

QString VisualizationFrame::uniquePanelObjectName(const QString& base) const {
  if (findChild<PanelDockWidget*>(base) == nullptr) {
    return base;
  }
  for (int i = 2; i < 1000; ++i) {
    const QString candidate = QStringLiteral("%1_%2").arg(base).arg(i);
    if (findChild<PanelDockWidget*>(candidate) == nullptr) {
      return candidate;
    }
  }
  return QStringLiteral("%1_%2").arg(base).arg(QDateTime::currentMSecsSinceEpoch());
}

void VisualizationFrame::registerPanelDock(PanelDockWidget* dock) {
  if (dock == nullptr || dock->property("panelDockRegistered").toBool()) {
    return;
  }
  dock->setProperty("panelDockRegistered", true);
  connect(dock, &PanelDockWidget::activated, this, [this, dock]() {
    last_active_dock_ = dock;
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
    } else if (panelTypeId(dock) == QLatin1String("LogDock")) {
      if (auto* panel = qobject_cast<log_panel::LogPanel*>(dock->widget())) {
        setActiveLogPanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("TableDock")) {
      if (auto* panel = qobject_cast<table::TablePanel*>(dock->widget())) {
        setActiveTablePanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("PublishDock")) {
      if (auto* panel = qobject_cast<publish_panel::PublishPanel*>(dock->widget())) {
        setActivePublishPanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("GaugeDock")) {
      if (auto* panel = qobject_cast<gauge::GaugePanel*>(dock->widget())) {
        setActiveGaugePanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("MapDock")) {
      if (auto* panel = qobject_cast<map::MapPanel*>(dock->widget())) {
        setActiveMapPanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("IndicatorDock")) {
      if (auto* panel = qobject_cast<indicator::IndicatorPanel*>(dock->widget())) {
        setActiveIndicatorPanel(panel);
      }
    }
  });
  connect(dock, &QDockWidget::visibilityChanged, this,
          &VisualizationFrame::onDockPanelVisibilityChange);
  connect(this, &VisualizationFrame::fullScreenChange, dock,
          &PanelDockWidget::overrideVisibility);
  connect(dock, &QDockWidget::dockLocationChanged, this,
          &VisualizationFrame::markConfigModified);
  connect(dock, &QDockWidget::topLevelChanged, this,
          &VisualizationFrame::markConfigModified);
  connect(dock, &PanelDockWidget::closed, this,
          &VisualizationFrame::markConfigModified);
  wireMainPanelExpandTracking(dock);
}

void VisualizationFrame::updatePlotDockTitle(PanelDockWidget* dock,
                                             plot::PlotPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? tr("Plot") : title);
}

void VisualizationFrame::capturePlotPanelConfigs() {
  std::vector<common::PlotPanelPersistConfig> panels;
  panels.reserve(4);
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr || panelTypeId(dock) != QLatin1String("PlotDock")) {
      continue;
    }
    auto* panel = qobject_cast<plot::PlotPanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panels.push_back(plot::ToPersistConfig(dock->objectName(), panel->config()));
  }
  manager_->setPlotPanels(panels);
}

void VisualizationFrame::ensurePlotDockExists(const QString& object_name) {
  if (object_name.isEmpty() ||
      findChild<PanelDockWidget*>(object_name) != nullptr) {
    return;
  }
  PanelDockWidget* dock = createPlotPanelDock(object_name);
  addMainPanelDock(dock, Qt::LeftDockWidgetArea);
}

void VisualizationFrame::restorePlotPanelConfigs() {
  const std::vector<common::PlotPanelPersistConfig>& saved =
      manager_->plotPanels();
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
    auto* dock = findChild<PanelDockWidget*>(
        QString::fromStdString(entry.object_name));
    auto* panel = dock != nullptr
                      ? qobject_cast<plot::PlotPanel*>(dock->widget())
                      : nullptr;
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(plot::FromPersistConfig(entry));
    updatePlotDockTitle(dock, panel);
  }
}

void VisualizationFrame::captureImagePanelConfigs() {
  std::vector<common::ImagePanelPersistConfig> panels;
  panels.reserve(4);
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock == nullptr || panelTypeId(dock) != QLatin1String("ImageDock")) {
      continue;
    }
    auto* panel = qobject_cast<image::ImagePanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    panels.push_back(image::ToPersistConfig(dock->objectName(), panel->config()));
  }
  manager_->setImagePanels(panels);
}

void VisualizationFrame::ensureImageDockExists(const QString& object_name) {
  if (object_name.isEmpty() ||
      findChild<PanelDockWidget*>(object_name) != nullptr) {
    return;
  }
  PanelDockWidget* dock = createImagePanelDock(object_name);
  addMainPanelDock(dock, Qt::LeftDockWidgetArea);
}

void VisualizationFrame::restoreImagePanelConfigs() {
  const std::vector<common::ImagePanelPersistConfig>& saved =
      manager_->imagePanels();
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
    auto* dock = findChild<PanelDockWidget*>(
        QString::fromStdString(entry.object_name));
    auto* panel = dock != nullptr
                      ? qobject_cast<image::ImagePanel*>(dock->widget())
                      : nullptr;
    if (panel == nullptr) {
      continue;
    }
    panel->setConfig(image::FromPersistConfig(entry));
    updateImageDockTitle(dock, panel);
  }
}

void VisualizationFrame::setupLeftSidebarTabs() {
  if (topics_dock_ == nullptr) {
    return;
  }
  if (problems_dock_ != nullptr) {
    tabifyDockWidget(topics_dock_, problems_dock_);
  }
  if (displays_dock_ != nullptr && problems_dock_ != nullptr) {
    tabifyDockWidget(problems_dock_, displays_dock_);
  }
  topics_dock_->raise();
}

void VisualizationFrame::refreshAllPlotSettingsChannels() {
  for (PanelDockWidget* dock : findChildren<PanelDockWidget*>()) {
    const QString type = panelTypeId(dock);
    if (type == QLatin1String("PlotDock")) {
      if (auto* panel = qobject_cast<plot::PlotPanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    } else if (type == QLatin1String("ImageDock")) {
      if (auto* panel = qobject_cast<image::ImagePanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    } else if (type == QLatin1String("TableDock")) {
      if (auto* panel = qobject_cast<table::TablePanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    } else if (type == QLatin1String("PublishDock")) {
      if (auto* panel = qobject_cast<publish_panel::PublishPanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    } else if (type == QLatin1String("GaugeDock")) {
      if (auto* panel = qobject_cast<gauge::GaugePanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    } else if (type == QLatin1String("MapDock")) {
      if (auto* panel = qobject_cast<map::MapPanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    } else if (type == QLatin1String("IndicatorDock")) {
      if (auto* panel = qobject_cast<indicator::IndicatorPanel*>(dock->widget())) {
        panel->refreshSettingsChannels();
      }
    }
  }
}

void VisualizationFrame::applyPlotSettingsVisibilityFromSession() {
  if (!manager_->plotPanels().empty()) {
    return;
  }
  showPropertyInspector(manager_->plotSettingsVisible());
}

void VisualizationFrame::installPlotFocusTracking() {
  connect(qApp, &QApplication::focusChanged, this,
          [this](QWidget* /*old_focus*/, QWidget* new_focus) {
            if (new_focus == nullptr) {
              return;
            }
            for (PanelDockWidget* dock : orderedDockWidgets()) {
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

void VisualizationFrame::setActivePlotPanel(plot::PlotPanel* panel) {
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
  if (panel != nullptr && active_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(active_log_panel_);
    active_log_panel_ = nullptr;
  }
  if (panel != nullptr && active_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(active_table_panel_);
    active_table_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(active_gauge_panel_);
    active_gauge_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  if (panel != nullptr && active_indicator_panel_ != nullptr) {
    clearPropertyInspectorForIndicator(active_indicator_panel_);
    active_indicator_panel_ = nullptr;
  }
  active_plot_panel_ = panel;
  if (panel != nullptr) {
    bindPlotToPropertyInspector(panel);
  }
}

void VisualizationFrame::setActiveImagePanel(image::ImagePanel* panel) {
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
  if (panel != nullptr && active_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(active_log_panel_);
    active_log_panel_ = nullptr;
  }
  if (panel != nullptr && active_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(active_table_panel_);
    active_table_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(active_gauge_panel_);
    active_gauge_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  if (panel != nullptr && active_indicator_panel_ != nullptr) {
    clearPropertyInspectorForIndicator(active_indicator_panel_);
    active_indicator_panel_ = nullptr;
  }
  active_image_panel_ = panel;
  if (panel != nullptr) {
    bindImageToPropertyInspector(panel);
  }
}

void VisualizationFrame::setActiveTeleopPanel(teleop::TeleopPanel* panel) {
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
  if (panel != nullptr && active_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(active_log_panel_);
    active_log_panel_ = nullptr;
  }
  if (panel != nullptr && active_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(active_table_panel_);
    active_table_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(active_gauge_panel_);
    active_gauge_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  if (panel != nullptr && active_indicator_panel_ != nullptr) {
    clearPropertyInspectorForIndicator(active_indicator_panel_);
    active_indicator_panel_ = nullptr;
  }
  active_teleop_panel_ = panel;
  if (panel != nullptr) {
    bindTeleopToPropertyInspector(panel);
  }
}

void VisualizationFrame::activatePanelDock(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  if (auto* plot = qobject_cast<plot::PlotPanel*>(dock->widget())) {
    setActivePlotPanel(plot);
  } else if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
    setActiveImagePanel(image);
  } else if (auto* teleop = qobject_cast<teleop::TeleopPanel*>(dock->widget())) {
    setActiveTeleopPanel(teleop);
  } else if (auto* log_panel = qobject_cast<log_panel::LogPanel*>(dock->widget())) {
    setActiveLogPanel(log_panel);
  }
}

void VisualizationFrame::setActiveLogPanel(log_panel::LogPanel* panel) {
  if (active_log_panel_ == panel) {
    if (panel != nullptr) {
      bindLogToPropertyInspector(panel);
    }
    return;
  }
  if (active_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(active_log_panel_);
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
  if (panel != nullptr && active_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(active_table_panel_);
    active_table_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(active_gauge_panel_);
    active_gauge_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  if (panel != nullptr && active_indicator_panel_ != nullptr) {
    clearPropertyInspectorForIndicator(active_indicator_panel_);
    active_indicator_panel_ = nullptr;
  }
  active_log_panel_ = panel;
  if (panel != nullptr) {
    bindLogToPropertyInspector(panel);
  }
}

void VisualizationFrame::setActiveTablePanel(table::TablePanel* panel) {
  if (active_table_panel_ == panel) {
    if (panel != nullptr) {
      bindTableToPropertyInspector(panel);
    }
    return;
  }
  if (active_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(active_table_panel_);
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
  if (panel != nullptr && active_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(active_log_panel_);
    active_log_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(active_gauge_panel_);
    active_gauge_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  if (panel != nullptr && active_indicator_panel_ != nullptr) {
    clearPropertyInspectorForIndicator(active_indicator_panel_);
    active_indicator_panel_ = nullptr;
  }
  active_table_panel_ = panel;
  if (panel != nullptr) {
    bindTableToPropertyInspector(panel);
  }
}

void VisualizationFrame::setActivePublishPanel(publish_panel::PublishPanel* panel) {
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
  if (panel != nullptr && active_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(active_log_panel_);
    active_log_panel_ = nullptr;
  }
  if (panel != nullptr && active_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(active_table_panel_);
    active_table_panel_ = nullptr;
  }
  active_publish_panel_ = panel;
  if (panel != nullptr) {
    bindPublishToPropertyInspector(panel);
  }
}

void VisualizationFrame::setActiveGaugePanel(gauge::GaugePanel* panel) {
  if (active_gauge_panel_ == panel) {
    if (panel != nullptr) {
      bindGaugeToPropertyInspector(panel);
    }
    return;
  }
  if (active_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(active_gauge_panel_);
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
  if (panel != nullptr && active_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(active_log_panel_);
    active_log_panel_ = nullptr;
  }
  if (panel != nullptr && active_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(active_table_panel_);
    active_table_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  if (panel != nullptr && active_indicator_panel_ != nullptr) {
    clearPropertyInspectorForIndicator(active_indicator_panel_);
    active_indicator_panel_ = nullptr;
  }
  active_gauge_panel_ = panel;
  if (panel != nullptr) {
    bindGaugeToPropertyInspector(panel);
  }
}

void VisualizationFrame::setActiveMapPanel(map::MapPanel* panel) {
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
  if (panel != nullptr && active_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(active_log_panel_);
    active_log_panel_ = nullptr;
  }
  if (panel != nullptr && active_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(active_table_panel_);
    active_table_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(active_gauge_panel_);
    active_gauge_panel_ = nullptr;
  }
  if (panel != nullptr && active_indicator_panel_ != nullptr) {
    clearPropertyInspectorForIndicator(active_indicator_panel_);
    active_indicator_panel_ = nullptr;
  }
  active_map_panel_ = panel;
  if (panel != nullptr) {
    bindMapToPropertyInspector(panel);
  }
}

void VisualizationFrame::setActiveIndicatorPanel(indicator::IndicatorPanel* panel) {
  if (active_indicator_panel_ == panel) {
    if (panel != nullptr) {
      bindIndicatorToPropertyInspector(panel);
    }
    return;
  }
  if (active_indicator_panel_ != nullptr) {
    clearPropertyInspectorForIndicator(active_indicator_panel_);
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
  if (panel != nullptr && active_log_panel_ != nullptr) {
    clearPropertyInspectorForLog(active_log_panel_);
    active_log_panel_ = nullptr;
  }
  if (panel != nullptr && active_table_panel_ != nullptr) {
    clearPropertyInspectorForTable(active_table_panel_);
    active_table_panel_ = nullptr;
  }
  if (panel != nullptr && active_publish_panel_ != nullptr) {
    clearPropertyInspectorForPublish(active_publish_panel_);
    active_publish_panel_ = nullptr;
  }
  if (panel != nullptr && active_gauge_panel_ != nullptr) {
    clearPropertyInspectorForGauge(active_gauge_panel_);
    active_gauge_panel_ = nullptr;
  }
  if (panel != nullptr && active_map_panel_ != nullptr) {
    clearPropertyInspectorForMap(active_map_panel_);
    active_map_panel_ = nullptr;
  }
  active_indicator_panel_ = panel;
  if (panel != nullptr) {
    bindIndicatorToPropertyInspector(panel);
  }
}

void VisualizationFrame::wirePlotPanel(PanelDockWidget* dock,
                                       plot::PlotPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  connect(panel, &plot::PlotPanel::activated, this,
          [this, panel]() { setActivePlotPanel(panel); });
  connect(panel, &plot::PlotPanel::settingsToggled, this,
          [this, panel](bool visible) {
            setActivePlotPanel(panel);
            showPropertyInspector(visible);
            markConfigModified();
          });
  connect(panel, &plot::PlotPanel::panelSplitRequested, this,
          [this, dock](Qt::Orientation orientation) {
            onSplitActiveDock(dock, orientation);
          });
  connect(panel, &plot::PlotPanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  connect(panel, &plot::PlotPanel::panelExpandRequested, this,
          [this, dock]() { expandPanelDock(dock); });
  connect(panel, &plot::PlotPanel::panelChangeRequested, this,
          [this, dock](const QString& object_name) {
            changePanelInDock(dock, object_name);
          });
  connect(panel, &plot::PlotPanel::configChanged, this, [this, dock, panel]() {
    updatePlotDockTitle(dock, panel);
    if (panel == active_plot_panel_ && property_inspector_panel_ != nullptr) {
      const QString title = panel->config().title.trimmed();
      property_inspector_panel_->setContentWidget(
          panel->settingsWidgetForInspector(),
          title.isEmpty() ? tr("Plot") : title);
    }
    manager_->setPlotSettingsVisible(
        property_inspector_dock_ != nullptr && property_inspector_dock_->isVisible());
    markConfigModified();
  });
  connect(dock, &QDockWidget::visibilityChanged, this,
          [this, dock, panel](bool visible) {
            if (visible || panel != active_plot_panel_) {
              return;
            }
            plot::PlotPanel* fallback = nullptr;
            for (PanelDockWidget* candidate : orderedDockWidgets()) {
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

PanelDockWidget* VisualizationFrame::createPlotPanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("PlotDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(tr("Plot"), this);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("PlotDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelPlot")));
  auto* panel = new plot::PlotPanel(manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wirePlotPanel(dock, panel);
  updatePlotDockTitle(dock, panel);
  configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void VisualizationFrame::updateImageDockTitle(PanelDockWidget* dock,
                                              image::ImagePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? tr("Image") : title);
}

void VisualizationFrame::wireImagePanel(PanelDockWidget* dock,
                                      image::ImagePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  connect(panel, &image::ImagePanel::activated, this,
          [this, panel]() { setActiveImagePanel(panel); });
  connect(panel, &image::ImagePanel::settingsToggled, this,
          [this, panel](bool visible) {
            setActiveImagePanel(panel);
            showPropertyInspector(visible);
            markConfigModified();
          });
  connect(panel, &image::ImagePanel::panelSplitRequested, this,
          [this, dock](Qt::Orientation orientation) {
            onSplitActiveDock(dock, orientation);
          });
  connect(panel, &image::ImagePanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  connect(panel, &image::ImagePanel::panelExpandRequested, this,
          [this, dock]() { expandPanelDock(dock); });
  connect(panel, &image::ImagePanel::panelChangeRequested, this,
          [this, dock](const QString& object_name) {
            changePanelInDock(dock, object_name);
          });
  connect(panel, &image::ImagePanel::configChanged, this, [this, dock, panel]() {
    updateImageDockTitle(dock, panel);
    if (panel == active_image_panel_ && property_inspector_panel_ != nullptr) {
      const QString title = panel->config().title.trimmed();
      property_inspector_panel_->setContentWidget(
          panel->settingsWidgetForInspector(),
          title.isEmpty() ? tr("Image") : title);
    }
    markConfigModified();
  });
  connect(dock, &QDockWidget::visibilityChanged, this,
          [this, dock, panel](bool visible) {
            if (visible || panel != active_image_panel_) {
              return;
            }
            image::ImagePanel* fallback = nullptr;
            for (PanelDockWidget* candidate : orderedDockWidgets()) {
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

PanelDockWidget* VisualizationFrame::createImagePanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("ImageDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(tr("Image"), this);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("ImageDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelImage")));
  auto* panel = new image::ImagePanel(manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireImagePanel(dock, panel);
  updateImageDockTitle(dock, panel);
  configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void VisualizationFrame::updateTeleopDockTitle(PanelDockWidget* dock,
                                             teleop::TeleopPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? tr("Teleop") : title);
}

void VisualizationFrame::wireTeleopPanel(PanelDockWidget* dock,
                                       teleop::TeleopPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  connect(panel, &teleop::TeleopPanel::activated, this,
          [this, panel]() { setActiveTeleopPanel(panel); });
  connect(panel, &teleop::TeleopPanel::settingsToggled, this,
          [this, panel](bool visible) {
            setActiveTeleopPanel(panel);
            showPropertyInspector(visible);
            markConfigModified();
          });
  connect(panel, &teleop::TeleopPanel::panelSplitRequested, this,
          [this, dock](Qt::Orientation orientation) {
            onSplitActiveDock(dock, orientation);
          });
  connect(panel, &teleop::TeleopPanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  connect(panel, &teleop::TeleopPanel::panelChangeRequested, this,
          [this, dock](const QString& object_name) {
            changePanelInDock(dock, object_name);
          });
  connect(panel, &teleop::TeleopPanel::configChanged, this,
          [this, dock, panel]() {
            updateTeleopDockTitle(dock, panel);
            if (panel == active_teleop_panel_ &&
                property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? tr("Teleop") : title);
            }
            markConfigModified();
          });
  connect(dock, &QDockWidget::visibilityChanged, this,
          [this, dock, panel](bool visible) {
            if (visible || panel != active_teleop_panel_) {
              return;
            }
            teleop::TeleopPanel* fallback = nullptr;
            for (PanelDockWidget* candidate : orderedDockWidgets()) {
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

PanelDockWidget* VisualizationFrame::createTeleopPanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("TeleopDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(tr("Teleop"), this);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("TeleopDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelTeleop")));
  auto* panel = new teleop::TeleopPanel(manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireTeleopPanel(dock, panel);
  updateTeleopDockTitle(dock, panel);
  configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void VisualizationFrame::updateLogDockTitle(PanelDockWidget* dock,
                                            log_panel::LogPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? tr("Log") : title);
}

void VisualizationFrame::wireLogPanel(PanelDockWidget* dock, log_panel::LogPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  connect(panel, &log_panel::LogPanel::activated, this,
          [this, panel]() { setActiveLogPanel(panel); });
  connect(panel, &log_panel::LogPanel::settingsToggled, this,
          [this, panel](bool visible) {
            setActiveLogPanel(panel);
            showPropertyInspector(visible);
            markConfigModified();
          });
  connect(panel, &log_panel::LogPanel::panelRemoveRequested, dock, &QDockWidget::close);
  connect(panel, &log_panel::LogPanel::panelExpandRequested, this,
          [this, dock]() { expandPanelDock(dock); });
  connect(panel, &log_panel::LogPanel::panelSplitRequested, this,
          [this, dock](Qt::Orientation orientation) {
            onSplitActiveDock(dock, orientation);
          });
  connect(panel, &log_panel::LogPanel::panelChangeRequested, this,
          [this, dock](const QString& object_name) {
            changePanelInDock(dock, object_name);
          });
  connect(panel, &log_panel::LogPanel::configChanged, this,
          [this, dock, panel]() {
            updateLogDockTitle(dock, panel);
            if (panel == active_log_panel_ && property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? tr("Log") : title);
            }
            markConfigModified();
          });
  connect(dock, &QDockWidget::visibilityChanged, this,
          [this, dock, panel](bool visible) {
            if (visible || panel != active_log_panel_) {
              return;
            }
            log_panel::LogPanel* fallback = nullptr;
            for (PanelDockWidget* candidate : orderedDockWidgets()) {
              if (candidate == nullptr || candidate == dock ||
                  panelTypeId(candidate) != QLatin1String("LogDock") ||
                  !candidate->isVisible()) {
                continue;
              }
              fallback = qobject_cast<log_panel::LogPanel*>(candidate->widget());
              if (fallback != nullptr) {
                break;
              }
            }
            setActiveLogPanel(fallback);
          });
}

PanelDockWidget* VisualizationFrame::createLogPanelDock(const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("LogDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(tr("Log"), this);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("LogDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelLog")));
  auto* panel = new log_panel::LogPanel(manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireLogPanel(dock, panel);
  updateLogDockTitle(dock, panel);
  configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void VisualizationFrame::wireTfTreePanel(PanelDockWidget* dock, TfTreePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  connect(panel, &TfTreePanel::panelRemoveRequested, dock, &QDockWidget::close);
  connect(panel, &TfTreePanel::panelExpandRequested, this,
          [this, dock]() { expandPanelDock(dock); });
  connect(panel, &TfTreePanel::panelSplitRequested, this,
          [this, dock](Qt::Orientation orientation) {
            onSplitActiveDock(dock, orientation);
          });
  connect(panel, &TfTreePanel::panelChangeRequested, this,
          [this, dock](const QString& object_name) {
            changePanelInDock(dock, object_name);
          });
}

PanelDockWidget* VisualizationFrame::createTfTreePanelDock(const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("TfTreeDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(tr("Transform Tree"), this);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("TfTreeDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelTransformTree")));
  auto* panel = new TfTreePanel(manager_->tfBuffer(), manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireTfTreePanel(dock, panel);
  configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void VisualizationFrame::updateTableDockTitle(PanelDockWidget* dock,
                                              table::TablePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? tr("Table") : title);
}

void VisualizationFrame::wireTablePanel(PanelDockWidget* dock,
                                        table::TablePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  connect(panel, &table::TablePanel::activated, this,
          [this, panel]() { setActiveTablePanel(panel); });
  connect(panel, &table::TablePanel::settingsToggled, this,
          [this, panel](bool visible) {
            setActiveTablePanel(panel);
            showPropertyInspector(visible);
            markConfigModified();
          });
  connect(panel, &table::TablePanel::panelRemoveRequested, dock, &QDockWidget::close);
  connect(panel, &table::TablePanel::panelExpandRequested, this,
          [this, dock]() { expandPanelDock(dock); });
  connect(panel, &table::TablePanel::panelSplitRequested, this,
          [this, dock](Qt::Orientation orientation) {
            onSplitActiveDock(dock, orientation);
          });
  connect(panel, &table::TablePanel::panelChangeRequested, this,
          [this, dock](const QString& object_name) {
            changePanelInDock(dock, object_name);
          });
  connect(panel, &table::TablePanel::configChanged, this,
          [this, dock, panel]() {
            updateTableDockTitle(dock, panel);
            if (panel == active_table_panel_ &&
                property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? tr("Table") : title);
            }
            markConfigModified();
          });
  connect(dock, &QDockWidget::visibilityChanged, this,
          [this, dock, panel](bool visible) {
            if (visible || panel != active_table_panel_) {
              return;
            }
            table::TablePanel* fallback = nullptr;
            for (PanelDockWidget* candidate : orderedDockWidgets()) {
              if (candidate == nullptr || candidate == dock ||
                  panelTypeId(candidate) != QLatin1String("TableDock") ||
                  !candidate->isVisible()) {
                continue;
              }
              fallback = qobject_cast<table::TablePanel*>(candidate->widget());
              if (fallback != nullptr) {
                break;
              }
            }
            setActiveTablePanel(fallback);
          });
}

PanelDockWidget* VisualizationFrame::createTablePanelDock(const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("TableDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(tr("Table"), this);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("TableDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelTable")));
  auto* panel = new table::TablePanel(manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireTablePanel(dock, panel);
  updateTableDockTitle(dock, panel);
  configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void VisualizationFrame::updatePublishDockTitle(
    PanelDockWidget* dock, publish_panel::PublishPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? tr("Publish") : title);
}

void VisualizationFrame::wirePublishPanel(PanelDockWidget* dock,
                                          publish_panel::PublishPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  connect(panel, &publish_panel::PublishPanel::activated, this,
          [this, panel]() { setActivePublishPanel(panel); });
  connect(panel, &publish_panel::PublishPanel::settingsToggled, this,
          [this, panel](bool visible) {
            setActivePublishPanel(panel);
            showPropertyInspector(visible);
            markConfigModified();
          });
  connect(panel, &publish_panel::PublishPanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  connect(panel, &publish_panel::PublishPanel::panelExpandRequested, this,
          [this, dock]() { expandPanelDock(dock); });
  connect(panel, &publish_panel::PublishPanel::panelSplitRequested, this,
          [this, dock](Qt::Orientation orientation) {
            onSplitActiveDock(dock, orientation);
          });
  connect(panel, &publish_panel::PublishPanel::panelChangeRequested, this,
          [this, dock](const QString& object_name) {
            changePanelInDock(dock, object_name);
          });
  connect(panel, &publish_panel::PublishPanel::configChanged, this,
          [this, dock, panel]() {
            updatePublishDockTitle(dock, panel);
            if (panel == active_publish_panel_ &&
                property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? tr("Publish") : title);
            }
            markConfigModified();
          });
  connect(dock, &QDockWidget::visibilityChanged, this,
          [this, dock, panel](bool visible) {
            if (visible || panel != active_publish_panel_) {
              return;
            }
            publish_panel::PublishPanel* fallback = nullptr;
            for (PanelDockWidget* candidate : orderedDockWidgets()) {
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

PanelDockWidget* VisualizationFrame::createPublishPanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("PublishDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(tr("Publish"), this);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("PublishDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelPublish")));
  auto* panel = new publish_panel::PublishPanel(manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wirePublishPanel(dock, panel);
  updatePublishDockTitle(dock, panel);
  configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void VisualizationFrame::updateGaugeDockTitle(PanelDockWidget* dock,
                                              gauge::GaugePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? tr("Gauge") : title);
}

void VisualizationFrame::wireGaugePanel(PanelDockWidget* dock,
                                        gauge::GaugePanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  connect(panel, &gauge::GaugePanel::activated, this,
          [this, panel]() { setActiveGaugePanel(panel); });
  connect(panel, &gauge::GaugePanel::settingsToggled, this,
          [this, panel](bool visible) {
            setActiveGaugePanel(panel);
            showPropertyInspector(visible);
            markConfigModified();
          });
  connect(panel, &gauge::GaugePanel::panelRemoveRequested, dock, &QDockWidget::close);
  connect(panel, &gauge::GaugePanel::panelExpandRequested, this,
          [this, dock]() { expandPanelDock(dock); });
  connect(panel, &gauge::GaugePanel::panelSplitRequested, this,
          [this, dock](Qt::Orientation orientation) {
            onSplitActiveDock(dock, orientation);
          });
  connect(panel, &gauge::GaugePanel::panelChangeRequested, this,
          [this, dock](const QString& object_name) {
            changePanelInDock(dock, object_name);
          });
  connect(panel, &gauge::GaugePanel::configChanged, this,
          [this, dock, panel]() {
            updateGaugeDockTitle(dock, panel);
            if (panel == active_gauge_panel_ &&
                property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? tr("Gauge") : title);
            }
            markConfigModified();
          });
  connect(dock, &QDockWidget::visibilityChanged, this,
          [this, dock, panel](bool visible) {
            if (visible || panel != active_gauge_panel_) {
              return;
            }
            gauge::GaugePanel* fallback = nullptr;
            for (PanelDockWidget* candidate : orderedDockWidgets()) {
              if (candidate == nullptr || candidate == dock ||
                  panelTypeId(candidate) != QLatin1String("GaugeDock") ||
                  !candidate->isVisible()) {
                continue;
              }
              fallback = qobject_cast<gauge::GaugePanel*>(candidate->widget());
              if (fallback != nullptr) {
                break;
              }
            }
            setActiveGaugePanel(fallback);
          });
}

PanelDockWidget* VisualizationFrame::createGaugePanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("GaugeDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(tr("Gauge"), this);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("GaugeDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelGauge")));
  auto* panel = new gauge::GaugePanel(manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireGaugePanel(dock, panel);
  updateGaugeDockTitle(dock, panel);
  configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void VisualizationFrame::updateMapDockTitle(PanelDockWidget* dock,
                                            map::MapPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? tr("Map") : title);
}

void VisualizationFrame::wireMapPanel(PanelDockWidget* dock, map::MapPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  connect(panel, &map::MapPanel::activated, this,
          [this, panel]() { setActiveMapPanel(panel); });
  connect(panel, &map::MapPanel::settingsToggled, this,
          [this, panel](bool visible) {
            setActiveMapPanel(panel);
            showPropertyInspector(visible);
            markConfigModified();
          });
  connect(panel, &map::MapPanel::panelRemoveRequested, dock, &QDockWidget::close);
  connect(panel, &map::MapPanel::panelExpandRequested, this,
          [this, dock]() { expandPanelDock(dock); });
  connect(panel, &map::MapPanel::panelSplitRequested, this,
          [this, dock](Qt::Orientation orientation) {
            onSplitActiveDock(dock, orientation);
          });
  connect(panel, &map::MapPanel::panelChangeRequested, this,
          [this, dock](const QString& object_name) {
            changePanelInDock(dock, object_name);
          });
  connect(panel, &map::MapPanel::configChanged, this,
          [this, dock, panel]() {
            updateMapDockTitle(dock, panel);
            if (panel == active_map_panel_ && property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? tr("Map") : title);
            }
            markConfigModified();
          });
  connect(dock, &QDockWidget::visibilityChanged, this,
          [this, dock, panel](bool visible) {
            if (visible || panel != active_map_panel_) {
              return;
            }
            map::MapPanel* fallback = nullptr;
            for (PanelDockWidget* candidate : orderedDockWidgets()) {
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

PanelDockWidget* VisualizationFrame::createMapPanelDock(
    const QString& object_name) {
  const QString dock_name =
      object_name.isEmpty() ? uniquePanelObjectName(QStringLiteral("MapDock"))
                            : object_name;
  auto* dock = new PanelDockWidget(tr("Map"), this);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("MapDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelMap")));
  auto* panel = new map::MapPanel(manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireMapPanel(dock, panel);
  updateMapDockTitle(dock, panel);
  configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

void VisualizationFrame::updateIndicatorDockTitle(PanelDockWidget* dock,
                                                  indicator::IndicatorPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  const QString title = panel->config().title.trimmed();
  dock->setPanelTitle(title.isEmpty() ? tr("Indicator") : title);
}

void VisualizationFrame::wireIndicatorPanel(PanelDockWidget* dock,
                                            indicator::IndicatorPanel* panel) {
  if (dock == nullptr || panel == nullptr) {
    return;
  }
  connect(panel, &indicator::IndicatorPanel::activated, this,
          [this, panel]() { setActiveIndicatorPanel(panel); });
  connect(panel, &indicator::IndicatorPanel::settingsToggled, this,
          [this, panel](bool visible) {
            setActiveIndicatorPanel(panel);
            showPropertyInspector(visible);
            markConfigModified();
          });
  connect(panel, &indicator::IndicatorPanel::panelRemoveRequested, dock,
          &QDockWidget::close);
  connect(panel, &indicator::IndicatorPanel::panelExpandRequested, this,
          [this, dock]() { expandPanelDock(dock); });
  connect(panel, &indicator::IndicatorPanel::panelSplitRequested, this,
          [this, dock](Qt::Orientation orientation) {
            onSplitActiveDock(dock, orientation);
          });
  connect(panel, &indicator::IndicatorPanel::panelChangeRequested, this,
          [this, dock](const QString& object_name) {
            changePanelInDock(dock, object_name);
          });
  connect(panel, &indicator::IndicatorPanel::configChanged, this,
          [this, dock, panel]() {
            updateIndicatorDockTitle(dock, panel);
            if (panel == active_indicator_panel_ &&
                property_inspector_panel_ != nullptr) {
              const QString title = panel->config().title.trimmed();
              property_inspector_panel_->setContentWidget(
                  panel->settingsWidgetForInspector(),
                  title.isEmpty() ? tr("Indicator") : title);
            }
            markConfigModified();
          });
  connect(dock, &QDockWidget::visibilityChanged, this,
          [this, dock, panel](bool visible) {
            if (visible || panel != active_indicator_panel_) {
              return;
            }
            indicator::IndicatorPanel* fallback = nullptr;
            for (PanelDockWidget* candidate : orderedDockWidgets()) {
              if (candidate == nullptr || candidate == dock ||
                  panelTypeId(candidate) != QLatin1String("IndicatorDock") ||
                  !candidate->isVisible()) {
                continue;
              }
              fallback = qobject_cast<indicator::IndicatorPanel*>(candidate->widget());
              if (fallback != nullptr) {
                break;
              }
            }
            setActiveIndicatorPanel(fallback);
          });
}

PanelDockWidget* VisualizationFrame::createIndicatorPanelDock(
    const QString& object_name) {
  const QString dock_name = object_name.isEmpty()
                                ? uniquePanelObjectName(QStringLiteral("IndicatorDock"))
                                : object_name;
  auto* dock = new PanelDockWidget(tr("Indicator"), this);
  dock->setObjectName(dock_name);
  dock->setProperty("panelTypeId", QStringLiteral("IndicatorDock"));
  dock->setPanelIcon(IconLoader::panelIcon(QStringLiteral("PanelIndicator")));
  auto* panel = new indicator::IndicatorPanel(manager_.get(), dock);
  panel->installTitleBarTools(dock);
  dock->setContentWidget(panel);
  wireIndicatorPanel(dock, panel);
  updateIndicatorDockTitle(dock, panel);
  configureMainPanelDock(dock);
  registerPanelDock(dock);
  return dock;
}

PanelDockWidget* VisualizationFrame::duplicatePanelDock(
    PanelDockWidget* source) {
  if (source == nullptr) {
    return nullptr;
  }

  const QString type = panelTypeId(source);
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
  if (type == QLatin1String("LogDock")) {
    PanelDockWidget* dock = createLogPanelDock();
    auto* src_panel = qobject_cast<log_panel::LogPanel*>(source->widget());
    auto* dst_panel = qobject_cast<log_panel::LogPanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updateLogDockTitle(dock, dst_panel);
      setActiveLogPanel(dst_panel);
    }
    return dock;
  }
  if (type == QLatin1String("TfTreeDock")) {
    return createTfTreePanelDock();
  }
  if (type == QLatin1String("TableDock")) {
    PanelDockWidget* dock = createTablePanelDock();
    auto* src_panel = qobject_cast<table::TablePanel*>(source->widget());
    auto* dst_panel = qobject_cast<table::TablePanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updateTableDockTitle(dock, dst_panel);
      setActiveTablePanel(dst_panel);
    }
    return dock;
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
  if (type == QLatin1String("GaugeDock")) {
    PanelDockWidget* dock = createGaugePanelDock();
    auto* src_panel = qobject_cast<gauge::GaugePanel*>(source->widget());
    auto* dst_panel = qobject_cast<gauge::GaugePanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updateGaugeDockTitle(dock, dst_panel);
      setActiveGaugePanel(dst_panel);
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
  if (type == QLatin1String("IndicatorDock")) {
    PanelDockWidget* dock = createIndicatorPanelDock();
    auto* src_panel = qobject_cast<indicator::IndicatorPanel*>(source->widget());
    auto* dst_panel = qobject_cast<indicator::IndicatorPanel*>(dock->widget());
    if (src_panel != nullptr && dst_panel != nullptr) {
      dst_panel->cloneConfigFrom(src_panel->config());
      updateIndicatorDockTitle(dock, dst_panel);
      setActiveIndicatorPanel(dst_panel);
    }
    return dock;
  }

  return nullptr;
}

void VisualizationFrame::onAddPanel() {
  const QStringList available = hiddenPanels();
  if (available.isEmpty()) {
    QMessageBox::information(this, tr("Add New Panel"),
                             tr("All panels are already visible."));
    return;
  }
  AddPanelDialog dialog(available, this);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  showPanelByObjectName(dialog.selectedPanelObjectName());
}

void VisualizationFrame::registerDeletePanelAction(PanelDockWidget* dock) {
  if (dock == nullptr || delete_panel_menu_ == nullptr ||
      delete_panel_actions_.contains(dock)) {
    return;
  }
  auto* action = delete_panel_menu_->addAction(dock->windowTitle(), this,
                                               &VisualizationFrame::onDeletePanel);
  action->setData(QVariant::fromValue(static_cast<QObject*>(dock)));
  delete_panel_actions_.insert(dock, action);
  delete_panel_menu_->setEnabled(true);
}

void VisualizationFrame::unregisterDeletePanelAction(PanelDockWidget* dock) {
  if (dock == nullptr || delete_panel_menu_ == nullptr) {
    return;
  }
  QAction* action = delete_panel_actions_.take(dock);
  if (action == nullptr) {
    return;
  }
  delete_panel_menu_->removeAction(action);
  action->deleteLater();
  delete_panel_menu_->setEnabled(!delete_panel_menu_->actions().isEmpty());
}

void VisualizationFrame::onDeletePanel() {
  auto* action = qobject_cast<QAction*>(sender());
  if (action == nullptr) {
    return;
  }
  auto* dock = qobject_cast<PanelDockWidget*>(action->data().value<QObject*>());
  if (dock != nullptr) {
    dock->hide();
  }
}

void VisualizationFrame::updateRecentConfigMenu() {
  if (recent_configs_menu_ == nullptr) {
    return;
  }
  recent_configs_menu_->clear();
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
    auto* action =
        recent_configs_menu_->addAction(display_name, this,
                                        &VisualizationFrame::onRecentConfigSelected);
    action->setData(path);
  }
}

void VisualizationFrame::markRecentConfig(const QString& path) {
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

void VisualizationFrame::onRecentConfigSelected() {
  auto* action = qobject_cast<QAction*>(sender());
  if (action == nullptr) {
    return;
  }
  const QString path = action->data().toString();
  if (path.isEmpty()) {
    return;
  }
  if (!loadConfig(path)) {
    QMessageBox::warning(this, tr("Open Config"),
                         tr("Failed to load config:\n%1").arg(path));
  }
}

void VisualizationFrame::showHelpPanel() {
  if (help_dock_ == nullptr) {
    return;
  }
  help_dock_->show();
  help_dock_->raise();
}

void VisualizationFrame::onResetDefaultLayout() {
  applyDefaultDockLayout();
  markConfigModified();
}

void VisualizationFrame::onHelpAbout() {
  QString about_text =
      tr("This is Autoviz.\n\nCompiled against Qt version %1.")
          .arg(QLatin1String(QT_VERSION_STR));
#ifdef AUTOVIZ_USE_OGRE
  about_text += tr("\nOgre rendering backend enabled.");
#endif
  QMessageBox::about(QApplication::activeWindow(), tr("About"), about_text);
}

void VisualizationFrame::onViewOrbit() {
  manager_->setViewControllerName("Orbit");
  markConfigModified();
}

void VisualizationFrame::onViewXyOrbit() {
  manager_->setViewControllerName("XYOrbit");
  markConfigModified();
}

void VisualizationFrame::onViewTopDown() {
  manager_->setViewControllerName("TopDown");
  markConfigModified();
}

void VisualizationFrame::onViewTopDownOrtho() {
  manager_->setViewControllerName("TopDownOrtho");
  markConfigModified();
}

void VisualizationFrame::onViewThirdPersonFollow() {
  manager_->setViewControllerName("ThirdPersonFollow");
  markConfigModified();
}

void VisualizationFrame::onViewFps() {
  manager_->setViewControllerName("FPS");
  markConfigModified();
}

void VisualizationFrame::onToggleFullscreen() {
  setFullScreen(!windowState().testFlag(Qt::WindowFullScreen));
}

void VisualizationFrame::onBackendOpenGl() {
  manager_->setRenderBackendName("OpenGL");
  applyRenderBackend(QStringLiteral("OpenGL"));
  syncRenderBackendMenu(QStringLiteral("OpenGL"));
  markConfigModified();
}

void VisualizationFrame::onBackendOgre() {
  manager_->setRenderBackendName("Ogre");
  applyRenderBackend(QStringLiteral("Ogre"));
  syncRenderBackendMenu(QStringLiteral("Ogre"));
  markConfigModified();
}

void VisualizationFrame::syncViewControllerMenu(const QString& name) {
  const auto sync = [&](QAction* action, const char* view_name) {
    if (action != nullptr) {
      action->setChecked(name == QLatin1String(view_name));
    }
  };
  sync(view_orbit_action_, "Orbit");
  sync(view_xy_orbit_action_, "XYOrbit");
  sync(view_topdown_action_, "TopDown");
  sync(view_topdown_ortho_action_, "TopDownOrtho");
  sync(view_third_person_action_, "ThirdPersonFollow");
  sync(view_fps_action_, "FPS");
}

void VisualizationFrame::syncRenderBackendMenu(const QString& name) {
  if (backend_opengl_action_ != nullptr) {
    backend_opengl_action_->setChecked(name == QLatin1String("OpenGL"));
  }
  if (backend_ogre_action_ != nullptr) {
    backend_ogre_action_->setChecked(name == QLatin1String("Ogre"));
  }
}

void VisualizationFrame::setRenderingPaused(bool paused) {
  if (paused) {
    render_timer_.stop();
    refresh_timer_.stop();
  } else {
    applyTargetFrameRate(manager_->targetFrameRate());
    if (!refresh_timer_.isActive()) {
      refresh_timer_.start(1000);
    }
  }
}

}  // namespace autoviz
