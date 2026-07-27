/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/visualization_frame.hpp"

#include "autoviz/common/selection.hpp"

#include <algorithm>
#include <QActionGroup>
#include <QApplication>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QKeySequence>
#include <QDateTime>
#include <QDir>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSettings>
#include <QShortcut>
#include <QStatusBar>
#include <QCursor>
#include <QVariant>
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
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/displays_panel.hpp"
#include "autoviz/ui/help_panel.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/image_panel.hpp"
#include "autoviz/ui/playback_panel.hpp"
#include "autoviz/ui/selection_panel.hpp"
#include "autoviz/ui/tool_properties_panel.hpp"
#include "autoviz/ui/tf_tree_panel.hpp"
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

  manager_->setImageUpdateCallback(
      [this](const QString& source, const QImage& image) {
        if (image_panel_ != nullptr) {
          image_panel_->setImage(source, image);
        }
      });

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
  central_container_ = new QWidget(this);
  auto* central_layout = new QHBoxLayout(central_container_);
  central_layout->setContentsMargins(0, 0, 0, 0);
  central_layout->setSpacing(0);

  hide_left_dock_button_ = new QToolButton(central_container_);
  hide_left_dock_button_->setArrowType(Qt::LeftArrow);
  hide_left_dock_button_->setFixedWidth(16);
  hide_left_dock_button_->setAutoRaise(true);
  hide_left_dock_button_->setCheckable(true);
  hide_left_dock_button_->setSizePolicy(QSizePolicy::Minimum,
                                        QSizePolicy::Expanding);
  connect(hide_left_dock_button_, &QToolButton::toggled, this,
          &VisualizationFrame::onHideLeftDockToggled);

  viewport_host_ = new QWidget(central_container_);
  viewport_layout_ = new QVBoxLayout(viewport_host_);
  viewport_layout_->setContentsMargins(0, 0, 0, 0);
  viewport_layout_->setSpacing(0);

  hide_right_dock_button_ = new QToolButton(central_container_);
  hide_right_dock_button_->setArrowType(Qt::RightArrow);
  hide_right_dock_button_->setFixedWidth(16);
  hide_right_dock_button_->setAutoRaise(true);
  hide_right_dock_button_->setCheckable(true);
  hide_right_dock_button_->setSizePolicy(QSizePolicy::Minimum,
                                         QSizePolicy::Expanding);
  connect(hide_right_dock_button_, &QToolButton::toggled, this,
          &VisualizationFrame::onHideRightDockToggled);

  central_layout->addWidget(hide_left_dock_button_);
  central_layout->addWidget(viewport_host_, 1);
  central_layout->addWidget(hide_right_dock_button_);
  setCentralWidget(central_container_);
}

void VisualizationFrame::setupUi() {
  updateWindowTitle();
  resize(1280, 800);
  setupCentralContainer();

  channel_dock_ = new PanelDockWidget(tr("Autolink Channels"), this);
  channel_dock_->setObjectName(QStringLiteral("ChannelsDock"));
  channel_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Channels")));
  channel_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  auto* channel_container = new QWidget(channel_dock_);
  auto* channel_layout = new QVBoxLayout(channel_container);
  channel_list_ = new QListWidget(channel_container);
  channel_layout->addWidget(channel_list_);
  channel_container->setLayout(channel_layout);
  channel_dock_->setContentWidget(channel_container);
  addDockWidget(Qt::LeftDockWidgetArea, channel_dock_);

  displays_dock_ = new PanelDockWidget(tr("Displays"), this);
  displays_dock_->setObjectName(QStringLiteral("DisplaysDock"));
  displays_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Displays")));
  displays_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  displays_panel_ = new DisplaysPanel(manager_, displays_dock_);
  displays_dock_->setContentWidget(displays_panel_);
  addDockWidget(Qt::LeftDockWidgetArea, displays_dock_);

  strata_floor_dock_ = new PanelDockWidget(tr("Strata Floors"), this);
  strata_floor_dock_->setObjectName(QStringLiteral("StrataFloorDock"));
  strata_floor_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Displays")));
  strata_floor_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  strata_floor_panel_ = new StrataFloorPanel(manager_.get(), strata_floor_dock_);
  strata_floor_dock_->setContentWidget(strata_floor_panel_);
  addDockWidget(Qt::LeftDockWidgetArea, strata_floor_dock_);

  playback_dock_ = new PanelDockWidget(tr("Playback"), this);
  playback_dock_->setObjectName(QStringLiteral("PlaybackDock"));
  playback_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Playback")));
  playback_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  playback_panel_ = new PlaybackPanel(&manager_->playback(), playback_dock_);
  playback_dock_->setContentWidget(playback_panel_);
  addDockWidget(Qt::RightDockWidgetArea, playback_dock_);

  views_dock_ = new PanelDockWidget(tr("Views"), this);
  views_dock_->setObjectName(QStringLiteral("ViewsDock"));
  views_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Views")));
  views_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  views_panel_ = new ViewsPanel(nullptr, manager_.get(), views_dock_);
  views_dock_->setContentWidget(views_panel_);
  addDockWidget(Qt::RightDockWidgetArea, views_dock_);

  tool_props_dock_ = new PanelDockWidget(tr("Tool Properties"), this);
  tool_props_dock_->setObjectName(QStringLiteral("ToolPropertiesDock"));
  tool_props_dock_->setPanelIcon(
      IconLoader::panelIcon(QStringLiteral("ToolProperties")));
  tool_props_dock_->setAllowedAreas(Qt::LeftDockWidgetArea |
                                   Qt::RightDockWidgetArea);
  tool_properties_panel_ =
      new ToolPropertiesPanel(manager_, tool_props_dock_);
  tool_props_dock_->setContentWidget(tool_properties_panel_);
  addDockWidget(Qt::RightDockWidgetArea, tool_props_dock_);

  selection_dock_ = new PanelDockWidget(tr("Selection"), this);
  selection_dock_->setObjectName(QStringLiteral("SelectionDock"));
  selection_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Selection")));
  selection_dock_->setAllowedAreas(Qt::LeftDockWidgetArea |
                                  Qt::RightDockWidgetArea);
  selection_panel_ = new SelectionPanel(selection_dock_);
  selection_dock_->setContentWidget(selection_panel_);
  addDockWidget(Qt::RightDockWidgetArea, selection_dock_);

  tf_dock_ = new PanelDockWidget(tr("TF Tree"), this);
  tf_dock_->setObjectName(QStringLiteral("TfTreeDock"));
  tf_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("TfTree")));
  tf_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  tf_tree_panel_ = new TfTreePanel(manager_->tfBuffer(), tf_dock_);
  tf_dock_->setContentWidget(tf_tree_panel_);
  addDockWidget(Qt::RightDockWidgetArea, tf_dock_);

  transformation_dock_ = new PanelDockWidget(tr("Transformation"), this);
  transformation_dock_->setObjectName(QStringLiteral("TransformationDock"));
  transformation_dock_->setPanelIcon(
      IconLoader::panelIcon(QStringLiteral("Transformation")));
  transformation_dock_->setAllowedAreas(Qt::LeftDockWidgetArea |
                                        Qt::RightDockWidgetArea);
  transformation_panel_ = new TransformationPanel(
      &manager_->transformationManager(), transformation_dock_);
  transformation_dock_->setContentWidget(transformation_panel_);
  addDockWidget(Qt::RightDockWidgetArea, transformation_dock_);
  connect(transformation_panel_, &TransformationPanel::transformerChanged,
          this, [this]() { requestViewportUpdate(); });

  image_dock_ = new PanelDockWidget(tr("Image"), this);
  image_dock_->setObjectName(QStringLiteral("ImageDock"));
  image_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Image")));
  image_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  image_panel_ = new ImagePanel(image_dock_);
  image_dock_->setContentWidget(image_panel_);
  addDockWidget(Qt::RightDockWidgetArea, image_dock_);

  help_dock_ = new PanelDockWidget(tr("Help"), this);
  help_dock_->setObjectName(QStringLiteral("HelpDock"));
  help_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Help")));
  help_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  help_panel_ = new HelpPanel(help_dock_);
  help_dock_->setContentWidget(help_panel_);
  addDockWidget(Qt::RightDockWidgetArea, help_dock_);

#ifdef AUTOVIZ_USE_QML_DRONE
  drone_dock_ = new PanelDockWidget(tr("Vehicle 3D"), this);
  drone_dock_->setObjectName(QStringLiteral("Drone3DDock"));
  drone_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Views")));
  drone_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  vehicle_panel_ = new Vehicle3DPanel(drone_dock_);
  drone_dock_->setContentWidget(vehicle_panel_);
  addDockWidget(Qt::RightDockWidgetArea, drone_dock_);
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

  // Bottom Time bar spans full window width (RViz default.rviz layout).
  setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::BottomDockWidgetArea);

  setupStatusBar();
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
  if (hide_left_dock_button_ != nullptr) {
    hide_left_dock_button_->setVisible(!full_screen);
  }
  if (hide_right_dock_button_ != nullptr) {
    hide_right_dock_button_->setVisible(!full_screen);
  }
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
  applyActiveTool(manager_->tools().activeToolId());
  ensureTimeDockAtBottom();
  syncDeletePanelMenu();
  if (image_panel_ != nullptr) {
    image_panel_->setImage(manager_->latestImageSource(),
                           manager_->latestImage());
  }
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
  manager_->setDockHideState(hide_left_dock_button_->isChecked(),
                             hide_right_dock_button_->isChecked());
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
    if (dock != nullptr && dock->isVisible()) {
      visible_panels.push_back(dock->objectName().toStdString());
    }
  }
  manager_->setVisiblePanels(visible_panels);
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
      dock->show();
    } else {
      dock->hide();
    }
  }
  ensureTimeDockAtBottom();
  syncDeletePanelMenu();
}

void VisualizationFrame::restoreWindowLayout() {
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
  syncToolbarToActiveTool();
  restorePanelLayouts();
  applyPanelVisibility();
  restoreDockHideState();
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
  channel_list_->clear();
  for (const auto& channel : manager_->channels()) {
    channel_list_->addItem(
        QStringLiteral("%1  [%2]")
            .arg(QString::fromStdString(channel.channel_name),
                 QString::fromStdString(
                     commsgs::NormalizeMessageType(channel.message_type))));
  }
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
  QList<PanelDockWidget*> docks = {displays_dock_,   strata_floor_dock_, selection_dock_,
                                   tool_props_dock_, views_dock_,        time_dock_,
                                   playback_dock_,   channel_dock_,      transformation_dock_,
                                   tf_dock_,         image_dock_};
#ifdef AUTOVIZ_USE_QML_DRONE
  docks.push_back(drone_dock_);
#endif
  docks.push_back(help_dock_);
  return docks;
}

QStringList VisualizationFrame::hiddenPanels() const {
  QStringList hidden;
  for (PanelDockWidget* dock : orderedDockWidgets()) {
    if (dock != nullptr && !dock->isVisible()) {
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

void VisualizationFrame::applyDefaultDockLayout() {
  channel_dock_->hide();
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

  tabifyDockWidget(selection_dock_, tool_props_dock_);
  tabifyDockWidget(selection_dock_, views_dock_);
  selection_dock_->raise();

  ensureTimeDockAtBottom();
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
  if (hide_left_dock_button_ != nullptr) {
    hide_left_dock_button_->setArrowType(hide ? Qt::RightArrow : Qt::LeftArrow);
  }
}

void VisualizationFrame::hideRightDock(bool hide) {
  hideDockImpl(Qt::RightDockWidgetArea, hide);
  if (hide_right_dock_button_ != nullptr) {
    hide_right_dock_button_->setArrowType(hide ? Qt::LeftArrow : Qt::RightArrow);
  }
}

void VisualizationFrame::onHideLeftDockToggled(bool hide) {
  hideLeftDock(hide);
  markConfigModified();
}

void VisualizationFrame::onHideRightDockToggled(bool hide) {
  hideRightDock(hide);
  markConfigModified();
}

void VisualizationFrame::onDockPanelVisibilityChange(bool visible) {
  auto* dock_widget = qobject_cast<PanelDockWidget*>(sender());
  if (dock_widget != nullptr) {
    if (visible) {
      registerDeletePanelAction(dock_widget);
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
  if (area == Qt::LeftDockWidgetArea && hide_left_dock_button_ != nullptr &&
      hide_left_dock_button_->isChecked()) {
    hide_left_dock_button_->setChecked(false);
  }
  if (area == Qt::RightDockWidgetArea && hide_right_dock_button_ != nullptr &&
      hide_right_dock_button_->isChecked()) {
    hide_right_dock_button_->setChecked(false);
  }
}

void VisualizationFrame::restoreDockHideState() {
  if (hide_left_dock_button_ == nullptr || hide_right_dock_button_ == nullptr) {
    return;
  }
  hide_left_dock_button_->blockSignals(true);
  hide_right_dock_button_->blockSignals(true);
  hide_left_dock_button_->setChecked(manager_->hideLeftDock());
  hide_left_dock_button_->setArrowType(
      manager_->hideLeftDock() ? Qt::RightArrow : Qt::LeftArrow);
  hide_right_dock_button_->setChecked(manager_->hideRightDock());
  hide_right_dock_button_->setArrowType(
      manager_->hideRightDock() ? Qt::LeftArrow : Qt::RightArrow);
  hide_left_dock_button_->blockSignals(false);
  hide_right_dock_button_->blockSignals(false);
  hideLeftDock(manager_->hideLeftDock());
  hideRightDock(manager_->hideRightDock());
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
  const QString object_name = dialog.selectedPanelObjectName();
  if (object_name.isEmpty()) {
    return;
  }
  PanelDockWidget* dock = findChild<PanelDockWidget*>(object_name);
  if (dock == nullptr) {
    return;
  }
  dock->show();
  dock->raise();
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
