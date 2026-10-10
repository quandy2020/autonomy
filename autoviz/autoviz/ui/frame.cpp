/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame.hpp"

#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QDockWidget>
#include <QGridLayout>
#include <QGuiApplication>
#include <QTabWidget>
#include <QTimer>
#include <QVector3D>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/selection.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/app/preferences.hpp"
#include "autoviz/ui/channels/channels_panel.hpp"
#include "autoviz/ui/record/record_panel.hpp"
#include "autoviz/ui/displays/panel.hpp"
#include "autoviz/ui/image/image_panel.hpp"
#include "autoviz/ui/inspector/property_panel.hpp"
#include "autoviz/ui/inspector/selection_panel.hpp"
#include "autoviz/ui/inspector/tool_properties_panel.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/panel_host.hpp"
#include "autoviz/ui/plot/plot_panel.hpp"
#include "autoviz/ui/raw/panel.hpp"
#include "autoviz/ui/tf_tree/panel.hpp"
#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/time/panel.hpp"
#include "autoviz/ui/viewport_panel.hpp"
#include "autoviz/ui/views/panel.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

VisualizationFrame::VisualizationFrame(
    std::shared_ptr<common::VisualizationManager> manager, QWidget* parent)
    : QMainWindow(parent), manager_(std::move(manager)) {
  layout_ = std::make_unique<FrameLayout>(this);
  panels_ = std::make_unique<FramePanels>(this);
  viewport_ = std::make_unique<FrameViewport>(this);
  chrome_ = std::make_unique<FrameChrome>(this);
  session_ = std::make_unique<FrameSession>(this);

  const QIcon app_icon = IconLoader::applicationIcon();
  if (!app_icon.isNull()) {
    setWindowIcon(app_icon);
  }
  setupUi();
  setAcceptDrops(true);
  session_->setupRecordDropOverlay();
  qApp->installEventFilter(this);
  setDockNestingEnabled(true);
  setTabPosition(Qt::AllDockWidgetAreas, QTabWidget::North);
  setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowNestedDocks |
                 QMainWindow::AllowTabbedDocks);
  chrome_->setupMenu();
  session_->applyShortcutPreferences(LoadAppUiPreferences().shortcuts);
  chrome_->setupToolbar();

  manager_->setRedrawCallback([this]() { viewport_->requestViewportUpdate(); });
  manager_->setBackgroundColorCallback([this](const std::string& color) {
    session_->applyBackgroundColor(
        common::ParseColorProperty(color, QColor(48, 48, 48)));
  });
  manager_->setViewControllerCallback([this](const std::string& name) {
    viewport_->applyViewController(QString::fromStdString(name));
  });
  manager_->setRenderBackendCallback([this](const std::string& name) {
    viewport_->applyRenderBackend(QString::fromStdString(name));
  });
  manager_->setFrameRateCallback([this](int rate) {
    viewport_->applyTargetFrameRate(rate);
  });

  viewport_->createViewport(QString::fromStdString(manager_->renderBackendName()));
  session_->applyBackgroundColor(
      common::ParseColorProperty(manager_->backgroundColor(), QColor(48, 48, 48)));

  connect(&session_->render_timer_, &QTimer::timeout, this,
          &VisualizationFrame::onRenderTick);
  viewport_->applyTargetFrameRate(manager_->targetFrameRate());
  session_->render_elapsed_.start();

  connect(&session_->refresh_timer_, &QTimer::timeout, this,
          &VisualizationFrame::onRefreshTick);
  session_->refresh_timer_.setTimerType(Qt::PreciseTimer);
  session_->refresh_timer_.start(1000);

  connect(qApp, &QGuiApplication::applicationStateChanged, this,
          [this](Qt::ApplicationState) { session_->syncOffscreenPause(); });

  connect(qApp, &QApplication::aboutToQuit, this,
          &VisualizationFrame::onAboutToQuit);

  connect(panels_->displays_panel_, &DisplaysPanel::fixedFrameChanged, this,
          &VisualizationFrame::onFixedFrameChanged);
  connect(panels_->displays_panel_, &DisplaysPanel::backgroundColorChanged, this,
          &VisualizationFrame::applyBackgroundColor);
  connect(panels_->displays_panel_, &DisplaysPanel::displaysChanged, this, [this]() {
    panels_->syncImageDisplayWindows();
    viewport_->forEachViewportPanel([](ViewportPanelEntry& entry) {
      if (entry.widget != nullptr) {
        entry.widget->update();
      }
    });
  });

  session_->connectConfigModifiedSignals();

  if (manager_->windowStateBase64().empty()) {
    layout_->applyDefaultDockLayout();
  } else {
    layout_->applyMainPanelDefaultLayout();
  }
  session_->restorePanelLayouts();
  layout_->restoreDockHideState();
  layout_->ensureTimeDockAtBottom();
  panels_->syncDeletePanelMenu();

  connect(panels_->time_panel_, &TimePanel::layoutChanged, this, [this]() {
    if (panels_->time_dock_ == nullptr || panels_->time_dock_->isFloating()) {
      session_->markConfigModified();
      return;
    }
    panels_->time_dock_->updateGeometry();
    session_->markConfigModified();
  });
  connect(panels_->time_panel_, &TimePanel::resetRequested, this, [this]() {
    session_->onReset();
  });

  if (panels_->views_panel_ != nullptr) {
    connect(panels_->views_panel_, &ViewsPanel::viewsChanged, this,
            [this]() { session_->syncViewsToManager(); });
    connect(panels_->views_panel_, &ViewsPanel::viewChanged, this, [this]() {
      if (rendering::ViewController* controller = viewport_->activeViewController()) {
        manager_->setViewControllerName(controller->typeName().toStdString());
      }
      viewport_->syncViewportTitleBarTools();
      viewport_->requestViewportUpdate();
    });
  }

  chrome_->updateChannelList();
  chrome_->updateStatusBar();

  manager_->setSelectionChangedCallback(
      [this](const std::vector<common::SelectionEntry>& entries) {
        session_->updateSelectionPanel(entries);
      });
  manager_->setSelectionFocusCallback([this](const QVector3D& target) {
    if (rendering::ViewController* controller = viewport_->activeViewController()) {
      controller->setTarget(target);
      viewport_->requestViewportUpdate();
    }
  });

  viewport_->syncToolContext();
  chrome_->applyActiveTool(manager_->tools().activeToolId());
}

VisualizationFrame::~VisualizationFrame() {
  session_->render_timer_.stop();
  session_->refresh_timer_.stop();
  // PointCloud2 / Map / etc. hold Ogre objects that must be released while
  // render windows still exist. manager_->shutdown() runs after ~Frame.
  if (manager_ != nullptr) {
    manager_->detachDisplaysFromScene();
  }
  viewport_->forEachViewportPanel([](ViewportPanelEntry& entry) {
    if (entry.ogre_viewport != nullptr) {
      if (auto* host = entry.ogre_viewport->ogreSceneHost()) {
        host->clear();
      }
    }
  });
  if (qApp != nullptr) {
    QObject::disconnect(qApp, &QApplication::aboutToQuit, this,
                        &VisualizationFrame::onAboutToQuit);
    qApp->removeEventFilter(this);
  }
}

void VisualizationFrame::setupUi() {
  session_->updateWindowTitle();
  resize(1280, 800);
  layout_->setupCentralContainer();

  viewport_->viewport_dock_ = new PanelDockWidget(tr("3D View"), this);
  viewport_->viewport_dock_->setObjectName(QStringLiteral("ViewportDock"));
  viewport_->viewport_dock_->setProperty("panelTypeId", QStringLiteral("ViewportDock"));
  viewport_->viewport_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Panel3D")));
  auto* viewport_host = new QWidget(viewport_->viewport_dock_);
  viewport_host->setObjectName(QString::fromLatin1(AppThemeIds::kViewportHost));
  viewport_host->setMinimumSize(80, 60);
  auto* viewport_layout = new QGridLayout(viewport_host);
  viewport_layout->setContentsMargins(0, 0, 0, 0);
  viewport_layout->setSpacing(0);
  viewport_->viewport_dock_->setContentWidget(viewport_host);
  layout_->configureMainPanelDock(viewport_->viewport_dock_);
  layout_->addMainPanelDock(viewport_->viewport_dock_, Qt::RightDockWidgetArea);
  viewport_->registerPrimaryViewportPanel();
  if (ViewportPanelEntry* primary = viewport_->viewportEntryForDock(viewport_->viewport_dock_)) {
    viewport_->installViewportFloatingToolbar(*primary);
    viewport_->installViewportHudOverlay(*primary);
  }
  panels_->registerPanelDock(viewport_->viewport_dock_);
  viewport_->wireViewportPanel(*viewport_->viewportEntryForDock(viewport_->viewport_dock_));
  viewport_->installViewportTitleBarToolsForEntry(*viewport_->viewportEntryForDock(viewport_->viewport_dock_));

  panels_->channel_dock_ = new PanelDockWidget(tr("Messages"), this);
  panels_->channel_dock_->setObjectName(QStringLiteral("ChannelsDock"));
  panels_->channel_dock_->setProperty("panelTypeId", QStringLiteral("ChannelsDock"));
  panels_->channel_dock_->setPanelIcon(
      IconLoader::panelIcon(QStringLiteral("PanelRawMessages")));
  panels_->raw_messages_panel_ = new RawMessagesPanel(manager_.get(), panels_->channel_dock_);
  panels_->channel_dock_->setContentWidget(panels_->raw_messages_panel_);
  connect(panels_->raw_messages_panel_, &RawMessagesPanel::addToPlotRequested, this,
          [this](const QString& channel, const QString& field_path) {
            plot::PlotPanel* plot =
                panels_->active_plot_panel_ != nullptr ? panels_->active_plot_panel_ : panels_->plot_panel_;
            if (plot != nullptr) {
              plot->addSeriesFromTopic(channel, field_path);
            }
          });
  connect(panels_->raw_messages_panel_, &RawMessagesPanel::configChanged, this,
          [this]() { session_->markConfigModified(); });
  layout_->addMainPanelDock(panels_->channel_dock_, Qt::LeftDockWidgetArea);
  panels_->channel_dock_->hide();

  panels_->channels_dock_ = new PanelDockWidget(tr("Channels"), this);
  panels_->channels_dock_->setObjectName(QStringLiteral("ChannelBrowserDock"));
  panels_->channels_dock_->setProperty("panelTypeId",
                              QStringLiteral("ChannelBrowserDock"));
  panels_->channels_dock_->setPanelIcon(
      IconLoader::dockPanelIcon(QStringLiteral("ChannelBrowserDock")));
  panels_->channels_panel_ = new ChannelsPanel(manager_.get(), panels_->channels_dock_);
  panels_->channels_dock_->setContentWidget(panels_->channels_panel_);
  connect(panels_->channels_panel_, &ChannelsPanel::openInRawMessagesRequested, this,
          [this](const QString& channel) {
            if (panels_->channel_dock_ != nullptr) {
              panels_->channel_dock_->show();
              panels_->channel_dock_->raise();
            }
            if (panels_->raw_messages_panel_ != nullptr) {
              panels_->raw_messages_panel_->selectChannel(channel);
            }
          });
  connect(panels_->channels_panel_, &ChannelsPanel::addToPlotRequested, this,
          [this](const QString& channel, const QString& field_path) {
            if (panels_->plot_dock_ != nullptr) {
              panels_->plot_dock_->show();
              panels_->plot_dock_->raise();
            }
            plot::PlotPanel* plot = panels_->active_plot_panel_ != nullptr
                                       ? panels_->active_plot_panel_
                                       : panels_->plot_panel_;
            if (plot != nullptr) {
              plot->addSeriesFromTopic(channel, field_path);
            }
          });
  layout_->addMainPanelDock(panels_->channels_dock_, Qt::LeftDockWidgetArea);
  panels_->channels_dock_->hide();

  panels_->displays_dock_ = new PanelDockWidget(tr("Displays"), this);
  panels_->displays_dock_->setObjectName(QStringLiteral("DisplaysDock"));
  panels_->displays_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Displays")));
  panels_->displays_panel_ = new DisplaysPanel(manager_, panels_->displays_dock_);
  panels_->displays_dock_->setContentWidget(panels_->displays_panel_);
  layout_->addSidebarDock(panels_->displays_dock_, Qt::LeftDockWidgetArea);

  panels_->properties_dock_ = new PanelDockWidget(tr("Properties"), this);
  panels_->properties_dock_->setObjectName(QStringLiteral("PropertiesDock"));
  panels_->properties_dock_->setPanelIcon(
      IconLoader::panelIcon(QStringLiteral("ToolProperties")));
  panels_->property_inspector_panel_ = new PropertyInspectorPanel(panels_->properties_dock_);
  panels_->properties_dock_->setContentWidget(panels_->property_inspector_panel_);
  layout_->addSidebarDock(panels_->properties_dock_, Qt::LeftDockWidgetArea);
  tabifyDockWidget(panels_->displays_dock_, panels_->properties_dock_);
  panels_->displays_dock_->raise();
  layout_->left_sidebar_shows_properties_ = false;
  connect(panels_->properties_dock_, &QDockWidget::visibilityChanged, this,
          [this](bool visible) {
            if (visible && panels_->properties_dock_ != nullptr &&
                !panels_->properties_dock_->visibleRegion().isEmpty()) {
              layout_->left_sidebar_shows_properties_ = true;
            }
          });
  connect(panels_->displays_dock_, &PanelDockWidget::closed, this, [this]() {
    layout_->displays_closed_by_user_ = true;
  });
  connect(panels_->displays_dock_, &QDockWidget::visibilityChanged, this,
          [this](bool visible) {
            if (visible && panels_->displays_dock_ != nullptr &&
                !panels_->displays_dock_->visibleRegion().isEmpty() &&
                (panels_->properties_dock_ == nullptr ||
                 panels_->properties_dock_->visibleRegion().isEmpty())) {
              layout_->left_sidebar_shows_properties_ = false;
            }
          });
  if (QAction* displays_toggle = panels_->displays_dock_->toggleViewAction()) {
    connect(displays_toggle, &QAction::triggered, this, [this](bool checked) {
      layout_->displays_closed_by_user_ = !checked;
    });
  }

  panels_->views_dock_ = new PanelDockWidget(tr("Views"), this);
  panels_->views_dock_->setObjectName(QStringLiteral("ViewsDock"));
  panels_->views_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Views")));
  panels_->views_panel_ = new ViewsPanel(nullptr, manager_.get(), panels_->views_dock_);
  panels_->views_dock_->setContentWidget(panels_->views_panel_);
  layout_->addSidebarDock(panels_->views_dock_, Qt::RightDockWidgetArea);
  connect(panels_->views_dock_, &QDockWidget::visibilityChanged, this,
          [this](bool /*visible*/) { viewport_->syncViewportTitleBarTools(); });

  panels_->record_dock_ =
      panels_->createRecordPanelDock(QStringLiteral("RecordDock"));
  layout_->addSidebarDock(panels_->record_dock_, Qt::RightDockWidgetArea);
  tabifyDockWidget(panels_->views_dock_, panels_->record_dock_);
  panels_->views_dock_->raise();

  panels_->tool_props_dock_ = new PanelDockWidget(tr("Tool Properties"), this);
  panels_->tool_props_dock_->setObjectName(QStringLiteral("ToolPropertiesDock"));
  panels_->tool_props_dock_->setPanelIcon(
      IconLoader::panelIcon(QStringLiteral("ToolProperties")));
  panels_->tool_properties_panel_ =
      new ToolPropertiesPanel(manager_, panels_->tool_props_dock_);
  panels_->tool_props_dock_->setContentWidget(panels_->tool_properties_panel_);
  layout_->addSidebarDock(panels_->tool_props_dock_, Qt::RightDockWidgetArea);

  panels_->selection_dock_ = new PanelDockWidget(tr("Selection"), this);
  panels_->selection_dock_->setObjectName(QStringLiteral("SelectionDock"));
  panels_->selection_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Selection")));
  panels_->selection_panel_ = new SelectionPanel(manager_.get(), panels_->selection_dock_);
  panels_->selection_dock_->setContentWidget(panels_->selection_panel_);
  layout_->addSidebarDock(panels_->selection_dock_, Qt::RightDockWidgetArea);

  // Teleop — default hidden on right sidebar; Panels menu toggle / Add Panel.
  panels_->teleop_dock_ = panels_->createTeleopPanelDock(QStringLiteral("TeleopDock"));
  layout_->addSidebarDock(panels_->teleop_dock_, Qt::RightDockWidgetArea);
  panels_->teleop_dock_->hide();

  panels_->tf_dock_ = panels_->createTfTreePanelDock(QStringLiteral("TfTreeDock"));
  panels_->tf_tree_panel_ = qobject_cast<TfTreePanel*>(panels_->tf_dock_->widget());
  layout_->addMainPanelDock(panels_->tf_dock_, Qt::LeftDockWidgetArea);
  panels_->tf_dock_->hide();

  // Channel Graph — default hidden; Panels menu toggle / Add Panel.
  panels_->channel_graph_dock_ =
      panels_->createChannelGraphPanelDock(QStringLiteral("ChannelGraphDock"));
  layout_->addMainPanelDock(panels_->channel_graph_dock_, Qt::LeftDockWidgetArea);
  panels_->channel_graph_dock_->hide();

  panels_->image_dock_ = panels_->createImagePanelDock(QStringLiteral("ImageDock"));
  panels_->image_panel_ = qobject_cast<image::ImagePanel*>(panels_->image_dock_->widget());
  layout_->addMainPanelDock(panels_->image_dock_, Qt::LeftDockWidgetArea);
  // Center column defaults to 3D View only (RViz parity). Image opens via
  // Panels menu / Add Panel / session restore — not at first paint.
  panels_->image_dock_->hide();
  panels_->installImageFocusTracking();
  panels_->setActiveImagePanel(panels_->image_panel_);
  // Image Display decodes on the UI thread; forward frames so the panel still
  // updates even if its own channel handoff races.
  manager_->setImageUpdateCallback(
      [this](const QString& source, const QImage& image) {
        panels_->updateImageDisplayWindowFrame(source, image);
        bool forwarded = false;
        for (PanelDockWidget* dock : layout_->orderedDockWidgets()) {
          if (dock == nullptr || !dock->isVisible() ||
              panels_->panelTypeId(dock) != QLatin1String("ImageDock")) {
            continue;
          }
          if (auto* panel = qobject_cast<image::ImagePanel*>(dock->widget())) {
            panel->setFrameFromDisplay(image);
            forwarded = true;
          }
        }
        if (!forwarded && panels_->image_panel_ != nullptr) {
          panels_->image_panel_->setFrameFromDisplay(image);
        }
      });

  panels_->plot_dock_ = panels_->createPlotPanelDock(QStringLiteral("PlotDock"));
  panels_->plot_panel_ = qobject_cast<plot::PlotPanel*>(panels_->plot_dock_->widget());
  panels_->installPlotFocusTracking();
  layout_->addMainPanelDock(panels_->plot_dock_, Qt::LeftDockWidgetArea);
  panels_->plot_dock_->hide();
  panels_->setActivePlotPanel(panels_->plot_panel_);

  panels_->time_dock_ = new PanelDockWidget(tr("Time"), this);
  panels_->time_dock_->setObjectName(QStringLiteral("TimeDock"));
  panels_->time_dock_->setPanelIcon(IconLoader::panelIcon(QStringLiteral("Time")));
  panels_->time_dock_->setAllowedAreas(Qt::BottomDockWidgetArea);
  panels_->time_dock_->setFeatures(QDockWidget::DockWidgetClosable |
                          QDockWidget::DockWidgetMovable);
  panels_->time_panel_ = new TimePanel(manager_.get(), panels_->time_dock_);
  panels_->time_dock_->setContentWidget(panels_->time_panel_);
  addDockWidget(Qt::BottomDockWidgetArea, panels_->time_dock_);
  panels_->time_dock_->hide();

  for (PanelDockWidget* dock : layout_->orderedDockWidgets()) {
    connect(dock, &QDockWidget::visibilityChanged, this,
            &VisualizationFrame::onDockPanelVisibilityChange,
            Qt::UniqueConnection);
    connect(this, &VisualizationFrame::fullScreenChange, dock,
            &PanelDockWidget::overrideVisibility, Qt::UniqueConnection);
  }

  // Bottom Time bar spans full window width; sidebars stack above it.
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::BottomDockWidgetArea);

  chrome_->setupStatusBar();

  for (PanelDockWidget* dock : layout_->orderedDockWidgets()) {
    if (dock == nullptr || dock == panels_->time_dock_ || dock == viewport_->viewport_dock_) {
      continue;
    }
    // RViz2 Displays / Selection / Tool Properties / Views / Time: no Foxglove
    // title-bar tool strip — only the panel content + close/collapse chrome.
    if (dock == panels_->displays_dock_ || dock == panels_->properties_dock_ ||
        dock == panels_->selection_dock_ ||
        dock == panels_->tool_props_dock_ || dock == panels_->views_dock_) {
      continue;
    }
    if (panels_->panelTypeId(dock) == QLatin1String("PlotDock") ||
        panels_->panelTypeId(dock) == QLatin1String("ImageDock") ||
        panels_->panelTypeId(dock) == QLatin1String("TeleopDock") ||
        panels_->panelTypeId(dock) == QLatin1String("TfTreeDock") ||
        panels_->panelTypeId(dock) == QLatin1String("PublishDock") ||
        panels_->panelTypeId(dock) == QLatin1String("MapDock") ||
        panels_->panelTypeId(dock) == QLatin1String("ChannelGraphDock") ||
        panels_->panelTypeId(dock) == QLatin1String("BehaviorTreeDock") ||
        panels_->panelTypeId(dock) == QLatin1String("ServiceDock")) {
      continue;
    }
    chrome_->installStandardPanelTitleTools(dock);
  }
}

bool VisualizationFrame::loadConfig(const QString& path) {
  return session_->loadConfig(path);
}

bool VisualizationFrame::saveConfig(const QString& path) {
  return session_->saveConfig(path);
}

void VisualizationFrame::applyStartupWindowState() {
  session_->applyStartupWindowState();
}

bool VisualizationFrame::openRecordFile(const QString& path) {
  return session_->openRecordFile(path);
}

void VisualizationFrame::setRenderingPaused(bool paused) {
  session_->setRenderingPaused(paused);
}

void VisualizationFrame::onRenderTick() {
  session_->onRenderTick();
}

void VisualizationFrame::onRefreshTick() {
  session_->onRefreshTick();
}

void VisualizationFrame::onAboutToQuit() {
  session_->onAboutToQuit();
}

void VisualizationFrame::onOpenConfig() {
  session_->onOpenConfig();
}

void VisualizationFrame::onOpenRecord() {
  session_->onOpenRecord();
}

void VisualizationFrame::onSaveConfig() {
  session_->onSaveConfig();
}

void VisualizationFrame::onSaveConfigAs() {
  session_->onSaveConfigAs();
}

void VisualizationFrame::onFixedFrameChanged(const QString& frame_name) {
  session_->onFixedFrameChanged(frame_name);
}

void VisualizationFrame::onToggleFullscreen() {
  session_->onToggleFullscreen();
}

void VisualizationFrame::onBackendOgre() {
  session_->onBackendOgre();
}

void VisualizationFrame::onScreenshot() {
  session_->onScreenshot();
}

void VisualizationFrame::setFullScreen(bool full_screen) {
  session_->setFullScreen(full_screen);
}

void VisualizationFrame::onToolTriggered(QAction* action) {
  chrome_->onToolTriggered(action);
}

void VisualizationFrame::onAddPanel() {
  panels_->onAddPanel();
}

void VisualizationFrame::onSplitActiveDock(PanelDockWidget* source, Qt::Orientation orientation) {
  layout_->onSplitActiveDock(source, orientation);
}

void VisualizationFrame::showPanelByObjectName(const QString& object_name) {
  layout_->showPanelByObjectName(object_name);
}

void VisualizationFrame::onDeletePanel() {
  panels_->onDeletePanel();
}

void VisualizationFrame::onRecentConfigSelected() {
  session_->onRecentConfigSelected();
}

void VisualizationFrame::onHelpAbout() {
  session_->onHelpAbout();
}

void VisualizationFrame::onAppSettings() {
  session_->onAppSettings();
}

void VisualizationFrame::onResetDefaultLayout() {
  session_->onResetDefaultLayout();
}

void VisualizationFrame::onHideLeftDockToggled(bool hide) {
  layout_->onHideLeftDockToggled(hide);
}

void VisualizationFrame::onHideRightDockToggled(bool hide) {
  layout_->onHideRightDockToggled(hide);
}

void VisualizationFrame::onDockPanelVisibilityChange(bool visible) {
  layout_->onDockPanelVisibilityChange(visible);
}

void VisualizationFrame::applyBackgroundColor(const QColor& color) {
  session_->applyBackgroundColor(color);
}

void VisualizationFrame::markConfigModified() {
  session_->markConfigModified();
}

void VisualizationFrame::changeEvent(QEvent* event) {
  QMainWindow::changeEvent(event);
  session_->changeEvent(event);
}

void VisualizationFrame::leaveEvent(QEvent* event) {
  session_->leaveEvent(event);
  QMainWindow::leaveEvent(event);
}

void VisualizationFrame::keyPressEvent(QKeyEvent* event) {
  session_->keyPressEvent(event);
  QMainWindow::keyPressEvent(event);
}

void VisualizationFrame::resizeEvent(QResizeEvent* event) {
  session_->resizeEvent(event);
  QMainWindow::resizeEvent(event);
}

void VisualizationFrame::dragEnterEvent(QDragEnterEvent* event) {
  session_->dragEnterEvent(event);
}

void VisualizationFrame::dragMoveEvent(QDragMoveEvent* event) {
  session_->dragMoveEvent(event);
}

void VisualizationFrame::dragLeaveEvent(QDragLeaveEvent* event) {
  session_->dragLeaveEvent(event);
}

void VisualizationFrame::dropEvent(QDropEvent* event) {
  session_->dropEvent(event);
}

bool VisualizationFrame::eventFilter(QObject* watched, QEvent* event) {
  if (session_->eventFilter(watched, event)) {
    return true;
  }
  return QMainWindow::eventFilter(watched, event);
}

void VisualizationFrame::closeEvent(QCloseEvent* event) {
  // Stop timers / release display GPU objects, then leave the event loop
  // without hideChildren (NVIDIA + WA_PaintOnScreen SIGSEGV on that walk).
  session_->onAboutToQuit();
  event->accept();
  if (qApp != nullptr) {
    qApp->setQuitOnLastWindowClosed(false);
    QCoreApplication::exit(0);
  }
}

void VisualizationFrame::onAddToolTriggered() {
  chrome_->onAddToolTriggered();
}

void VisualizationFrame::onRemoveToolTriggered(QAction* action) {
  chrome_->onRemoveToolTriggered(action);
}

void VisualizationFrame::onReset() {
  session_->onReset();
}

void VisualizationFrame::syncViewsToManager() {
  session_->syncViewsToManager();
}

}  // namespace autoviz
