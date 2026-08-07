/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <chrono>
#include <functional>
#include <memory>

#include <QActionGroup>
#include <QColor>
#include <QElapsedTimer>
#include <QHash>
#include <QKeyEvent>
#include <QMainWindow>
#include <QTimer>
#include <QVector3D>

class QMenu;
class QShortcut;
class QStackedWidget;
class QToolButton;

#include "autoviz/common/selection.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/app_preferences.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"

class QLabel;
class QListWidget;
class QToolBar;
class QToolButton;
class QVBoxLayout;
class QGridLayout;

class QStackedWidget;

namespace autoviz {

struct AppSettingsResult;

namespace plot {
class PlotPanel;
}
namespace image {
class ImagePanel;
}
namespace teleop {
class TeleopPanel;
}
namespace log_panel {
class LogPanel;
}
namespace table {
class TablePanel;
}
namespace publish_panel {
class PublishPanel;
}
namespace gauge {
class GaugePanel;
}
namespace map {
class MapPanel;
}
namespace indicator {
class IndicatorPanel;
}
namespace service_panel {
class ServicePanel;
}
namespace parameters_panel {
class ParametersPanel;
}
namespace channel_graph {
class ChannelGraphPanel;
}
namespace state_transitions {
class StateTransitionPanel;
}
namespace audio_panel {
class AudioPanel;
}

class DisplaysPanel;
class MainPanelHost;
class PropertyInspectorPanel;
class Vehicle3DPanel;
class HelpPanel;
class PlaybackPanel;
class RawMessagesPanel;
class ChannelsPanel;
class ProblemsPanel;
class VariablesPanel;
class VariableSliderPanel;
class SelectionPanel;
class TfTreePanel;
class TransformationPanel;
class TimePanel;
class ToolPropertiesPanel;
class ViewsPanel;
class StrataFloorPanel;
class ViewportFloatingToolbar;
namespace rendering {
class OgreRenderWindow;
class RenderWindow;
class ViewController;
}

class VisualizationFrame : public QMainWindow {
  Q_OBJECT

 public:
  explicit VisualizationFrame(
      std::shared_ptr<common::VisualizationManager> manager,
      QWidget* parent = nullptr);
  ~VisualizationFrame() override;

  bool loadConfig(const QString& path);
  bool saveConfig(const QString& path);
  void applyStartupWindowState();

  /** Stops viewport render/refresh timers while the widget is off-screen. */
  void setRenderingPaused(bool paused);

 signals:
  void fullScreenChange(bool hidden);

 protected:
  void keyPressEvent(QKeyEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private slots:
  void onRenderTick();
  void onRefreshTick();
  void onAboutToQuit();
  void onOpenConfig();
  void onSaveConfig();
  void onSaveConfigAs();
  void onFixedFrameChanged(const QString& frame);
  void onViewOrbit();
  void onViewXyOrbit();
  void onViewTopDown();
  void onViewTopDownOrtho();
  void onViewThirdPersonFollow();
  void onViewFps();
  void onToggleFullscreen();
  void onBackendOpenGl();
  void onBackendOgre();
  void onScreenshot();
  void setFullScreen(bool full_screen);
  void onToolTriggered(QAction* action);
  void onAddPanel();
  void onAddPanelMenuTriggered(QAction* action);
  void onSplitActiveDock(PanelDockWidget* source, Qt::Orientation orientation);
  void populateAddPanelMenu();
  void showPanelByObjectName(const QString& object_name);
  void onDeletePanel();
  void onRecentConfigSelected();
  void showHelpPanel();
  void onHelpAbout();
  void onAppSettings();
  void onResetDefaultLayout();
  void onHideLeftDockToggled(bool hide);
  void onHideRightDockToggled(bool hide);
  void onDockPanelVisibilityChange(bool visible);
  void applyBackgroundColor(const QColor& color);

 private:
  void setupUi();
  void setupCentralContainer();
  void setupMainPanelHost();
  QMainWindow* dockHostForPanel(const PanelDockWidget* dock) const;
  bool isMainPanel(const PanelDockWidget* dock) const;
  void addMainPanelDock(PanelDockWidget* dock,
                        Qt::DockWidgetArea area = Qt::LeftDockWidgetArea);
  void ensureMainPanelDockAttached(PanelDockWidget* dock,
                                   Qt::DockWidgetArea area = Qt::LeftDockWidgetArea);
  void ensureSidebarDockAttached(PanelDockWidget* dock);
  Qt::DockWidgetArea defaultSidebarArea(const PanelDockWidget* dock) const;
  void syncCenterLayout();
  void addSidebarDock(PanelDockWidget* dock, Qt::DockWidgetArea area);
  void configureMainPanelDock(PanelDockWidget* dock);
  void configureSidebarDock(PanelDockWidget* dock, Qt::DockWidgetArea area);
  void showPropertyInspector(bool visible);
  void bindPlotToPropertyInspector(plot::PlotPanel* panel);
  void clearPropertyInspectorForPlot(plot::PlotPanel* panel);
  void bindImageToPropertyInspector(image::ImagePanel* panel);
  void clearPropertyInspectorForImage(image::ImagePanel* panel);
  void bindTeleopToPropertyInspector(teleop::TeleopPanel* panel);
  void clearPropertyInspectorForTeleop(teleop::TeleopPanel* panel);
  void bindLogToPropertyInspector(log_panel::LogPanel* panel);
  void clearPropertyInspectorForLog(log_panel::LogPanel* panel);
  void bindTableToPropertyInspector(table::TablePanel* panel);
  void clearPropertyInspectorForTable(table::TablePanel* panel);
  void bindPublishToPropertyInspector(publish_panel::PublishPanel* panel);
  void clearPropertyInspectorForPublish(publish_panel::PublishPanel* panel);
  void bindServiceToPropertyInspector(service_panel::ServicePanel* panel);
  void clearPropertyInspectorForService(service_panel::ServicePanel* panel);
  void bindGaugeToPropertyInspector(gauge::GaugePanel* panel);
  void clearPropertyInspectorForGauge(gauge::GaugePanel* panel);
  void bindAudioToPropertyInspector(audio_panel::AudioPanel* panel);
  void clearPropertyInspectorForAudio(audio_panel::AudioPanel* panel);
  void applyMainPanelDefaultLayout();
  void setupMenu();
  void configureMenuBar();
  void setupToolbar();
  void setupToolbarLayoutControls();
  void syncToolbarLayoutControls();
  void configureFlexibleDock(PanelDockWidget* dock);
  struct ViewportPanelEntry {
    PanelDockWidget* dock = nullptr;
    QWidget* host = nullptr;
    QGridLayout* layout = nullptr;
    QWidget* widget = nullptr;
    rendering::RenderWindow* gl_viewport = nullptr;
    rendering::OgreRenderWindow* ogre_viewport = nullptr;
    ViewportFloatingToolbar* floating_toolbar = nullptr;
    QToolButton* expand_button = nullptr;
    QToolButton* settings_button = nullptr;

    rendering::ViewController* viewController() const;
  };
  void registerPrimaryViewportPanel();
  PanelDockWidget* createViewportPanelDock(const QString& object_name = QString());
  void installViewportTitleBarToolsForEntry(ViewportPanelEntry& entry);
  void installViewportFloatingToolbar(ViewportPanelEntry& entry);
  void syncViewportFloatingToolbarForEntry(const ViewportPanelEntry& entry);
  void onViewportInspectTool();
  void onViewportToggle2dCamera();
  void onViewportMeasureTool();
  void onViewportRecenterOnFrame();
  void installStandardPanelTitleTools(PanelDockWidget* dock);
  PanelContextMenuCallbacks makePanelContextMenuCallbacks(PanelDockWidget* dock);
  ViewportPanelEntry* viewportEntryForDock(PanelDockWidget* dock);
  const ViewportPanelEntry* viewportEntryForDock(PanelDockWidget* dock) const;
  ViewportPanelEntry* activeViewportEntry();
  void setActiveViewportDock(PanelDockWidget* dock);
  void forEachViewportPanel(const std::function<void(ViewportPanelEntry&)>& fn);
  void destroyRenderWindowInEntry(ViewportPanelEntry& entry);
  void createRenderWindowInEntry(ViewportPanelEntry& entry, const QString& backend);
  void applyViewportEntryRenderSettings(ViewportPanelEntry& entry);
  void connectViewportInteractionsForEntry(ViewportPanelEntry& entry);
  void syncViewportTitleBarToolsForEntry(const ViewportPanelEntry& entry);
  void wireViewportPanel(ViewportPanelEntry& entry);
  void ensureViewportPanelReady(PanelDockWidget* dock);
  void removeViewportPanel(PanelDockWidget* dock);
  void syncViewportTitleBarTools();
  void expandPanelDock(PanelDockWidget* dock);
  void restoreExpandedMainPanel();
  void syncMainPanelExpandUi(PanelDockWidget* expanded_dock);
  void wireMainPanelExpandTracking(PanelDockWidget* dock);
  void changePanelInDock(PanelDockWidget* source, const QString& target_object_name);
  QString panelTypeId(const PanelDockWidget* dock) const;
  QString uniquePanelObjectName(const QString& base) const;
  PanelDockWidget* createPlotPanelDock(const QString& object_name = QString());
  PanelDockWidget* createImagePanelDock(const QString& object_name = QString());
  PanelDockWidget* createTeleopPanelDock(const QString& object_name = QString());
  PanelDockWidget* createLogPanelDock(const QString& object_name = QString());
  PanelDockWidget* createTfTreePanelDock(const QString& object_name = QString());
  PanelDockWidget* createTablePanelDock(const QString& object_name = QString());
  PanelDockWidget* createPublishPanelDock(const QString& object_name = QString());
  PanelDockWidget* createServicePanelDock(const QString& object_name = QString());
  PanelDockWidget* createGaugePanelDock(const QString& object_name = QString());
  PanelDockWidget* createMapPanelDock(const QString& object_name = QString());
  PanelDockWidget* createIndicatorPanelDock(const QString& object_name = QString());
  PanelDockWidget* createParametersPanelDock(const QString& object_name = QString());
  PanelDockWidget* createChannelGraphPanelDock(const QString& object_name = QString());
  PanelDockWidget* createStateTransitionPanelDock(
      const QString& object_name = QString());
  PanelDockWidget* createAudioPanelDock(const QString& object_name = QString());
  void wirePlotPanel(PanelDockWidget* dock, plot::PlotPanel* panel);
  void wireImagePanel(PanelDockWidget* dock, image::ImagePanel* panel);
  void wireTeleopPanel(PanelDockWidget* dock, teleop::TeleopPanel* panel);
  void wireLogPanel(PanelDockWidget* dock, log_panel::LogPanel* panel);
  void wireTfTreePanel(PanelDockWidget* dock, TfTreePanel* panel);
  void wireTablePanel(PanelDockWidget* dock, table::TablePanel* panel);
  void wirePublishPanel(PanelDockWidget* dock, publish_panel::PublishPanel* panel);
  void wireServicePanel(PanelDockWidget* dock, service_panel::ServicePanel* panel);
  void wireGaugePanel(PanelDockWidget* dock, gauge::GaugePanel* panel);
  void wireMapPanel(PanelDockWidget* dock, map::MapPanel* panel);
  void wireIndicatorPanel(PanelDockWidget* dock, indicator::IndicatorPanel* panel);
  void wireParametersPanel(PanelDockWidget* dock,
                           parameters_panel::ParametersPanel* panel);
  void wireChannelGraphPanel(PanelDockWidget* dock,
                             channel_graph::ChannelGraphPanel* panel);
  void wireStateTransitionPanel(
      PanelDockWidget* dock, state_transitions::StateTransitionPanel* panel);
  void wireAudioPanel(PanelDockWidget* dock, audio_panel::AudioPanel* panel);
  void wireVariableRefresh();
  void refreshGlobalVariableConsumers();
  void setActivePlotPanel(plot::PlotPanel* panel);
  void setActiveImagePanel(image::ImagePanel* panel);
  void setActiveTeleopPanel(teleop::TeleopPanel* panel);
  void setActiveLogPanel(log_panel::LogPanel* panel);
  void setActiveTablePanel(table::TablePanel* panel);
  void setActivePublishPanel(publish_panel::PublishPanel* panel);
  void setActiveServicePanel(service_panel::ServicePanel* panel);
  void setActiveGaugePanel(gauge::GaugePanel* panel);
  void setActiveAudioPanel(audio_panel::AudioPanel* panel);
  void setActiveMapPanel(map::MapPanel* panel);
  void setActiveIndicatorPanel(indicator::IndicatorPanel* panel);
  void setActiveStateTransitionPanel(
      state_transitions::StateTransitionPanel* panel);
  void activatePanelDock(PanelDockWidget* dock);
  void updateImageDockTitle(PanelDockWidget* dock, image::ImagePanel* panel);
  void updateTeleopDockTitle(PanelDockWidget* dock, teleop::TeleopPanel* panel);
  void updateLogDockTitle(PanelDockWidget* dock, log_panel::LogPanel* panel);
  void updateTableDockTitle(PanelDockWidget* dock, table::TablePanel* panel);
  void updatePublishDockTitle(PanelDockWidget* dock,
                              publish_panel::PublishPanel* panel);
  void updateServiceDockTitle(PanelDockWidget* dock,
                              service_panel::ServicePanel* panel);
  void updateGaugeDockTitle(PanelDockWidget* dock, gauge::GaugePanel* panel);
  void updateAudioDockTitle(PanelDockWidget* dock, audio_panel::AudioPanel* panel);
  void updateMapDockTitle(PanelDockWidget* dock, map::MapPanel* panel);
  void updateIndicatorDockTitle(PanelDockWidget* dock,
                                indicator::IndicatorPanel* panel);
  void updateStateTransitionDockTitle(
      PanelDockWidget* dock, state_transitions::StateTransitionPanel* panel);
  void bindMapToPropertyInspector(map::MapPanel* panel);
  void clearPropertyInspectorForMap(map::MapPanel* panel);
  void bindIndicatorToPropertyInspector(indicator::IndicatorPanel* panel);
  void clearPropertyInspectorForIndicator(indicator::IndicatorPanel* panel);
  void bindStateTransitionToPropertyInspector(
      state_transitions::StateTransitionPanel* panel);
  void clearPropertyInspectorForStateTransition(
      state_transitions::StateTransitionPanel* panel);
  void clearStateTransitionInspectorBinding();
  void deactivateStateTransitionIfNot(
      state_transitions::StateTransitionPanel* keep);
  void ensureDefaultStateTransitionDockTab();
  void setupLeftSidebarTabs();
  void refreshAllPlotSettingsChannels();
  void applyPlotSettingsVisibilityFromSession();
  void installPlotFocusTracking();
  void capturePlotPanelConfigs();
  void restorePlotPanelConfigs();
  void ensurePlotDockExists(const QString& object_name);
  void captureImagePanelConfigs();
  void restoreImagePanelConfigs();
  void ensureImageDockExists(const QString& object_name);
  void captureStateTransitionPanelConfigs();
  void restoreStateTransitionPanelConfigs();
  void ensureStateTransitionDockExists(const QString& object_name);
  void updatePlotDockTitle(PanelDockWidget* dock, plot::PlotPanel* panel);
  void registerPanelDock(PanelDockWidget* dock);
  PanelDockWidget* duplicatePanelDock(PanelDockWidget* source);
  bool panelTypeSupportsMultiInstance(const QString& panel_type_id) const;
  void applyFoxgloveDefaultLayout();
  PanelDockWidget* activeDockForSplit() const;
  void setupToolShortcuts();
  void rebuildToolbar();
  void applyActiveTool(const std::string& tool_id);
  void syncActiveToolUi();
  void syncToolbarToActiveTool();
  void updateViewportCursor();
  void onAddToolTriggered();
  void onRemoveToolTriggered(QAction* action);
  void registerPanelMenuToggle(PanelDockWidget* dock);
  void registerPanelToggleActions();
  void syncDeletePanelMenu();
  void syncViewControllerMenu(const QString& name);
  void syncRenderBackendMenu(const QString& name);
  void applyDefaultDockLayout();
  void ensureTimeDockAtBottom();
  void hideDockImpl(Qt::DockWidgetArea area, bool hide);
  void hideLeftDock(bool hide);
  void hideRightDock(bool hide);
  void restoreDockHideState();
  void restorePanelLayouts();
  QList<PanelDockWidget*> orderedDockWidgets() const;
  QStringList hiddenPanels() const;
  void registerDeletePanelAction(PanelDockWidget* dock);
  void unregisterDeletePanelAction(PanelDockWidget* dock);
  void updateRecentConfigMenu();
  void markRecentConfig(const QString& path);
  void updateChannelList();
  void updateStatusBar();
  void updateFps();
  void onReset();
  void setupStatusBar();
  void applyShortcutPreferences(const QHash<QString, QKeySequence>& shortcuts);
  void applyUiPreferences(const AppSettingsResult& settings,
                          const AppUiPreferences& previous_ui);

  void applyViewController(const QString& name);
  void applyRenderBackend(const QString& name);
  void createViewport(const QString& backend);
  void connectViewportInteractions();
  void applyTargetFrameRate(int fps);
  void requestViewportUpdate();
  void viewportTick(float delta_seconds);
  void syncToolContext();
  rendering::ViewController* activeViewController();

  void syncViewsFromManager();
  void syncViewsToManager();
  void captureCurrentView();
  void applyCurrentView();
  void captureWindowLayout();
  void restoreWindowLayout();
  void applyPanelVisibility();
  void updateSelectionPanel(
      const std::vector<common::SelectionEntry>& entries);
  void markConfigModified();
  void clearConfigModified();
  void updateWindowTitle();
  void connectConfigModifiedSignals();

  std::shared_ptr<common::VisualizationManager> manager_;
  MainPanelHost* main_panel_host_ = nullptr;
  QHash<PanelDockWidget*, ViewportPanelEntry> viewport_panels_;
  PanelDockWidget* active_viewport_dock_ = nullptr;
  PanelDockWidget* channel_dock_ = nullptr;
  PanelDockWidget* channels_dock_ = nullptr;
  PanelDockWidget* problems_dock_ = nullptr;
  PanelDockWidget* displays_dock_ = nullptr;
  PanelDockWidget* playback_dock_ = nullptr;
  PanelDockWidget* time_dock_ = nullptr;
  PanelDockWidget* strata_floor_dock_ = nullptr;
  PanelDockWidget* views_dock_ = nullptr;
  PanelDockWidget* tool_props_dock_ = nullptr;
  PanelDockWidget* selection_dock_ = nullptr;
  PanelDockWidget* tf_dock_ = nullptr;
  PanelDockWidget* transformation_dock_ = nullptr;
  PanelDockWidget* image_dock_ = nullptr;
  PanelDockWidget* plot_dock_ = nullptr;
  PanelDockWidget* log_dock_ = nullptr;
  PanelDockWidget* property_inspector_dock_ = nullptr;
  PanelDockWidget* variables_dock_ = nullptr;
  PanelDockWidget* viewport_dock_ = nullptr;
  PanelDockWidget* help_dock_ = nullptr;
#ifdef AUTOVIZ_USE_QML_DRONE
  PanelDockWidget* drone_dock_ = nullptr;
  Vehicle3DPanel* vehicle_panel_ = nullptr;
#endif
  DisplaysPanel* displays_panel_ = nullptr;
  PlaybackPanel* playback_panel_ = nullptr;
  TimePanel* time_panel_ = nullptr;
  StrataFloorPanel* strata_floor_panel_ = nullptr;
  ViewsPanel* views_panel_ = nullptr;
  ToolPropertiesPanel* tool_properties_panel_ = nullptr;
  SelectionPanel* selection_panel_ = nullptr;
  RawMessagesPanel* raw_messages_panel_ = nullptr;
  ChannelsPanel* channels_panel_ = nullptr;
  ProblemsPanel* problems_panel_ = nullptr;
  VariablesPanel* variables_panel_ = nullptr;
  VariableSliderPanel* variable_slider_panel_ = nullptr;
  PanelDockWidget* variable_slider_dock_ = nullptr;
  image::ImagePanel* image_panel_ = nullptr;
  plot::PlotPanel* plot_panel_ = nullptr;
  log_panel::LogPanel* log_panel_ = nullptr;
  plot::PlotPanel* active_plot_panel_ = nullptr;
  image::ImagePanel* active_image_panel_ = nullptr;
  teleop::TeleopPanel* active_teleop_panel_ = nullptr;
  log_panel::LogPanel* active_log_panel_ = nullptr;
  table::TablePanel* active_table_panel_ = nullptr;
  publish_panel::PublishPanel* active_publish_panel_ = nullptr;
  service_panel::ServicePanel* active_service_panel_ = nullptr;
  gauge::GaugePanel* active_gauge_panel_ = nullptr;
  audio_panel::AudioPanel* active_audio_panel_ = nullptr;
  map::MapPanel* active_map_panel_ = nullptr;
  indicator::IndicatorPanel* active_indicator_panel_ = nullptr;
  state_transitions::StateTransitionPanel* active_state_transition_panel_ = nullptr;
  plot::PlotPanel* inspector_plot_panel_ = nullptr;
  image::ImagePanel* inspector_image_panel_ = nullptr;
  teleop::TeleopPanel* inspector_teleop_panel_ = nullptr;
  log_panel::LogPanel* inspector_log_panel_ = nullptr;
  table::TablePanel* inspector_table_panel_ = nullptr;
  publish_panel::PublishPanel* inspector_publish_panel_ = nullptr;
  service_panel::ServicePanel* inspector_service_panel_ = nullptr;
  gauge::GaugePanel* inspector_gauge_panel_ = nullptr;
  audio_panel::AudioPanel* inspector_audio_panel_ = nullptr;
  map::MapPanel* inspector_map_panel_ = nullptr;
  indicator::IndicatorPanel* inspector_indicator_panel_ = nullptr;
  state_transitions::StateTransitionPanel* inspector_state_transition_panel_ =
      nullptr;
  PropertyInspectorPanel* property_inspector_panel_ = nullptr;
  HelpPanel* help_panel_ = nullptr;
  TfTreePanel* tf_tree_panel_ = nullptr;
  TransformationPanel* transformation_panel_ = nullptr;
  QToolBar* tool_bar_ = nullptr;
  QWidget* toolbar_spacer_ = nullptr;
  QToolButton* toolbar_add_panel_button_ = nullptr;
  QMenu* add_panel_menu_ = nullptr;
  QAction* toolbar_toggle_left_dock_action_ = nullptr;
  QAction* toolbar_toggle_right_dock_action_ = nullptr;
  PanelDockWidget* last_active_dock_ = nullptr;
  PanelDockWidget* expanded_main_panel_dock_ = nullptr;
  QByteArray pre_expand_main_panel_state_;
  QActionGroup* tool_action_group_ = nullptr;
  QAction* add_tool_action_ = nullptr;
  QMenu* remove_tool_menu_ = nullptr;
  QList<QAction*> toolbar_tool_actions_;
  QList<QShortcut*> tool_shortcuts_;
  QMenu* panels_menu_ = nullptr;
  QMenu* recent_configs_menu_ = nullptr;
  QMenu* delete_panel_menu_ = nullptr;
  QAction* fullscreen_action_ = nullptr;
  QAction* open_config_action_ = nullptr;
  QAction* save_config_action_ = nullptr;
  QAction* save_config_as_action_ = nullptr;
  QAction* quit_action_ = nullptr;
  QAction* view_orbit_action_ = nullptr;
  QAction* view_xy_orbit_action_ = nullptr;
  QAction* view_topdown_action_ = nullptr;
  QAction* view_topdown_ortho_action_ = nullptr;
  QAction* view_third_person_action_ = nullptr;
  QAction* view_fps_action_ = nullptr;
  QAction* backend_opengl_action_ = nullptr;
  QAction* backend_ogre_action_ = nullptr;
  bool toolbar_visible_ = true;
  QHash<PanelDockWidget*, QAction*> delete_panel_actions_;
  QStringList recent_configs_;
  QTimer render_timer_;
  QTimer refresh_timer_;
  QElapsedTimer render_elapsed_;
  QLabel* status_label_ = nullptr;
  int frame_count_ = 0;
  std::chrono::steady_clock::time_point last_fps_calc_;
  QString config_path_;
  bool config_modified_ = false;
  bool suppress_config_modified_ = false;
  static constexpr int kRecentConfigCount = 10;
};

}  // namespace autoviz
