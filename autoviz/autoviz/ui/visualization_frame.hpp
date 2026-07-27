/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <chrono>
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

#include "autoviz/common/selection.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"

class QLabel;
class QListWidget;
class QToolBar;
class QToolButton;
class QVBoxLayout;

namespace autoviz {

class DisplaysPanel;
class Vehicle3DPanel;
class HelpPanel;
class ImagePanel;
class PlaybackPanel;
class SelectionPanel;
class TfTreePanel;
class TransformationPanel;
class TimePanel;
class ToolPropertiesPanel;
class ViewsPanel;
class StrataFloorPanel;
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

  /** Stops viewport render/refresh timers while the widget is off-screen. */
  void setRenderingPaused(bool paused);

 signals:
  void fullScreenChange(bool hidden);

 protected:
  void keyPressEvent(QKeyEvent* event) override;

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
  void onDeletePanel();
  void onRecentConfigSelected();
  void showHelpPanel();
  void onHelpAbout();
  void onHideLeftDockToggled(bool hide);
  void onHideRightDockToggled(bool hide);
  void onDockPanelVisibilityChange(bool visible);
  void applyBackgroundColor(const QColor& color);

 private:
  void setupUi();
  void setupCentralContainer();
  void setupMenu();
  void setupToolbar();
  void setupToolShortcuts();
  void rebuildToolbar();
  void applyActiveTool(const std::string& tool_id);
  void syncActiveToolUi();
  void syncToolbarToActiveTool();
  void updateViewportCursor();
  void onAddToolTriggered();
  void onRemoveToolTriggered(QAction* action);
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
  QWidget* central_container_ = nullptr;
  QWidget* viewport_host_ = nullptr;
  QVBoxLayout* viewport_layout_ = nullptr;
  QWidget* viewport_widget_ = nullptr;
  QToolButton* hide_left_dock_button_ = nullptr;
  QToolButton* hide_right_dock_button_ = nullptr;
  rendering::RenderWindow* gl_viewport_ = nullptr;
  rendering::OgreRenderWindow* ogre_viewport_ = nullptr;
  PanelDockWidget* channel_dock_ = nullptr;
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
  ImagePanel* image_panel_ = nullptr;
  HelpPanel* help_panel_ = nullptr;
  TfTreePanel* tf_tree_panel_ = nullptr;
  TransformationPanel* transformation_panel_ = nullptr;
  QListWidget* channel_list_ = nullptr;
  QToolBar* tool_bar_ = nullptr;
  QActionGroup* tool_action_group_ = nullptr;
  QAction* add_tool_action_ = nullptr;
  QMenu* remove_tool_menu_ = nullptr;
  QList<QAction*> toolbar_tool_actions_;
  QList<QShortcut*> tool_shortcuts_;
  QMenu* panels_menu_ = nullptr;
  QMenu* recent_configs_menu_ = nullptr;
  QMenu* delete_panel_menu_ = nullptr;
  QAction* fullscreen_action_ = nullptr;
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
