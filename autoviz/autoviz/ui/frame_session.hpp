/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame_session.hpp
 * @brief VisualizationFrame collaborator — config I/O, record DnD, timers.
 *
 * Owns session path / dirty state, recent configs, @ref RecordDropOverlay,
 * render and refresh @c QTimer%s, and File / View / Help action handlers that
 * mutate session or application-wide settings.
 *
 * @see VisualizationFrame
 * @see RecordDropOverlay
 * @see AppSettingsDialog
 * @see common::SessionConfig
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <QHash>
#include <QList>
#include <QObject>
#include <QPointer>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QElapsedTimer>

#include <QColor>
#include <QWidget>
#include <QVector3D>

#include "autoviz/common/selection.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/viewport_panel.hpp"
#include "autoviz/ui/app/preferences.hpp"
#include "autoviz/rendering/view_controller.hpp"

class QLabel;
class QMenu;
class QToolBar;
class QToolButton;
class QAction;
class QActionGroup;
class QShortcut;
class QMimeData;
class QDragEnterEvent;
class QDragMoveEvent;
class QDragLeaveEvent;
class QDropEvent;
class QKeyEvent;
class QResizeEvent;
class QEvent;

namespace autoviz {

class VisualizationFrame;
struct AppSettingsResult;
struct AppUiPreferences;
class MainPanelHost;
class DisplaysPanel;
class TimePanel;
class ViewsPanel;
class ToolPropertiesPanel;
class SelectionPanel;
class RawMessagesPanel;
class ChannelsPanel;
class PropertyInspectorPanel;
class TfTreePanel;
namespace plot { class PlotPanel; }
namespace image { class ImagePanel; }
namespace teleop { class TeleopPanel; }
namespace publish_panel { class PublishPanel; }
namespace map { class MapPanel; }
namespace service_panel { class ServicePanel; }
namespace channel_graph { class ChannelGraphPanel; }
namespace rendering { class ViewController; class OgreRenderWindow; }

/**
 * @class FrameSession
 * @brief Collaborator for @ref VisualizationFrame — session responsibilities.
 *
 * ## Timers
 *
 * - @c render_timer_ — drives @ref onRenderTick() at the target FPS
 * - @c refresh_timer_ — ~1 Hz channel list / FPS / status refresh
 *
 * Both pause when the window is minimized or the app is inactive
 * (@ref syncOffscreenPause()).
 *
 * ## Config
 *
 * @ref loadConfig() / @ref saveConfig() serialize window state, docks, views,
 * and panel-specific configs. Dirty state updates the window title via
 * @ref updateWindowTitle().
 *
 * @note Not a @c QObject — timers are members; slots live on
 *       @ref VisualizationFrame and forward here.
 *
 * @see VisualizationFrame::loadConfig()
 * @see RecordDropOverlay
 */
class FrameSession {
  friend class VisualizationFrame;
  friend class FrameLayout;
  friend class FramePanels;
  friend class FrameViewport;
  friend class FrameChrome;

 public:
  /**
   * @brief Constructs session helpers bound to @p frame.
   * @param frame Non-owning back-pointer (must outlive this object).
   */
  explicit FrameSession(VisualizationFrame* frame);

  /** @brief Default destructor; stops are handled by the frame destructor. */
  ~FrameSession() = default;

  FrameSession(const FrameSession&) = delete;
  FrameSession& operator=(const FrameSession&) = delete;

  /**
   * @brief Pushes the captured current view onto the active ViewController.
   * @see captureCurrentView()
   */
  void applyCurrentView();

  /**
   * @brief Applies panel show/hide flags from the loaded session config.
   */
  void applyPanelVisibility();

  /**
   * @brief Rebinds tool / app shortcuts from preference map @p shortcuts.
   * @param shortcuts Id → key sequence map from @ref AppUiPreferences.
   */
  void applyShortcutPreferences(const QHash<QString, QKeySequence>& shortcuts);

  /**
   * @brief Restores saved @c QMainWindow geometry / state after construction.
   * @see VisualizationFrame::applyStartupWindowState()
   */
  void applyStartupWindowState();

  /**
   * @brief Applies settings-dialog results (backend, FPS, docks, language, …).
   *
   * @param settings Values accepted from @ref AppSettingsDialog.
   * @param previous_ui Prior UI prefs (for detecting language / HUD changes).
   */
  void applyUiPreferences(const AppSettingsResult& settings,
                          const AppUiPreferences& previous_ui);

  /**
   * @brief Snapshots the active camera into session view state.
   */
  void captureCurrentView();

  /**
   * @brief Snapshots window geometry, dock state, and layout into session.
   */
  void captureWindowLayout();

  /**
   * @brief Handles window-state changes (minimize → pause rendering).
   * @param event Qt change event forwarded from the frame.
   */
  void changeEvent(QEvent* event);

  /**
   * @brief Clears the config-modified flag and refreshes the title.
   */
  void clearConfigModified();

  /**
   * @brief Connects panel/config signals to @ref markConfigModified().
   */
  void connectConfigModifiedSignals();

  /**
   * @brief Accepts record MIME types on drag enter.
   * @param event Drag-enter event from the frame.
   */
  void dragEnterEvent(QDragEnterEvent* event);

  /**
   * @brief Hides the drop overlay when the drag leaves.
   * @param event Drag-leave event from the frame.
   */
  void dragLeaveEvent(QDragLeaveEvent* event);

  /**
   * @brief Updates overlay visibility while dragging over the window.
   * @param event Drag-move event from the frame.
   */
  void dragMoveEvent(QDragMoveEvent* event);

  /**
   * @brief Opens the dropped record file.
   * @param event Drop event from the frame.
   * @see openRecordFile()
   */
  void dropEvent(QDropEvent* event);

  /**
   * @brief Application event filter (focus / overlay interaction).
   *
   * @param watched Filtered object.
   * @param event Event under consideration.
   * @return @c true if consumed; @c false to propagate.
   */
  bool eventFilter(QObject* watched, QEvent* event);

  /**
   * @brief Inspects @p mime for Autolink record paths.
   *
   * @param mime Drag MIME data.
   * @param drop When @c true, opens the file; when @c false, only validates.
   * @return @c true if the MIME contained an acceptable record payload.
   */
  bool handleRecordMime(const QMimeData* mime, bool drop);

  /**
   * @brief Tool / shortcut key handling forwarded from the frame.
   * @param event Key press event.
   */
  void keyPressEvent(QKeyEvent* event);

  /**
   * @brief Positions @c record_drop_overlay_ over the frame client area.
   */
  void layoutRecordDropOverlay();

  /**
   * @brief Clears hover / transient tool UI when the pointer leaves.
   * @param event Leave event from the frame.
   */
  void leaveEvent(QEvent* event);

  /**
   * @brief Loads session config from @p path and applies layout / views.
   *
   * @param path Config file path (@c .autoviz / @c .yaml / @c .rviz).
   * @return @c true on success.
   * @see saveConfig()
   */
  bool loadConfig(const QString& path);

  /**
   * @brief Sets the dirty flag and updates the window title.
   */
  void markConfigModified();

  /**
   * @brief Inserts @p path at the front of the recent-config list.
   * @param path Absolute config path.
   */
  void markRecentConfig(const QString& path);

  /**
   * @brief Opens @ref AppSettingsDialog and applies accepted values.
   */
  void onAppSettings();

  /**
   * @brief Switches the active viewport to the Ogre backend.
   */
  void onBackendOgre();

  /**
   * @brief Shows the About Autoviz dialog.
   */
  void onHelpAbout();

  /**
   * @brief File → Open Config… dialog then @ref loadConfig().
   */
  void onOpenConfig();

  /**
   * @brief File → Open Record… dialog then @ref openRecordFile().
   */
  void onOpenRecord();

  /**
   * @brief Loads the config path stored on the sender recent-menu action.
   */
  void onRecentConfigSelected();

  /**
   * @brief Resets docks to the built-in default layout.
   */
  void onResetDefaultLayout();

  /**
   * @brief Saves to @c config_path_, or Save As when empty.
   */
  void onSaveConfig();

  /**
   * @brief Save Config As… dialog; updates @c config_path_ and recents.
   */
  void onSaveConfigAs();

  /**
   * @brief Captures a screenshot of the active viewport to disk.
   */
  void onScreenshot();

  /**
   * @brief Toggles full-screen from the View menu / shortcut.
   * @see setFullScreen()
   */
  void onToggleFullscreen();

  /**
   * @brief Opens @p path as an Autolink record (converting bag/mcap if needed).
   *
   * @param path Path to @c .record, @c .bag, or @c .mcap.
   * @return @c true if playback started.
   */
  bool openRecordFile(const QString& path);

  /**
   * @brief Relays resize to overlay layout.
   * @param event Resize event from the frame.
   */
  void resizeEvent(QResizeEvent* event);

  /**
   * @brief Restores per-panel layout blobs from session config.
   */
  void restorePanelLayouts();

  /**
   * @brief Restores window geometry / dock state from session config.
   */
  void restoreWindowLayout();

  /**
   * @brief Writes the current session to @p path.
   *
   * @param path Destination path.
   * @return @c true on success.
   * @see loadConfig()
   */
  bool saveConfig(const QString& path);

  /**
   * @brief Enters or leaves full-screen presentation.
   * @param full_screen Desired full-screen state.
   */
  void setFullScreen(bool full_screen);

  /**
   * @brief Starts or stops render / refresh timers.
   * @param paused @c true to stop timers.
   */
  void setRenderingPaused(bool paused);

  /**
   * @brief Creates and parents @ref RecordDropOverlay (initially hidden).
   */
  void setupRecordDropOverlay();

  /**
   * @brief Shows or hides the record drop overlay.
   * @param visible Desired visibility.
   */
  void showRecordDropOverlay(bool visible);

  /**
   * @brief Pauses rendering when minimized / inactive; resumes otherwise.
   */
  void syncOffscreenPause();

  /**
   * @brief Checks the matching backend menu action for @p name.
   * @param name Backend id (@c "OpenGL" / @c "Ogre").
   */
  void syncRenderBackendMenu(const QString& name);

  /**
   * @brief Pulls view controller / saved views from the manager into the UI.
   */
  void syncViewsFromManager();

  /**
   * @brief Rebuilds the Recent Configs submenu from @c recent_configs_.
   */
  void updateRecentConfigMenu();

  /**
   * @brief Pushes selection entries into the Selection panel.
   * @param entries Current selection set from the manager.
   */
  void updateSelectionPanel(const std::vector<common::SelectionEntry>& entries);

  /**
   * @brief Updates the window title (app name, config path, dirty marker).
   */
  void updateWindowTitle();

  /**
   * @brief Applies viewport background color to all render windows.
   * @param color Scene clear color.
   */
  void applyBackgroundColor(const QColor& color);

  /**
   * @brief Flush / teardown hook before application quit.
   */
  void onAboutToQuit();

  /**
   * @brief Fixed-frame change from Displays / settings (may be a no-op body).
   * @param frame New fixed TF frame name.
   */
  void onFixedFrameChanged(const QString& /*frame*/);

  /**
   * @brief Low-frequency tick: channel list, FPS, status bar.
   */
  void onRefreshTick();

  /**
   * @brief High-frequency tick: @ref FrameViewport::viewportTick() + draw.
   */
  void onRenderTick();

  /**
   * @brief Global Reset: clears playback and visualization state.
   */
  void onReset();

  /**
   * @brief Pushes Views panel edits into the visualization manager.
   */
  void syncViewsToManager();

 private:
  /**
   * @brief Creates missing main panels referenced by VisiblePanels / mosaic.
   *
   * Split 3D Views (@c ViewportDock_2, …) are not in ImagePanels/PlotPanels;
   * they must exist before @ref restoreWindowLayout applies the mosaic.
   */
  void ensureSessionMainPanels();

  /**
   * @brief Activates a visible 3D View after layout restore, then reapplies
   *        @c CurrentView onto that controller.
   */
  void activateRestoredViewportAndView();

  /** Non-owning back-pointer to the main window. */
  VisualizationFrame* frame_ = nullptr;

  /** Full-window drop hint for Autolink records (may be @ref RecordDropOverlay). */
  QWidget* record_drop_overlay_ = nullptr;

  /** MRU list of config paths (capped by @ref kRecentConfigCount). */
  QStringList recent_configs_;

  /** Drives viewport rendering at the target frame rate. */
  QTimer render_timer_;

  /** ~1 Hz UI refresh (channels, FPS, status). */
  QTimer refresh_timer_;

  /** Measures wall time between render ticks for delta seconds. */
  QElapsedTimer render_elapsed_;

  /** Current session config path (empty until first save / load). */
  QString config_path_;

  /** @c true when unsaved changes exist. */
  bool config_modified_ = false;

  /** Suppress dirty marking during programmatic load/apply. */
  bool suppress_config_modified_ = false;

  /** Application is inactive (pause rendering). */
  bool app_inactive_ = false;

  /** Maximum entries kept in @c recent_configs_. */
  static constexpr int kRecentConfigCount = 10;
};

}  // namespace autoviz
