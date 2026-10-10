/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame.hpp
 * @brief Top-level Autoviz main window — composition root for the UI shell.
 *
 * @ref VisualizationFrame is a thin @c QMainWindow facade. It owns the shared
 * @ref common::VisualizationManager and five collaborator objects that each
 * own a slice of state and behavior:
 *
 * - @ref FrameLayout — dock host attachment, center tiling, expand/restore
 * - @ref FramePanels — create / wire / activate feature panels and docks
 * - @ref FrameViewport — 3D/2D viewport entries, render backends, tools, HUD
 * - @ref FrameChrome — menus, toolbar, shortcuts, status / FPS readout
 * - @ref FrameSession — config I/O, record drag-and-drop, render timers
 *
 * Public methods and Qt slots on this class typically forward to the matching
 * collaborator. Collaborators are @c friend classes so they can reach each
 * other through the frame without exposing a large public surface.
 *
 * @see FrameLayout
 * @see FramePanels
 * @see FrameViewport
 * @see FrameChrome
 * @see FrameSession
 * @see common::VisualizationManager
 * @see MainPanelHost
 */

#pragma once

#include <memory>

#include <QColor>
#include <QMainWindow>
#include <QString>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/frame_chrome.hpp"
#include "autoviz/ui/frame_layout.hpp"
#include "autoviz/ui/frame_panels.hpp"
#include "autoviz/ui/frame_session.hpp"
#include "autoviz/ui/frame_viewport.hpp"
#include "autoviz/ui/panel/dock.hpp"

class QAction;
class QDragEnterEvent;
class QDragLeaveEvent;
class QDragMoveEvent;
class QDropEvent;
class QEvent;
class QKeyEvent;
class QResizeEvent;

namespace autoviz {

/**
 * @class VisualizationFrame
 * @brief Top-level Autoviz main window and composition root for the UI shell.
 *
 * ## Collaborators
 *
 * @code
 * VisualizationFrame
 *   ├── FrameLayout    (docks, center grid, expand)
 *   ├── FramePanels    (panel factory / wiring)
 *   ├── FrameViewport  (render windows, HUD, tools)
 *   ├── FrameChrome    (menus, toolbar, status)
 *   └── FrameSession   (config, DnD, timers)
 * @endcode
 *
 * ## Data flow
 *
 * - **Out:** File / Panels / View menu actions and toolbar tools forward into
 *   the collaborators; viewport redraw is driven by session render timers.
 * - **In:** Manager callbacks (background, view controller, backend, FPS)
 *   are wired in the constructor; session config restores layout and views.
 *
 * @note Collaborators are not @c QObject subclasses; slots that need
 *       @c QObject::connect live on this frame and forward into them.
 *
 * @see FrameLayout
 * @see FramePanels
 * @see FrameViewport
 * @see FrameChrome
 * @see FrameSession
 */
class VisualizationFrame : public QMainWindow {
  Q_OBJECT

  friend class FrameLayout;
  friend class FramePanels;
  friend class FrameViewport;
  friend class FrameChrome;
  friend class FrameSession;

 public:
  /**
   * @brief Constructs the Autoviz main window.
   *
   * Creates the five collaborators, builds the default dock layout via
   * @ref setupUi(), installs menus / toolbar / drop overlay, wires manager
   * callbacks (redraw, background, view controller, backend, frame rate),
   * and starts the render / refresh timers.
   *
   * @param manager Shared visualization manager (displays, tools, TF, time).
   *        Must not be null; ownership is shared with the frame.
   * @param parent Optional parent widget. Defaults to @c nullptr (top-level).
   */
  explicit VisualizationFrame(
      std::shared_ptr<common::VisualizationManager> manager,
      QWidget* parent = nullptr);

  /**
   * @brief Destroys the frame and stops render / refresh timers.
   *
   * Also disconnects the application @c aboutToQuit handler and removes the
   * installed event filter. Collaborators are destroyed with @c unique_ptr.
   */
  ~VisualizationFrame() override;

  /**
   * @brief Loads an Autoviz (or legacy RViz) session config from disk.
   *
   * Restores panel layout, visibility, view state, and panel-specific
   * settings through @ref FrameSession.
   *
   * @param path Absolute or relative path to a @c .autoviz / @c .yaml /
   *        @c .rviz file.
   * @return @c true if the config was applied successfully; @c false on I/O
   *         or parse failure (caller may show a warning dialog).
   * @see saveConfig()
   * @see applyStartupWindowState()
   */
  bool loadConfig(const QString& path);

  /**
   * @brief Saves the current session to @p path.
   *
   * Captures window geometry/state, dock layout, views, and panel configs.
   *
   * @param path Destination path (typically @c *.autoviz).
   * @return @c true on success; @c false if writing failed.
   * @see loadConfig()
   */
  bool saveConfig(const QString& path);

  /**
   * @brief Applies the window geometry / state restored from the last session.
   *
   * Called after construction when a previous config provided a saved
   * @c QByteArray window state. Safe to call when no state is stored (no-op).
   *
   * @see loadConfig()
   * @see FrameSession::applyStartupWindowState()
   */
  void applyStartupWindowState();

  /**
   * @brief Opens an Autolink record (or converts @c .bag / @c .mcap) and starts
   *        playback.
   *
   * Shows the record drop overlay flow's open dialog path when invoked from
   * the File menu; also used by drag-and-drop handlers in @ref FrameSession.
   *
   * @param path Path to @c .record, @c .bag, or @c .mcap.
   * @return @c true if the file was accepted and playback started.
   * @see FrameSession::openRecordFile()
   */
  bool openRecordFile(const QString& path);

  /**
   * @brief Pauses or resumes viewport render and refresh timers.
   *
   * Used when the window is minimized / off-screen to avoid burning GPU and
   * to keep channel processing aligned with visibility.
   *
   * @param paused @c true to stop timers; @c false to restore the target
   *        frame rate and refresh cadence.
   * @see FrameSession::syncOffscreenPause()
   */
  void setRenderingPaused(bool paused);

  /**
   * @brief Accesses the dock-layout collaborator.
   * @see FrameLayout
   */
  FrameLayout& layout() { return *layout_; }

  /**
   * @brief Accesses the panel factory / wiring collaborator.
   * @see FramePanels
   */
  FramePanels& panels() { return *panels_; }

  /**
   * @brief Accesses the viewport / render collaborator.
   * @see FrameViewport
   */
  FrameViewport& viewport() { return *viewport_; }

  /**
   * @brief Accesses the menus / toolbar / status collaborator.
   * @see FrameChrome
   */
  FrameChrome& chrome() { return *chrome_; }

  /**
   * @brief Accesses the session / config / timer collaborator.
   * @see FrameSession
   */
  FrameSession& session() { return *session_; }

  /** @copydoc layout() */
  const FrameLayout& layout() const { return *layout_; }
  /** @copydoc panels() */
  const FramePanels& panels() const { return *panels_; }
  /** @copydoc viewport() */
  const FrameViewport& viewport() const { return *viewport_; }
  /** @copydoc chrome() */
  const FrameChrome& chrome() const { return *chrome_; }
  /** @copydoc session() */
  const FrameSession& session() const { return *session_; }

  /**
   * @brief Returns a reference to the shared visualization manager.
   * @note Prefer @ref managerPtr() when a @c shared_ptr must be retained.
   */
  common::VisualizationManager& manager() { return *manager_; }

  /** @copydoc manager() */
  const common::VisualizationManager& manager() const { return *manager_; }

  /**
   * @brief Returns the @c shared_ptr held by this frame (for child panels that
   *        keep a non-owning or shared reference).
   */
  std::shared_ptr<common::VisualizationManager> managerPtr() const {
    return manager_;
  }

 signals:
  /**
   * @brief Emitted when the window enters or leaves full-screen mode.
   *
   * Connected to the Fullscreen menu action so the checkable state stays in
   * sync when fullscreen is toggled programmatically or via shortcut.
   *
   * @param hidden @c true when chrome (menu/toolbar) is considered hidden in
   *        full-screen presentation; otherwise @c false.
   * @see setFullScreen()
   * @see onToggleFullscreen()
   */
  void fullScreenChange(bool hidden);

 protected:
  /**
   * @brief Forwards window-state changes and notifies @ref FrameSession.
   *
   * Calls @c QMainWindow::changeEvent() first, then
   * @ref FrameSession::changeEvent() for off-screen pause bookkeeping.
   *
   * @param event Qt change event (window state, locale, …).
   */
  void changeEvent(QEvent* event) override;

  /**
   * @brief Lets @ref FrameSession clear hover / tool UI, then calls the base
   *        implementation.
   *
   * @param event Leave event from Qt.
   */
  void leaveEvent(QEvent* event) override;

  /**
   * @brief Dispatches key handling to @ref FrameSession (tools / shortcuts),
   *        then @c QMainWindow::keyPressEvent().
   *
   * @param event Key press event.
   */
  void keyPressEvent(QKeyEvent* event) override;

  /**
   * @brief Relays resize to @ref FrameSession (e.g. record drop overlay
   *        layout) then @c QMainWindow::resizeEvent().
   *
   * @param event Resize event.
   */
  void resizeEvent(QResizeEvent* event) override;

  /**
   * @brief Accepts Autolink record MIME payloads for drag-and-drop open.
   *
   * Implementation lives in @ref FrameSession::dragEnterEvent().
   *
   * @param event Drag-enter event.
   */
  void dragEnterEvent(QDragEnterEvent* event) override;

  /**
   * @brief Updates drop-overlay visibility while dragging a record over the
   *        window.
   *
   * @param event Drag-move event.
   * @see FrameSession::dragMoveEvent()
   */
  void dragMoveEvent(QDragMoveEvent* event) override;

  /**
   * @brief Hides the record drop overlay when the drag leaves the window.
   *
   * @param event Drag-leave event.
   * @see FrameSession::dragLeaveEvent()
   */
  void dragLeaveEvent(QDragLeaveEvent* event) override;

  /**
   * @brief Opens the dropped record / bag / mcap via @ref FrameSession.
   *
   * @param event Drop event.
   * @see openRecordFile()
   */
  void dropEvent(QDropEvent* event) override;

  /**
   * @brief Application-wide filter installed on @c qApp for focus and overlay
   *        interaction.
   *
   * @param watched Object being filtered.
   * @param event Event under consideration.
   * @return @c true if @ref FrameSession handled the event; otherwise falls
   *         back to @c QMainWindow::eventFilter().
   */
  bool eventFilter(QObject* watched, QEvent* event) override;

  /**
   * @brief Tears down Ogre/GLX surfaces before the window hierarchy is hidden.
   */
  void closeEvent(QCloseEvent* event) override;

 private slots:
  /**
   * @brief Render timer timeout: advances viewport tick / draw.
   *
   * Connected to @ref FrameSession render timer. Forwards to
   * @ref FrameSession::onRenderTick().
   */
  void onRenderTick();

  /**
   * @brief Low-frequency refresh (channel list, FPS, status).
   *
   * Connected to @ref FrameSession refresh timer (typically 1 Hz).
   */
  void onRefreshTick();

  /**
   * @brief Flush pending config / teardown work before the application exits.
   */
  void onAboutToQuit();

  /**
   * @brief File → Open Config… dialog; loads via @ref loadConfig().
   */
  void onOpenConfig();

  /**
   * @brief File → Open Record… dialog; opens playback via @ref openRecordFile().
   */
  void onOpenRecord();

  /**
   * @brief File → Save Config (uses current path or Save As when empty).
   */
  void onSaveConfig();

  /**
   * @brief File → Save Config As… and updates the recent-config list.
   */
  void onSaveConfigAs();

  /**
   * @brief Displays panel fixed-frame combo changed.
   *
   * @param frame New fixed TF frame name (may be empty while clearing).
   */
  void onFixedFrameChanged(const QString& frame);

  /**
   * @brief View → Fullscreen checkable action toggled.
   * @see setFullScreen()
   * @see fullScreenChange()
   */
  void onToggleFullscreen();

  /**
   * @brief Render backend menu: select the Ogre 1.x viewport.
   */
  void onBackendOgre();

  /**
   * @brief Captures a screenshot of the active viewport.
   */
  void onScreenshot();

  /**
   * @brief Enters or leaves full-screen presentation.
   *
   * @param full_screen @c true to show full-screen; @c false to restore.
   * @see onToggleFullscreen()
   * @see fullScreenChange()
   */
  void setFullScreen(bool full_screen);

  /**
   * @brief Toolbar tool action triggered (Select / Measure / Interact / …).
   *
   * @param action The @c QAction in the tool action group (tool id in
   *        @c QAction::data()).
   */
  void onToolTriggered(QAction* action);

  /**
   * @brief Panels → Add Panel… (or toolbar equivalent).
   */
  void onAddPanel();

  /**
   * @brief Splits @p source dock with a duplicate panel.
   *
   * @param source Dock that initiated Split right / Split down.
   * @param orientation @c Qt::Horizontal or @c Qt::Vertical split direction.
   */
  void onSplitActiveDock(PanelDockWidget* source, Qt::Orientation orientation);

  /**
   * @brief Shows (and raises) a dock by its @c objectName.
   *
   * Used by Panels menu toggles and “change panel” actions.
   *
   * @param object_name Dock object name (e.g. @c DisplaysDock).
   */
  void showPanelByObjectName(const QString& object_name);

  /**
   * @brief Deletes the currently selected / last-active deletable panel.
   */
  void onDeletePanel();

  /**
   * @brief Recent-config menu entry activated (sender holds the path).
   */
  void onRecentConfigSelected();

  /**
   * @brief Help → About Autoviz dialog.
   */
  void onHelpAbout();

  /**
   * @brief Opens the application settings dialog and applies preferences.
   */
  void onAppSettings();

  /**
   * @brief Resets docks to the built-in default layout.
   */
  void onResetDefaultLayout();

  /**
   * @brief Time panel / menu Reset: clears playback and visualization state.
   */
  void onReset();

  /**
   * @brief Toolbar control: hide or show left-side docks.
   *
   * @param hide @c true to hide the left dock area.
   */
  void onHideLeftDockToggled(bool hide);

  /**
   * @brief Toolbar control: hide or show right-side docks.
   *
   * @param hide @c true to hide the right dock area.
   */
  void onHideRightDockToggled(bool hide);

  /**
   * @brief Any registered panel dock visibility changed.
   *
   * Triggers layout retile / menu toggle sync as needed.
   *
   * @param visible New visibility of the sender dock.
   */
  void onDockPanelVisibilityChange(bool visible);

  /**
   * @brief Applies a new 3D viewport background color (from Displays or
   *        settings).
   *
   * @param color Scene clear / background color.
   */
  void applyBackgroundColor(const QColor& color);

  /**
   * @brief Pushes Views panel edits back into @ref common::VisualizationManager.
   */
  void syncViewsToManager();

  /**
   * @brief Removes a custom tool from the toolbar (Remove Tool menu).
   *
   * @param action Menu action identifying the tool to remove.
   */
  void onRemoveToolTriggered(QAction* action);

  /**
   * @brief Adds a tool from the plugin / tool catalog to the toolbar.
   */
  void onAddToolTriggered();

 private:
  /**
   * @brief Builds the initial dock / panel tree (3D view, Displays, Time, …).
   *
   * Called once from the constructor before menus and manager callbacks.
   */
  void setupUi();

  /**
   * @brief Marks the session dirty so the window title shows a modified hint.
   *
   * Invoked from panel/config change signals via a private slot wrapper so
   * @c QObject::connect can target @ref VisualizationFrame (collaborators are
   * not QObjects).
   */
  void markConfigModified();

  /** Shared display / tool / TF / time hub used by all panels and viewports. */
  std::shared_ptr<common::VisualizationManager> manager_;

  /** Dock host, center grid tiling, expand-one-panel state. */
  std::unique_ptr<FrameLayout> layout_;

  /** Feature panel docks, create/wire helpers, active-panel tracking. */
  std::unique_ptr<FramePanels> panels_;

  /** Viewport entries, OpenGL/Ogre windows, floating toolbar and HUD. */
  std::unique_ptr<FrameViewport> viewport_;

  /** Menu bar, main toolbar, tool shortcuts, status / FPS labels. */
  std::unique_ptr<FrameChrome> chrome_;

  /** Config load/save, record DnD overlay, render/refresh timers. */
  std::unique_ptr<FrameSession> session_;
};

}  // namespace autoviz
