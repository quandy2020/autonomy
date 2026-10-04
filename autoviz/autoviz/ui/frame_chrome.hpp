/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame_chrome.hpp
 * @brief VisualizationFrame collaborator — menus, toolbar, shortcuts, status.
 *
 * Owns the menu bar, main toolbar (tools + layout controls), tool shortcuts,
 * Panels menu toggles, recent-config submenu, and status / FPS labels.
 * Methods are invoked from @ref VisualizationFrame slots and from other
 * collaborators via friendship.
 *
 * @see VisualizationFrame
 * @see FrameLayout
 * @see FramePanels
 * @see FrameViewport
 * @see FrameSession
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
#include <QVector3D>

#include "autoviz/common/selection.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/viewport_panel.hpp"
#include "autoviz/ui/app/preferences.hpp"

class QWidget;
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
namespace rendering { class ViewController; class RenderWindow; class OgreRenderWindow; }

/**
 * @class FrameChrome
 * @brief Collaborator for @ref VisualizationFrame — chrome responsibilities.
 *
 * ## Owned UI
 *
 * - Menu bar (File / Panels / View / Help) and recent-config submenu
 * - Main toolbar: tool action group, Add Panel, hide left/right docks
 * - Tool keyboard shortcuts (@c QShortcut list)
 * - Status bar labels (hint text + FPS)
 *
 * ## Non-ownership
 *
 * Does not own docks or panels; registers menu toggles against docks created
 * by @ref FramePanels / @ref FrameLayout.
 *
 * @note Not a @c QObject — lifetime is tied to @ref VisualizationFrame via
 *       @c unique_ptr. Friend access reaches @c frame_ and sibling
 *       collaborators.
 *
 * @see VisualizationFrame::chrome()
 * @see IconLoader
 * @see CreatePanelContextMenu()
 */
class FrameChrome {
  friend class VisualizationFrame;
  friend class FrameLayout;
  friend class FramePanels;
  friend class FrameViewport;
  friend class FrameSession;

 public:
  /**
   * @brief Constructs chrome helpers bound to @p frame.
   *
   * Does not build menus/toolbar yet — call @ref setupMenu(),
   * @ref setupToolbar(), @ref setupStatusBar() from the frame constructor.
   *
   * @param frame Non-owning back-pointer to the main window (must outlive
   *        this object).
   */
  explicit FrameChrome(VisualizationFrame* frame);

  /** @brief Default destructor; Qt children are parented to the frame. */
  ~FrameChrome() = default;

  FrameChrome(const FrameChrome&) = delete;
  FrameChrome& operator=(const FrameChrome&) = delete;

  /**
   * @brief Marks the toolbar tool matching @p tool_id as checked.
   *
   * @param tool_id Tool registry id (e.g. @c "Interact", @c "Select").
   * @see syncToolbarToActiveTool()
   */
  void applyActiveTool(const std::string& tool_id);

  /**
   * @brief Populates the menu bar structure (File / Panels / View / Help).
   *
   * Creates actions and wires them to @ref VisualizationFrame slots.
   * @see setupMenu()
   */
  void configureMenuBar();

  /**
   * @brief Installs standard title-bar tools (split / change / expand) on
   *        @p dock.
   *
   * @param dock Panel dock receiving Foxglove-style title tools.
   * @see CreateStandardPanelTitleTools()
   * @see makePanelContextMenuCallbacks()
   */
  void installStandardPanelTitleTools(PanelDockWidget* dock);

  /**
   * @brief Builds context-menu callbacks for Change / Split / Expand / Remove
   *        on @p dock.
   *
   * @param dock Dock the menu operates on.
   * @return Callback bundle consumed by @ref CreatePanelContextMenu() and
   *         title-bar tool factories.
   */
  PanelContextMenuCallbacks makePanelContextMenuCallbacks(PanelDockWidget* dock);

  /**
   * @brief Opens Add Panel dialog / menu flow (toolbar or Panels menu).
   * @see FramePanels::onAddPanel()
   */
  void onAddPanel();

  /**
   * @brief Adds a tool from the catalog to the toolbar (Add Tool action).
   */
  void onAddToolTriggered();

  /**
   * @brief Removes a custom tool identified by @p action from the toolbar.
   *
   * @param action Remove-Tool submenu action (tool id in action data).
   */
  void onRemoveToolTriggered(QAction* action);

  /**
   * @brief Activates the tool whose toolbar action was triggered.
   *
   * @param action Tool action in @c tool_action_group_ (id in @c data()).
   */
  void onToolTriggered(QAction* action);

  /**
   * @brief Rebuilds checkable Panels menu entries from registered docks.
   * @see registerPanelMenuToggle()
   */
  void rebuildPanelsMenuToggles();

  /**
   * @brief Tears down and rebuilds the main toolbar contents.
   *
   * Used after tool add/remove or preference changes.
   */
  void rebuildToolbar();

  /**
   * @brief Registers a checkable Panels-menu toggle for @p dock.
   *
   * @param dock Dock whose visibility is mirrored by the menu action.
   */
  void registerPanelMenuToggle(PanelDockWidget* dock);

  /**
   * @brief Wires all known docks into the Panels menu toggle list.
   */
  void registerPanelToggleActions();

  /**
   * @brief Full menu construction entry (calls @ref configureMenuBar() and
   *        related submenu setup).
   */
  void setupMenu();

  /**
   * @brief Creates status and FPS labels on the main window status bar.
   */
  void setupStatusBar();

  /**
   * @brief Installs @c QShortcut instances for built-in tools (from
   *        preferences / defaults).
   */
  void setupToolShortcuts();

  /**
   * @brief Creates the main toolbar, tool group, and layout controls.
   * @see setupToolbarLayoutControls()
   */
  void setupToolbar();

  /**
   * @brief Adds Add Panel button and hide-left / hide-right dock actions.
   */
  void setupToolbarLayoutControls();

  /**
   * @brief Syncs checkable toolbar state with the manager's active tool.
   */
  void syncActiveToolUi();

  /**
   * @brief Updates hide-left / hide-right toolbar button checked state from
   *        current dock visibility.
   */
  void syncToolbarLayoutControls();

  /**
   * @brief Checks the toolbar action matching the active tool id.
   */
  void syncToolbarToActiveTool();

  /**
   * @brief Refreshes channel-related status / menu state (1 Hz path).
   */
  void updateChannelList();

  /**
   * @brief Recomputes FPS from @c frame_count_ / elapsed wall time and updates
   *        @c fps_label_.
   */
  void updateFps();

  /**
   * @brief Refreshes the status hint label (selection, mode, etc.).
   */
  void updateStatusBar();

 private:
  /** Non-owning back-pointer to the main window. */
  VisualizationFrame* frame_ = nullptr;

  /** Main window toolbar hosting tools and layout controls. */
  QToolBar* tool_bar_ = nullptr;

  /** Expanding spacer between tool group and layout controls. */
  QWidget* toolbar_spacer_ = nullptr;

  /** Toolbar “Add Panel” button (opens @c add_panel_menu_). */
  QToolButton* toolbar_add_panel_button_ = nullptr;

  /** Popup catalog for adding panels from the toolbar. */
  QMenu* add_panel_menu_ = nullptr;

  /** Hide/show left dock area (checkable). */
  QAction* toolbar_toggle_left_dock_action_ = nullptr;

  /** Hide/show right dock area (checkable). */
  QAction* toolbar_toggle_right_dock_action_ = nullptr;

  /** Exclusive group for Select / Interact / Measure / … tools. */
  QActionGroup* tool_action_group_ = nullptr;

  /** Opens the add-tool dialog / catalog. */
  QAction* add_tool_action_ = nullptr;

  /** Submenu listing removable custom tools. */
  QMenu* remove_tool_menu_ = nullptr;

  /** All tool actions currently on the toolbar. */
  QList<QAction*> toolbar_tool_actions_;

  /** Keyboard shortcuts mirroring toolbar tools. */
  QList<QShortcut*> tool_shortcuts_;

  /** Top-level Panels menu. */
  QMenu* panels_menu_ = nullptr;

  /** Panels → Add Panel… action. */
  QAction* add_panel_action_ = nullptr;

  /** Checkable visibility toggles in the Panels menu (weak refs). */
  QList<QPointer<QAction>> panels_menu_toggle_actions_;

  /** File → Recent Configs submenu. */
  QMenu* recent_configs_menu_ = nullptr;

  /** Panels → Delete Panel submenu. */
  QMenu* delete_panel_menu_ = nullptr;

  /** View → Fullscreen checkable action. */
  QAction* fullscreen_action_ = nullptr;

  /** File → Open Config…. */
  QAction* open_config_action_ = nullptr;

  /** File → Open Record…. */
  QAction* open_record_action_ = nullptr;

  /** File → Save Config. */
  QAction* save_config_action_ = nullptr;

  /** File → Save Config As…. */
  QAction* save_config_as_action_ = nullptr;

  /** File → Quit. */
  QAction* quit_action_ = nullptr;

  /** View → Third-person / Orbit style camera. */
  QAction* view_third_person_action_ = nullptr;

  /** View → FPS camera. */
  QAction* view_fps_action_ = nullptr;

  /** View → Backend → OpenGL. */
  QAction* backend_opengl_action_ = nullptr;

  /** View → Backend → Ogre (when enabled at build time). */
  QAction* backend_ogre_action_ = nullptr;

  /** Whether the toolbar is currently shown (fullscreen may hide it). */
  bool toolbar_visible_ = true;

  /** Left status hint label. */
  QLabel* status_label_ = nullptr;

  /** Right-side FPS readout. */
  QLabel* fps_label_ = nullptr;

  /** Cached status hint text (selection / mode). */
  QString status_hint_;

  /** Frames counted since @c last_fps_calc_. */
  int frame_count_ = 0;

  /** Wall-clock stamp for FPS calculation. */
  std::chrono::steady_clock::time_point last_fps_calc_;
};

}  // namespace autoviz
