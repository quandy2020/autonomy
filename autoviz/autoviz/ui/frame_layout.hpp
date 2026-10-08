/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame_layout.hpp
 * @brief VisualizationFrame collaborator — dock host, center tiling, expand.
 *
 * Owns attachment of main-column vs sidebar docks, Foxglove-style center
 * grid tiling via @ref MainPanelHost, expand-one-panel state, and hide
 * left/right dock areas.
 *
 * @see VisualizationFrame
 * @see MainPanelHost
 * @see PanelDockWidget
 * @see FramePanels
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

class QMainWindow;
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
 * @class FrameLayout
 * @brief Collaborator for @ref VisualizationFrame — layout responsibilities.
 *
 * ## Center vs sidebar
 *
 * - **Main panels** (3D View, Image, Plot, …) live in @ref MainPanelHost as a
 *   splitter grid so nested separators stay grippable.
 * - **Sidebar panels** (Displays, Views, Properties, …) use normal
 *   @c QMainWindow dock areas (left/right).
 * - **Bottom** Time dock is kept at the bottom via @ref ensureTimeDockAtBottom().
 *
 * ## Expand / tile
 *
 * Expand hides sibling main panels and records
 * @c pre_expand_visible_main_panels_ for restore. Tiling is coalesced through
 * @ref scheduleTileCenterPanels() / @c center_tile_epoch_.
 *
 * @note Not a @c QObject — owned by @ref VisualizationFrame.
 *
 * @see MainPanelHost
 * @see PanelRoleMain()
 * @see PanelRoleSidebar()
 */
class FrameLayout {
  friend class VisualizationFrame;
  friend class FramePanels;
  friend class FrameViewport;
  friend class FrameChrome;
  friend class FrameSession;

 public:
  /**
   * @brief Constructs layout helpers bound to @p frame.
   *
   * @param frame Non-owning back-pointer (must outlive this object).
   */
  explicit FrameLayout(VisualizationFrame* frame);

  /** @brief Default destructor; host widgets are parented to the frame. */
  ~FrameLayout() = default;

  FrameLayout(const FrameLayout&) = delete;
  FrameLayout& operator=(const FrameLayout&) = delete;

  /**
   * @brief Returns the dock that should receive Split right / Split down.
   *
   * Prefers @c last_active_dock_ when it is a main panel; otherwise the
   * active viewport dock.
   *
   * @return Dock suitable for split, or @c nullptr if none.
   */
  PanelDockWidget* activeDockForSplit() const;

  /**
   * @brief Adds a main-column dock to the frame and configures it.
   *
   * @param dock Main panel dock (viewport, image, plot, …).
   * @param area Preferred dock area when floating / re-attaching (default left).
   * @see configureMainPanelDock()
   * @see ensureMainPanelDockAttached()
   */
  void addMainPanelDock(PanelDockWidget* dock,
                        Qt::DockWidgetArea area = Qt::LeftDockWidgetArea);

  /**
   * @brief Adds a sidebar dock and configures allowed areas.
   *
   * @param dock Sidebar panel dock.
   * @param area Left or right dock area.
   * @see configureSidebarDock()
   */
  void addSidebarDock(PanelDockWidget* dock, Qt::DockWidgetArea area);

  /**
   * @brief Applies the built-in Autoviz default dock arrangement.
   * @see applyFoxgloveDefaultLayout()
   * @see onResetDefaultLayout path in @ref FrameSession
   */
  void applyDefaultDockLayout();

  /**
   * @brief Applies a Foxglove-inspired default layout (main + sidebars).
   */
  void applyFoxgloveDefaultLayout();

  /**
   * @brief Applies default placement for main-column panels only.
   */
  void applyMainPanelDefaultLayout();

  /**
   * @brief Replaces the content of @p source with the panel type identified by
   *        @p target_object_name (Change panel).
   *
   * @param source Dock whose content is swapped.
   * @param target_object_name Catalog / object name of the replacement type.
   */
  void changePanelInDock(PanelDockWidget* source,
                         const QString& target_object_name);

  /**
   * @brief Common dock flags (floatable, closable, movable) for flexible docks.
   *
   * @param dock Dock to configure.
   */
  void configureFlexibleDock(PanelDockWidget* dock);

  /**
   * @brief Main-panel-specific dock flags and expand tracking hooks.
   *
   * @param dock Main panel dock.
   * @see wireMainPanelExpandTracking()
   */
  void configureMainPanelDock(PanelDockWidget* dock);

  /**
   * @brief Sidebar-specific allowed areas and features.
   *
   * @param dock Sidebar dock.
   * @param area Preferred dock area.
   */
  void configureSidebarDock(PanelDockWidget* dock, Qt::DockWidgetArea area);

  /**
   * @brief Default sidebar area for a dock type (Displays left, Views right, …).
   *
   * @param dock Dock to query (may use objectName / role).
   * @return Preferred @c Qt::DockWidgetArea.
   */
  Qt::DockWidgetArea defaultSidebarArea(const PanelDockWidget* dock) const;

  /**
   * @brief Returns the @c QMainWindow that should parent @p dock.
   *
   * Main panels use @ref MainPanelHost; sidebars use the visualization frame.
   *
   * @param dock Panel dock.
   * @return Host window, or @c nullptr if not yet set up.
   */
  QMainWindow* dockHostForPanel(const PanelDockWidget* dock) const;

  /**
   * @brief Ensures a main panel is attached to @ref MainPanelHost.
   *
   * @param dock Main panel dock.
   * @param area Fallback area if the host is unavailable.
   */
  void ensureMainPanelDockAttached(
      PanelDockWidget* dock,
      Qt::DockWidgetArea area = Qt::LeftDockWidgetArea);

  /**
   * @brief Ensures a sidebar dock is added to the frame dock area.
   *
   * @param dock Sidebar dock.
   */
  void ensureSidebarDockAttached(PanelDockWidget* dock);

  /**
   * @brief Forces the Time dock to the bottom area after layout changes.
   */
  void ensureTimeDockAtBottom();

  /**
   * @brief Expands @p dock to fill the center (hides sibling main panels).
   *
   * @param dock Main panel to expand.
   * @see restoreExpandedMainPanel()
   * @see syncMainPanelExpandUi()
   */
  void expandPanelDock(PanelDockWidget* dock);

  /**
   * @brief Implementation of hide/show for a dock area.
   *
   * @param area Left or right dock area.
   * @param hide @c true to hide; @c false to restore.
   */
  void hideDockImpl(Qt::DockWidgetArea area, bool hide);

  /**
   * @brief Hides or shows the left dock area.
   * @param hide @c true to hide.
   */
  void hideLeftDock(bool hide);

  /**
   * @brief Hides or shows the right dock area.
   * @param hide @c true to hide.
   */
  void hideRightDock(bool hide);

  /**
   * @brief @c true when @a area still has a visible, docked sidebar panel.
   */
  bool sidebarAreaHasVisibleDock(Qt::DockWidgetArea area) const;

  /**
   * @brief @c true if any main panel is currently floating outside the host.
   */
  bool isAnyMainPanelFloating() const;

  /**
   * @brief @c true while a center-dock drag should suppress auto-tiling.
   */
  bool isCenterDockInteractionBlocked() const;

  /**
   * @brief @c true if @p dock is classified as a main-column panel.
   * @param dock Dock to test.
   */
  bool isMainPanel(const PanelDockWidget* dock) const;

  /**
   * @brief @c true if @p dock is pinned into the center host (not sidebar).
   * @param dock Dock to test.
   */
  bool isMainPanelPinnedToCenter(const PanelDockWidget* dock) const;

  /**
   * @brief @c true when the Property Inspector dock is visible.
   */
  bool isPropertyInspectorVisible() const;

  /**
   * @brief Slot path: toolbar hide-left toggled.
   * @param hide Checked state from the toolbar action.
   */
  void onHideLeftDockToggled(bool hide);

  /**
   * @brief Slot path: toolbar hide-right toggled.
   * @param hide Checked state from the toolbar action.
   */
  void onHideRightDockToggled(bool hide);

  /**
   * @brief Duplicates @p source and splits it in @p orientation.
   *
   * @param source Dock requesting the split.
   * @param orientation Horizontal (side-by-side) or Vertical (stacked).
   * @see MainPanelHost::splitPanel()
   * @see FramePanels::duplicatePanelDock()
   */
  void onSplitActiveDock(PanelDockWidget* source, Qt::Orientation orientation);

  /**
   * @brief Returns docks in a stable order for session capture / menus.
   */
  QList<PanelDockWidget*> orderedDockWidgets() const;

  /**
   * @brief Raises Displays in the left sidebar (swaps with Properties when
   *        needed).
   */
  void raiseLeftSidebarDisplays();

  /**
   * @brief Raises Property Inspector in the left sidebar.
   */
  void raiseLeftSidebarProperties();

  /**
   * @brief Restores left/right hide state after fullscreen or layout reset.
   */
  void restoreDockHideState();

  /**
   * @brief Restores main panels hidden by @ref expandPanelDock().
   */
  void restoreExpandedMainPanel();

  /**
   * @brief Coalesces a center retile onto the next event-loop turn.
   * @see tileCenterPanels()
   */
  void scheduleTileCenterPanels();

  /**
   * @brief Creates the central container widget that holds @ref MainPanelHost.
   */
  void setupCentralContainer();

  /**
   * @brief Instantiates and installs @ref MainPanelHost as the center host.
   */
  void setupMainPanelHost();

  /**
   * @brief Shows and raises the dock named @p object_name.
   *
   * @param object_name Dock @c objectName (e.g. @c DisplaysDock).
   */
  void showPanelByObjectName(const QString& object_name);

  /**
   * @brief Shows or hides the Property Inspector dock.
   *
   * @param visible Desired visibility.
   */
  void showPropertyInspector(bool visible);

  /**
   * @brief Syncs center host membership with current main-panel visibility.
   */
  void syncCenterLayout();

  /**
   * @brief Updates expand-button checked state across main panels.
   *
   * @param expanded_dock Currently expanded dock, or @c nullptr if none.
   */
  void syncMainPanelExpandUi(PanelDockWidget* expanded_dock);

  /**
   * @brief Immediately retile visible main panels into the splitter grid.
   * @see MainPanelHost::tilePanels()
   */
  void tileCenterPanels();

  /**
   * @brief Connects expand / activation signals for a main panel dock.
   *
   * @param dock Main panel dock to track.
   */
  void wireMainPanelExpandTracking(PanelDockWidget* dock);

  /**
   * @brief Handles any registered dock visibility change (retile / menus).
   *
   * @param visible New visibility of the sender dock.
   */
  void onDockPanelVisibilityChange(bool visible);

 private:
  /** Non-owning back-pointer to the main window. */
  VisualizationFrame* frame_ = nullptr;

  /** Nested host for main-column splitter tiling. */
  MainPanelHost* main_panel_host_ = nullptr;

  /** Main panel currently expanded to fill the center, or @c nullptr. */
  PanelDockWidget* expanded_main_panel_dock_ = nullptr;

  /** Object names of main panels visible before the last expand. */
  QStringList pre_expand_visible_main_panels_;

  /** Re-entrancy guard while @ref tileCenterPanels() runs. */
  bool center_tiling_ = false;

  /** @c true when a coalesced retile is scheduled. */
  bool center_tile_pending_ = false;

  /** Suppress auto-tile during programmatic attach / drag. */
  bool suppress_center_tile_ = false;

  /** User has manually arranged center splitters; skip equalize. */
  bool center_manual_layout_ = false;

  /** Nested drag depth for center docks (blocks tile while > 0). */
  int center_dock_drag_count_ = 0;

  /** Re-entrancy guard in @ref ensureMainPanelDockAttached(). */
  bool ensuring_main_panel_attach_ = false;

  /** Epoch bumped to invalidate stale scheduled tiles. */
  int center_tile_epoch_ = 0;

  /** Left sidebar currently showing Properties instead of Displays. */
  bool left_sidebar_shows_properties_ = false;

  /**
   * User closed Displays (title-bar close or Panels menu). Viewport activation
   * must not call show() again until the user reopens it.
   */
  bool displays_closed_by_user_ = false;

  /** Last dock activated by title-bar click (for split / delete). */
  PanelDockWidget* last_active_dock_ = nullptr;
};

}  // namespace autoviz
