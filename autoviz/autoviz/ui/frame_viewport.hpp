/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame_viewport.hpp
 * @brief VisualizationFrame collaborator — 3D/2D viewports, backends, HUD.
 *
 * Owns the map of @ref ViewportPanelEntry keyed by dock, creates OpenGL/Ogre
 * render windows, installs floating toolbar and HUD overlays, and syncs
 * per-viewport local tools with the shared tool context.
 *
 * @see VisualizationFrame
 * @see ViewportPanelEntry
 * @see ViewportFloatingToolbar
 * @see ViewportHudOverlay
 * @see rendering::RenderWindow
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
 * @class FrameViewport
 * @brief Collaborator for @ref VisualizationFrame — viewport responsibilities.
 *
 * ## Multi-viewport
 *
 * Each 3D View dock has a @ref ViewportPanelEntry with its own render window,
 * floating toolbar, HUD, and @c local_tool_id. Only
 * @c active_viewport_dock_ feeds the global Views panel / tool context.
 *
 * ## Backends
 *
 * @ref applyRenderBackend() tears down and recreates the render window inside
 * each entry (OpenGL vs Ogre).
 *
 * @note Not a @c QObject — owned by @ref VisualizationFrame.
 *
 * @see ViewportPanelEntry
 * @see FrameSession::onRenderTick()
 */
class FrameViewport {
  friend class VisualizationFrame;
  friend class FrameLayout;
  friend class FramePanels;
  friend class FrameChrome;
  friend class FrameSession;

 public:
  /**
   * @brief Constructs viewport helpers bound to @p frame.
   * @param frame Non-owning back-pointer (must outlive this object).
   */
  explicit FrameViewport(VisualizationFrame* frame);

  /** @brief Default destructor; entries are destroyed with their docks. */
  ~FrameViewport() = default;

  FrameViewport(const FrameViewport&) = delete;
  FrameViewport& operator=(const FrameViewport&) = delete;

  /**
   * @brief ViewController of the active viewport entry.
   * @return Non-owning controller, or @c nullptr if none.
   */
  rendering::ViewController* activeViewController();

  /**
   * @brief Mutable entry for the active viewport dock.
   * @return Entry pointer, or @c nullptr if none.
   */
  ViewportPanelEntry* activeViewportEntry();

  /**
   * @brief Switches all (or active) viewports to render backend @p name.
   * @param name Backend id (@c "OpenGL" / @c "Ogre").
   */
  void applyRenderBackend(const QString& name);

  /**
   * @brief Updates the session render timer interval for @p fps.
   * @param fps Target frames per second (clamped by implementation).
   */
  void applyTargetFrameRate(int fps);

  /**
   * @brief Sets the active view controller type by name (Orbit, FPS, …).
   * @param name Controller type id.
   */
  void applyViewController(const QString& name);

  /**
   * @brief Pushes manager render settings (background, FPS hints) into @p entry.
   * @param entry Viewport entry to update.
   */
  void applyViewportEntryRenderSettings(ViewportPanelEntry& entry);

  /**
   * @brief Connects pick / tool / focus interactions for all viewport entries.
   */
  void connectViewportInteractions();

  /**
   * @brief Connects pick / tool / focus interactions for a single @p entry.
   * @param entry Viewport entry to wire.
   */
  void connectViewportInteractionsForEntry(ViewportPanelEntry& entry);

  /**
   * @brief Creates the OpenGL or Ogre render window inside @p entry.
   *
   * @param entry Target viewport entry (host widget must exist).
   * @param backend Backend name (@c "OpenGL" / @c "Ogre").
   */
  void createRenderWindowInEntry(ViewportPanelEntry& entry,
                                 const QString& backend);

  /**
   * @brief Creates the primary viewport (dock + entry + render window).
   * @param backend Initial render backend name.
   */
  void createViewport(const QString& backend);

  /**
   * @brief Creates an additional 3D View panel dock.
   *
   * @param object_name Optional fixed @c objectName; empty → unique name.
   * @return New viewport panel dock.
   */
  PanelDockWidget* createViewportPanelDock(
      const QString& object_name = QString());

  /**
   * @brief Destroys the render window (GL or Ogre) held by @p entry.
   * @param entry Entry whose render window is torn down.
   */
  void destroyRenderWindowInEntry(ViewportPanelEntry& entry);

  /**
   * @brief Lazily creates host layout / overlays when a viewport dock is shown.
   * @param dock Viewport panel dock.
   */
  void ensureViewportPanelReady(PanelDockWidget* dock);

  /**
   * @brief Invokes @p fn for every registered viewport entry.
   * @param fn Callback receiving a mutable entry reference.
   */
  void forEachViewportPanel(
      const std::function<void(ViewportPanelEntry&)>& fn);

  /**
   * @brief Installs @ref ViewportFloatingToolbar on @p entry.
   * @param entry Viewport entry to decorate.
   */
  void installViewportFloatingToolbar(ViewportPanelEntry& entry);

  /**
   * @brief Installs @ref ViewportHudOverlay on @p entry.
   * @param entry Viewport entry to decorate.
   */
  void installViewportHudOverlay(ViewportPanelEntry& entry);

  /**
   * @brief Installs expand / settings title-bar tools for @p entry.
   * @param entry Viewport entry whose dock title bar is updated.
   */
  void installViewportTitleBarToolsForEntry(ViewportPanelEntry& entry);

  /**
   * @brief Recenter-on-frame action from the floating toolbar.
   * @param dock Viewport dock that requested recenter.
   */
  void onViewportRecenterOnFrame(PanelDockWidget* dock);

  /**
   * @brief Toggles 2D (top-down ortho) vs 3D camera for @p dock.
   *
   * Saves / restores @ref ViewportPanelEntry::saved_3d_view_state.
   *
   * @param dock Viewport dock.
   */
  void onViewportToggle2dCamera(PanelDockWidget* dock);

  /**
   * @brief Pushes @p entry's @c local_tool_id into the render/tool context.
   * @param entry Viewport whose local tool becomes active.
   */
  void pushViewportLocalTool(ViewportPanelEntry& entry);

  /**
   * @brief Registers the primary 3D View dock created at startup.
   */
  void registerPrimaryViewportPanel();

  /**
   * @brief Removes and destroys the viewport entry for @p dock.
   * @param dock Viewport dock being closed / deleted.
   */
  void removeViewportPanel(PanelDockWidget* dock);

  /**
   * @brief Requests a redraw of the active (or all) viewport(s).
   */
  void requestViewportUpdate();

  /**
   * @brief Sets which viewport dock is active for Views / tools.
   * @param dock Viewport dock to activate, or @c nullptr to clear.
   */
  void setActiveViewportDock(PanelDockWidget* dock);

  /**
   * @brief Sets the per-viewport local tool id on @p dock.
   *
   * @param dock Viewport dock.
   * @param tool_id Tool registry id (e.g. @c "Measure").
   */
  void setViewportLocalTool(PanelDockWidget* dock, const std::string& tool_id);

  /**
   * @brief Syncs the shared tool context with the active viewport local tool.
   */
  void syncToolContext();

  /**
   * @brief Syncs floating-toolbar checked state with @p entry local tools.
   * @param entry Viewport entry to sync.
   */
  void syncViewportFloatingToolbarForEntry(const ViewportPanelEntry& entry);

  /**
   * @brief Syncs title-bar expand / settings tools for all viewport entries.
   */
  void syncViewportTitleBarTools();

  /**
   * @brief Syncs title-bar tools for a single @p entry.
   * @param entry Viewport entry to sync.
   */
  void syncViewportTitleBarToolsForEntry(const ViewportPanelEntry& entry);

  /**
   * @brief Toggles @p tool_id on @p dock (check → set, uncheck → Interact).
   *
   * @param dock Viewport dock.
   * @param tool_id Tool id to toggle.
   */
  void toggleViewportLocalTool(PanelDockWidget* dock,
                               const std::string& tool_id);

  /**
   * @brief Updates the cursor on the active render window for the current tool.
   */
  void updateViewportCursor();

  /**
   * @brief Looks up the mutable entry for @p dock.
   * @param dock Viewport dock key.
   * @return Entry pointer, or @c nullptr if not registered.
   */
  ViewportPanelEntry* viewportEntryForDock(PanelDockWidget* dock);

  /**
   * @brief Looks up the const entry for @p dock.
   * @param dock Viewport dock key.
   * @return Entry pointer, or @c nullptr if not registered.
   */
  const ViewportPanelEntry* viewportEntryForDock(PanelDockWidget* dock) const;

  /**
   * @brief Per-frame update: camera, tools, then render.
   * @param delta_seconds Wall time since the previous tick.
   */
  void viewportTick(float delta_seconds);

  /**
   * @brief Finishes wiring after dock + host widgets exist for @p entry.
   * @param entry Newly created or restored viewport entry.
   */
  void wireViewportPanel(ViewportPanelEntry& entry);

 private:
  /** Non-owning back-pointer to the main window. */
  VisualizationFrame* frame_ = nullptr;

  /** All viewport entries keyed by their @ref PanelDockWidget. */
  QHash<PanelDockWidget*, ViewportPanelEntry> viewport_panels_;

  /** Dock considered active for Views panel / global tools. */
  PanelDockWidget* active_viewport_dock_ = nullptr;

  /** Primary viewport dock created at startup. */
  PanelDockWidget* viewport_dock_ = nullptr;
};

}  // namespace autoviz
