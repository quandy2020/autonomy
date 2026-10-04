/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tool_manager.hpp
 * @brief Owns tool instances, active tool, toolbar membership, and event dispatch.
 *
 * Instantiates tools from @ref ToolRegistry, routes mouse/wheel/shortcut
 * events, and supports per-viewport local tools (e.g. Measure on a split).
 *
 * @see Tool
 * @see ToolRegistry
 * @see ToolContext
 * @see VisualizationManager::tools
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/session_config.hpp"
#include "autoviz/common/tool.hpp"

class QMouseEvent;
class QWheelEvent;

namespace autoviz {
namespace rendering {
class SceneOverlay;
}

namespace common {

/**
 * @class ToolManager
 * @brief Runtime manager for interactive viewport tools.
 *
 * ## Event routing
 *
 * Overloads that take only an event use the globally active tool. Overloads
 * with a @c tool_id dispatch to that tool for per-viewport local tools.
 */
class ToolManager {
 public:
  /**
   * @brief Constructs an empty manager (tools created on first use / context).
   */
  ToolManager();

  /**
   * @brief Destroys owned tool instances.
   */
  ~ToolManager();

  /**
   * @brief Sets the shared tool context forwarded to activate/update.
   * @param context Bundle of viewport / pick / selection services.
   */
  void setContext(ToolContext context);

  /**
   * @brief All instantiated tool ids (registry order).
   * @return Const reference to the id list.
   */
  const std::vector<std::string>& toolIds() const { return tool_ids_; }

  /**
   * @brief Id of the currently active tool.
   * @return Active tool id string.
   */
  std::string activeToolId() const { return active_tool_id_; }

  /**
   * @brief Activates the tool with @p id (deactivates the previous).
   *
   * @param id Tool id registered in this manager.
   * @return @c true if the tool exists and was activated.
   */
  bool setActiveTool(const std::string& id);

  /**
   * @brief Status text from the active tool.
   * @return Status string for the status bar.
   */
  QString activeStatusText() const;

  /**
   * @brief Mutable pointer to the active tool.
   * @return Tool pointer, or @c nullptr if none.
   */
  Tool* activeTool();

  /**
   * @brief Const pointer to the active tool.
   * @return Tool pointer, or @c nullptr if none.
   */
  const Tool* activeTool() const;

  /**
   * @brief Looks up a tool by id.
   * @param id Tool id.
   * @return Tool pointer, or @c nullptr if unknown.
   */
  Tool* toolById(const std::string& id);

  /**
   * @brief Const lookup by id.
   * @param id Tool id.
   * @return Tool pointer, or @c nullptr if unknown.
   */
  const Tool* toolById(const std::string& id) const;

  /**
   * @brief Property specs for the active tool.
   * @return Spec list for the Tools property panel.
   */
  std::vector<DisplayPropertySpec> activeToolPropertySpecs() const;

  /**
   * @brief Sets one property on the active tool.
   *
   * @param key Property key.
   * @param value New value string.
   */
  void setActiveToolProperty(const std::string& key, const std::string& value);

  /**
   * @brief Applies persisted tool property configs from a session.
   * @param configs Per-tool property maps.
   */
  void applyToolConfigs(const std::vector<ToolConfig>& configs);

  /**
   * @brief Captures current tool properties for session save.
   * @return Per-tool @ref ToolConfig list.
   */
  std::vector<ToolConfig> currentToolConfigs() const;

  /**
   * @brief Tool ids shown on the main toolbar.
   * @return Ordered toolbar id list.
   */
  const std::vector<std::string>& toolbarToolIds() const {
    return toolbar_tool_ids_;
  }

  /**
   * @brief Replaces the toolbar tool id list.
   * @param ids Ordered tool ids to show.
   */
  void setToolbarToolIds(const std::vector<std::string>& ids);

  /**
   * @brief Restores the default toolbar membership.
   */
  void resetToolbarToDefault();

  /**
   * @brief Adds a tool id to the toolbar if not already present.
   * @param id Tool id.
   * @return @c true if added.
   */
  bool addToolToToolbar(const std::string& id);

  /**
   * @brief Removes a tool id from the toolbar.
   * @param id Tool id.
   * @return @c true if removed.
   */
  bool removeToolFromToolbar(const std::string& id);

  /**
   * @brief Tool ids registered but not currently on the toolbar.
   * @return Id list for “Add Tool” menus.
   */
  std::vector<std::string> toolsNotInToolbar() const;

  /**
   * @brief Display label for a tool id.
   * @param id Tool id.
   * @return Label from registry / tool instance.
   */
  QString toolLabel(const std::string& id) const;

  /**
   * @brief Dispatches mouse press to the active tool.
   * @param event Qt mouse event.
   * @return @c true if consumed.
   */
  bool mousePressEvent(QMouseEvent* event);

  /**
   * @brief Dispatches mouse move to the active tool.
   * @param event Qt mouse event.
   * @return @c true if consumed.
   */
  bool mouseMoveEvent(QMouseEvent* event);

  /**
   * @brief Dispatches mouse release to the active tool.
   * @param event Qt mouse event.
   * @return @c true if consumed.
   */
  bool mouseReleaseEvent(QMouseEvent* event);

  /**
   * @brief Dispatches wheel to the active tool.
   * @param event Qt wheel event.
   * @return @c true if consumed.
   */
  bool wheelEvent(QWheelEvent* event);

  /**
   * @brief Dispatches mouse press to a specific tool (per-viewport local tool).
   * @param event Qt mouse event.
   * @param tool_id Target tool id.
   * @return @c true if consumed.
   */
  bool mousePressEvent(QMouseEvent* event, const std::string& tool_id);

  /**
   * @brief Dispatches mouse move to a specific tool.
   * @param event Qt mouse event.
   * @param tool_id Target tool id.
   * @return @c true if consumed.
   */
  bool mouseMoveEvent(QMouseEvent* event, const std::string& tool_id);

  /**
   * @brief Dispatches mouse release to a specific tool.
   * @param event Qt mouse event.
   * @param tool_id Target tool id.
   * @return @c true if consumed.
   */
  bool mouseReleaseEvent(QMouseEvent* event, const std::string& tool_id);

  /**
   * @brief Dispatches wheel to a specific tool.
   * @param event Qt wheel event.
   * @param tool_id Target tool id.
   * @return @c true if consumed.
   */
  bool wheelEvent(QWheelEvent* event, const std::string& tool_id);

  /**
   * @brief Calls @ref Tool::onDraw on the active tool.
   * @param scene Shared scene overlay.
   */
  void onDraw(rendering::SceneOverlay& scene);

  /**
   * @brief Draws one tool for a specific viewport (Measure is not on the shared scene).
   *
   * @param tool_id Tool to draw.
   * @param viewport_key Split panel key.
   * @param scene Overlay target for that viewport.
   */
  void drawTool(const std::string& tool_id, const std::string& viewport_key,
                rendering::SceneOverlay& scene);

  /**
   * @brief Drops per-viewport session state (e.g. Measure toggled off on a split).
   *
   * @param tool_id Tool id.
   * @param viewport_key Split panel key.
   */
  void clearToolViewportSession(const std::string& tool_id,
                                const std::string& viewport_key);

  /**
   * @brief RViz @c ToolManager::handleChar — activate by letter or toggle off.
   *
   * @param qt_key Qt key code.
   * @return @c true if a tool handled the shortcut.
   */
  bool handleShortcutKey(int qt_key);

  /**
   * @brief Default tool id restored after one-shot tools finish.
   * @return Default id (typically Move Camera / Interact).
   */
  std::string defaultToolId() const { return default_tool_id_; }

  /**
   * @brief Shortcut character registered for a tool.
   * @param id Tool id.
   * @return Shortcut char, or @c '\\0'.
   */
  char shortcutKeyForTool(const std::string& id) const;

  /**
   * @brief Whether the active tool allows viewport orbit/pan/zoom fallback.
   *
   * Only Move Camera forwards unconsumed events to viewport navigation.
   *
   * @return @c true if camera navigation should run after the tool.
   */
  bool allowsViewportNavigation() const;

  /**
   * @brief Same as @ref allowsViewportNavigation() for a specific tool id.
   * @param tool_id Tool to query.
   * @return @c true if that tool allows navigation fallback.
   */
  bool allowsViewportNavigation(const std::string& tool_id) const;

 private:
  /**
   * @brief Applies default property values to a newly created tool.
   * @param tool Tool to initialize.
   */
  void applyDefaultProperties(Tool* tool);

  std::vector<std::unique_ptr<Tool>> tools_;   /**< Owned tool instances. */
  std::vector<std::string> tool_ids_;          /**< Parallel id list. */
  std::vector<std::string> toolbar_tool_ids_;  /**< Toolbar membership. */
  std::string active_tool_id_;                 /**< Currently active id. */
  std::string default_tool_id_;                /**< Fallback after one-shot. */
  ToolContext context_;                        /**< Shared services bundle. */
};

}  // namespace common
}  // namespace autoviz
