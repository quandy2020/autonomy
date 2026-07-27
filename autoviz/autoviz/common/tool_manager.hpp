/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

class ToolManager {
 public:
  ToolManager();
  ~ToolManager();

  void setContext(ToolContext context);
  const std::vector<std::string>& toolIds() const { return tool_ids_; }
  std::string activeToolId() const { return active_tool_id_; }
  bool setActiveTool(const std::string& id);
  QString activeStatusText() const;

  Tool* activeTool();
  const Tool* activeTool() const;
  Tool* toolById(const std::string& id);
  const Tool* toolById(const std::string& id) const;
  std::vector<DisplayPropertySpec> activeToolPropertySpecs() const;
  void setActiveToolProperty(const std::string& key, const std::string& value);

  void applyToolConfigs(const std::vector<ToolConfig>& configs);
  std::vector<ToolConfig> currentToolConfigs() const;

  const std::vector<std::string>& toolbarToolIds() const {
    return toolbar_tool_ids_;
  }
  void setToolbarToolIds(const std::vector<std::string>& ids);
  void resetToolbarToDefault();
  bool addToolToToolbar(const std::string& id);
  bool removeToolFromToolbar(const std::string& id);
  std::vector<std::string> toolsNotInToolbar() const;
  QString toolLabel(const std::string& id) const;

  bool mousePressEvent(QMouseEvent* event);
  bool mouseMoveEvent(QMouseEvent* event);
  bool mouseReleaseEvent(QMouseEvent* event);
  bool wheelEvent(QWheelEvent* event);
  void onDraw(rendering::SceneOverlay& scene);
  /** RViz ToolManager::handleChar — activate tool by letter or toggle off. */
  bool handleShortcutKey(int qt_key);
  std::string defaultToolId() const { return default_tool_id_; }
  char shortcutKeyForTool(const std::string& id) const;
  /** Only Move Camera forwards unconsumed events to viewport orbit/pan/zoom. */
  bool allowsViewportNavigation() const;

 private:
  void applyDefaultProperties(Tool* tool);

  std::vector<std::unique_ptr<Tool>> tools_;
  std::vector<std::string> tool_ids_;
  std::vector<std::string> toolbar_tool_ids_;
  std::string active_tool_id_;
  std::string default_tool_id_;
  ToolContext context_;
};

}  // namespace common
}  // namespace autoviz
