/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/tool_manager.hpp"

#include <QMouseEvent>
#include <QWheelEvent>
#include <Qt>

#include <algorithm>
#include <cctype>

#include "autoviz/common/tool_registry.hpp"

namespace autoviz {
namespace common {
namespace {

DisplayPropertyMap DefaultProperties(const Tool& tool) {
  DisplayPropertyMap properties;
  for (const auto& spec : tool.propertySpecs()) {
    properties[spec.key] = spec.default_value;
  }
  return properties;
}

int ShortcutToQtKey(char shortcut) {
  if (shortcut == '\0') {
    return 0;
  }
  const unsigned char upper = static_cast<unsigned char>(
      std::toupper(static_cast<unsigned char>(shortcut)));
  if (upper < 'A' || upper > 'Z') {
    return 0;
  }
  return Qt::Key_A + static_cast<int>(upper - 'A');
}

}  // namespace

ToolManager::ToolManager() {
  ToolRegistry& registry = ToolRegistry::instance();
  for (const std::string& id : registry.toolIds()) {
    auto tool = registry.create(id);
    if (tool == nullptr) {
      continue;
    }
    applyDefaultProperties(tool.get());
    tool_ids_.push_back(id);
    tools_.push_back(std::move(tool));
  }
  active_tool_id_ = tool_ids_.empty() ? "" : "Interact";
  if (toolById(active_tool_id_) == nullptr && !tool_ids_.empty()) {
    active_tool_id_ = tool_ids_.front();
  }
  default_tool_id_ = toolById("Interact") != nullptr ? "Interact" : active_tool_id_;
  resetToolbarToDefault();
}

ToolManager::~ToolManager() = default;

void ToolManager::applyDefaultProperties(Tool* tool) {
  if (tool == nullptr) {
    return;
  }
  tool->setProperties(DefaultProperties(*tool));
}

void ToolManager::setContext(ToolContext context) {
  context_ = std::move(context);
  if (Tool* tool = activeTool()) {
    tool->updateContext(&context_);
  }
}

Tool* ToolManager::toolById(const std::string& id) {
  for (auto& tool : tools_) {
    if (tool->id() == id) {
      return tool.get();
    }
  }
  return nullptr;
}

const Tool* ToolManager::toolById(const std::string& id) const {
  for (const auto& tool : tools_) {
    if (tool->id() == id) {
      return tool.get();
    }
  }
  return nullptr;
}

Tool* ToolManager::activeTool() {
  return toolById(active_tool_id_);
}

std::vector<DisplayPropertySpec> ToolManager::activeToolPropertySpecs() const {
  if (const Tool* tool = toolById(active_tool_id_)) {
    return tool->propertySpecs();
  }
  return {};
}

void ToolManager::setActiveToolProperty(const std::string& key,
                                        const std::string& value) {
  if (Tool* tool = activeTool()) {
    tool->setPropertyValue(key, value);
  }
}

void ToolManager::applyToolConfigs(const std::vector<ToolConfig>& configs) {
  for (auto& tool : tools_) {
    applyDefaultProperties(tool.get());
  }
  for (const auto& config : configs) {
    if (Tool* tool = toolById(config.id)) {
      auto merged = tool->properties();
      for (const auto& prop : config.properties) {
        merged[prop.first] = prop.second;
      }
      tool->setProperties(merged);
    }
  }
}

std::vector<ToolConfig> ToolManager::currentToolConfigs() const {
  std::vector<ToolConfig> configs;
  configs.reserve(tools_.size());
  for (const auto& tool : tools_) {
    if (tool->propertySpecs().empty()) {
      continue;
    }
    ToolConfig config;
    config.id = tool->id();
    config.properties = tool->properties();
    configs.push_back(std::move(config));
  }
  return configs;
}

void ToolManager::resetToolbarToDefault() {
  toolbar_tool_ids_ = tool_ids_;
}

void ToolManager::setToolbarToolIds(const std::vector<std::string>& ids) {
  auto normalize_id = [this](const std::string& raw) -> std::string {
    static const struct {
      const char* alias;
      const char* canonical;
    } kAliases[] = {
        {"SetGoal", "NavGoal"},
        {"SetInitialPose", "PoseEstimate"},
        {"Move Camera", "MoveCamera"},
        {"Focus Camera", "FocusCamera"},
        {"Publish Point", "PublishPoint"},
        {"2D Goal Pose", "NavGoal"},
        {"2D Pose Estimate", "PoseEstimate"},
    };
    std::string id = raw;
    for (const auto& entry : kAliases) {
      if (id == entry.alias) {
        id = entry.canonical;
        break;
      }
    }
    return toolById(id) != nullptr ? id : std::string{};
  };

  if (ids.empty()) {
    resetToolbarToDefault();
    return;
  }
  std::vector<std::string> filtered;
  filtered.reserve(ids.size());
  for (const std::string& raw_id : ids) {
    const std::string id = normalize_id(raw_id);
    if (id.empty()) {
      continue;
    }
    if (std::find(filtered.begin(), filtered.end(), id) == filtered.end()) {
      filtered.push_back(id);
    }
  }
  if (filtered.empty()) {
    resetToolbarToDefault();
    return;
  }
  toolbar_tool_ids_ = std::move(filtered);
  if (std::find(toolbar_tool_ids_.begin(), toolbar_tool_ids_.end(),
                active_tool_id_) == toolbar_tool_ids_.end()) {
    setActiveTool(toolbar_tool_ids_.front());
  }
}

bool ToolManager::addToolToToolbar(const std::string& id) {
  if (toolById(id) == nullptr) {
    return false;
  }
  if (std::find(toolbar_tool_ids_.begin(), toolbar_tool_ids_.end(), id) !=
      toolbar_tool_ids_.end()) {
    return false;
  }
  toolbar_tool_ids_.push_back(id);
  return true;
}

bool ToolManager::removeToolFromToolbar(const std::string& id) {
  if (toolbar_tool_ids_.size() <= 1) {
    return false;
  }
  const auto it =
      std::find(toolbar_tool_ids_.begin(), toolbar_tool_ids_.end(), id);
  if (it == toolbar_tool_ids_.end()) {
    return false;
  }
  const bool removing_active = active_tool_id_ == id;
  toolbar_tool_ids_.erase(it);
  if (removing_active) {
    setActiveTool(toolbar_tool_ids_.front());
  }
  return true;
}

std::vector<std::string> ToolManager::toolsNotInToolbar() const {
  std::vector<std::string> available;
  for (const std::string& id : tool_ids_) {
    if (std::find(toolbar_tool_ids_.begin(), toolbar_tool_ids_.end(), id) ==
        toolbar_tool_ids_.end()) {
      available.push_back(id);
    }
  }
  return available;
}

QString ToolManager::toolLabel(const std::string& id) const {
  if (const Tool* tool = toolById(id)) {
    return tool->label();
  }
  return QString::fromStdString(id);
}

bool ToolManager::setActiveTool(const std::string& id) {
  Tool* previous = activeTool();
  if (previous != nullptr) {
    previous->deactivate();
  }
  for (auto& tool : tools_) {
    if (tool->id() == id) {
      active_tool_id_ = id;
      tool->activate(&context_);
      return true;
    }
  }
  if (previous != nullptr) {
    previous->activate(&context_);
  }
  return false;
}

QString ToolManager::activeStatusText() const {
  if (const Tool* tool = toolById(active_tool_id_)) {
    return tool->statusText();
  }
  return {};
}

bool ToolManager::mousePressEvent(QMouseEvent* event) {
  if (Tool* tool = activeTool()) {
    if (tool->mousePressEvent(event)) {
      return true;
    }
    return !allowsViewportNavigation();
  }
  return false;
}

bool ToolManager::mouseMoveEvent(QMouseEvent* event) {
  if (Tool* tool = activeTool()) {
    if (tool->mouseMoveEvent(event)) {
      return true;
    }
    return !allowsViewportNavigation();
  }
  return false;
}

bool ToolManager::mouseReleaseEvent(QMouseEvent* event) {
  if (Tool* tool = activeTool()) {
    if (tool->mouseReleaseEvent(event)) {
      return true;
    }
    return !allowsViewportNavigation();
  }
  return false;
}

bool ToolManager::wheelEvent(QWheelEvent* event) {
  if (Tool* tool = activeTool()) {
    if (tool->wheelEvent(event)) {
      return true;
    }
    return !allowsViewportNavigation();
  }
  return false;
}

void ToolManager::onDraw(rendering::SceneOverlay& scene) {
  if (Tool* tool = activeTool()) {
    tool->onDraw(scene);
  }
}

bool ToolManager::allowsViewportNavigation() const {
  return active_tool_id_ == "MoveCamera" || active_tool_id_ == "Interact";
}

char ToolManager::shortcutKeyForTool(const std::string& id) const {
  if (const Tool* tool = toolById(id)) {
    return tool->shortcutKey();
  }
  return '\0';
}

bool ToolManager::handleShortcutKey(int qt_key) {
  if (qt_key == Qt::Key_Escape) {
    if (active_tool_id_ != default_tool_id_) {
      return setActiveTool(default_tool_id_);
    }
    return false;
  }

  std::string target_id;
  for (const auto& tool : tools_) {
    const char key = tool->shortcutKey();
    if (key == '\0') {
      continue;
    }
    if (qt_key == ShortcutToQtKey(key)) {
      target_id = tool->id();
      break;
    }
  }
  if (target_id.empty()) {
    return false;
  }
  if (active_tool_id_ == target_id) {
    return setActiveTool(default_tool_id_);
  }
  return setActiveTool(target_id);
}

}  // namespace common
}  // namespace autoviz
