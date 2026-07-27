/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/tool_registry.hpp"

#include <QDir>
#include <QFileInfo>

#include "autoviz/common/path_env_utils.hpp"
#include "autoviz/common/plugin_loader.hpp"

#include "autoviz/tools/focus_camera_tool.hpp"
#include "autoviz/tools/interact_tool.hpp"
#include "autoviz/tools/measure_tool.hpp"
#include "autoviz/tools/move_camera_tool.hpp"
#include "autoviz/tools/nav_goal_tool.hpp"
#include "autoviz/tools/pose_estimate_tool.hpp"
#include "autoviz/tools/publish_point_tool.hpp"
#include "autoviz/tools/select_tool.hpp"

namespace autoviz {
namespace common {

ToolRegistry& ToolRegistry::instance() {
  static ToolRegistry registry;
  static bool initialized = false;
  if (!initialized) {
    registry.registerBuiltinTools();
    registry.loadPluginsFromEnv();
    initialized = true;
  }
  return registry;
}

void ToolRegistry::registerTool(const std::string& id, ToolCreator creator,
                                const char* label) {
  Entry entry;
  entry.creator = std::move(creator);
  if (label != nullptr) {
    entry.label = label;
  }
  entries_[id] = std::move(entry);
}

void ToolRegistry::registerBuiltinTools() {
  registerTool("Interact", [] { return std::make_unique<tools::InteractTool>(); });
  registerTool("MoveCamera",
               [] { return std::make_unique<tools::MoveCameraTool>(); });
  registerTool("Select", [] { return std::make_unique<tools::SelectTool>(); });
  registerTool("FocusCamera",
               [] { return std::make_unique<tools::FocusCameraTool>(); });
  registerTool("Measure", [] { return std::make_unique<tools::MeasureTool>(); });
  registerTool("PoseEstimate",
               [] { return std::make_unique<tools::PoseEstimateTool>(); });
  registerTool("NavGoal", [] { return std::make_unique<tools::NavGoalTool>(); });
  registerTool("PublishPoint",
               [] { return std::make_unique<tools::PublishPointTool>(); });
}

std::vector<std::string> ToolRegistry::toolIds() const {
  std::vector<std::string> ids;
  ids.reserve(entries_.size());
  for (const auto& [id, entry] : entries_) {
    (void)entry;
    ids.push_back(id);
  }
  return ids;
}

QString ToolRegistry::toolLabel(const std::string& id) const {
  const auto it = entries_.find(id);
  if (it == entries_.end()) {
    return QString::fromStdString(id);
  }
  if (!it->second.label.empty()) {
    return QString::fromStdString(it->second.label);
  }
  if (auto tool = it->second.creator()) {
    return tool->label();
  }
  return QString::fromStdString(id);
}

std::unique_ptr<Tool> ToolRegistry::create(const std::string& id) const {
  const auto it = entries_.find(id);
  if (it == entries_.end()) {
    return nullptr;
  }
  return it->second.creator();
}

void ToolRegistry::loadPluginsFromEnv() {
  for (const QString& part : pluginSearchPaths()) {
    loadPluginsFromPath(part.toStdString());
  }
}

void ToolRegistry::loadPluginsFromPath(const std::string& path) {
  QDir dir(QString::fromStdString(path));
  if (!dir.exists()) {
    return;
  }
  for (const QFileInfo& info : dir.entryInfoList(
           PluginLoader::libraryFilenameFilters(), QDir::Files | QDir::Readable)) {
    void* handle = PluginLoader::open(info.absoluteFilePath().toStdString());
    if (handle == nullptr) {
      continue;
    }
    using RegisterFn = void (*)(ToolRegistry*);
    auto* register_fn =
        reinterpret_cast<RegisterFn>(PluginLoader::symbol(handle, "autoviz_register_tools"));
    if (register_fn == nullptr) {
      PluginLoader::close(handle);
      continue;
    }
    register_fn(this);
    plugin_handles_.push_back(handle);
  }
}

}  // namespace common
}  // namespace autoviz
