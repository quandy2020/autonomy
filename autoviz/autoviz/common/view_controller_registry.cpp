/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/view_controller_registry.hpp"

#include <algorithm>

#include <QDir>
#include <QFileInfo>

#include "autoviz/common/path_env_utils.hpp"
#include "autoviz/common/plugin_loader.hpp"
#include "autoviz/plugins/view_controllers/follower/third_person_follower_view_controller.hpp"
#include "autoviz/plugins/view_controllers/fps/fps_view_controller.hpp"
#include "autoviz/plugins/view_controllers/frame/frame_view_controller.hpp"
#include "autoviz/plugins/view_controllers/orbit/orbit_view_controller.hpp"
#include "autoviz/plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.hpp"
#include "autoviz/plugins/view_controllers/xy_orbit/xy_orbit_view_controller.hpp"
#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace common {
namespace {

std::string RvizClassShortName(const std::string& rviz_class) {
  const std::size_t slash = rviz_class.rfind('/');
  if (slash == std::string::npos) {
    return rviz_class;
  }
  return rviz_class.substr(slash + 1);
}

}  // namespace

ViewControllerRegistry& ViewControllerRegistry::instance() {
  static ViewControllerRegistry registry;
  static bool initialized = false;
  if (!initialized) {
    registry.registerBuiltinTypes();
    registry.loadPluginsFromEnv();
    initialized = true;
  }
  return registry;
}

ViewControllerRegistry::ViewControllerRegistry() = default;

void ViewControllerRegistry::registerBuiltinTypes() {
  using display::view_controllers::FixedOrientationOrthoViewController;
  using display::view_controllers::FPSViewController;
  using display::view_controllers::FrameViewController;
  using display::view_controllers::OrbitViewController;
  using display::view_controllers::ThirdPersonFollowerViewController;
  using display::view_controllers::XYOrbitViewController;

  // RViz2 default_plugins view controllers (display/view_controllers/*).
  registerBuiltin(OrbitViewController::kTypeName, &OrbitViewController::apply);
  registerBuiltin(XYOrbitViewController::kTypeName,
                  &XYOrbitViewController::apply);
  registerBuiltin(FixedOrientationOrthoViewController::kTypeName,
                  &FixedOrientationOrthoViewController::apply);
  registerBuiltin(FPSViewController::kTypeName, &FPSViewController::apply);
  registerBuiltin(ThirdPersonFollowerViewController::kTypeName,
                  &ThirdPersonFollowerViewController::apply);

  // RViz FrameViewController → FPS backend (not listed separately in Views).
  appliers_[FrameViewController::kTypeName] = &FrameViewController::apply;

  // Legacy aliases (not shown in Views Type selector).
  appliers_["TopDown"] = &FixedOrientationOrthoViewController::apply;
  appliers_["FPSMotion"] = &FPSViewController::apply;
}

void ViewControllerRegistry::registerBuiltin(
    const std::string& type_name, ViewControllerApplier applier) {
  builtin_type_names_.push_back(type_name);
  registerType(type_name, std::move(applier));
}

void ViewControllerRegistry::registerType(const std::string& type_name,
                                          ViewControllerApplier applier) {
  if (appliers_.find(type_name) == appliers_.end()) {
    type_names_.push_back(type_name);
  }
  appliers_[type_name] = std::move(applier);
}

void ViewControllerRegistry::applyByName(
    const std::string& type_name, rendering::ViewController* controller) const {
  const auto it = appliers_.find(type_name);
  if (it != appliers_.end() && controller != nullptr) {
    it->second(controller);
    return;
  }
  applyByName(defaultTypeName(), controller);
}

void ViewControllerRegistry::applyByName(
    const QString& type_name, rendering::ViewController* controller) const {
  applyByName(type_name.toStdString(), controller);
}

std::string ViewControllerRegistry::mapRvizClass(
    const std::string& rviz_class) const {
  const std::string short_name = RvizClassShortName(rviz_class);
  if (short_name.find("ThirdPersonFollow") != std::string::npos) {
    return "ThirdPersonFollow";
  }
  if (short_name.find("XYOrbit") != std::string::npos) {
    return "XYOrbit";
  }
  if (short_name.find("TopDownOrtho") != std::string::npos ||
      short_name.find("FixedOrientationOrtho") != std::string::npos ||
      short_name.find("TopDown") != std::string::npos) {
    return "TopDownOrtho";
  }
  if (short_name.find("FrameViewController") != std::string::npos ||
      short_name.find("FrameAligned") != std::string::npos) {
    return "FPS";
  }
  if (short_name.find("FPSMotion") != std::string::npos ||
      short_name.find("FPS") != std::string::npos) {
    return "FPS";
  }
  if (short_name.find("Orbit") != std::string::npos) {
    return "Orbit";
  }
  return defaultTypeName();
}

bool ViewControllerRegistry::isKnownType(const std::string& type_name) const {
  return appliers_.find(type_name) != appliers_.end();
}

void ViewControllerRegistry::loadPluginsFromEnv() {
  for (const QString& part : pluginSearchPaths()) {
    loadPluginsFromPath(part.toStdString());
  }
}

void ViewControllerRegistry::loadPluginsFromPath(const std::string& path) {
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
    using RegisterFn = void (*)(ViewControllerRegistry*);
    auto* register_fn = reinterpret_cast<RegisterFn>(
        PluginLoader::symbol(handle, "autoviz_register_view_controllers"));
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
