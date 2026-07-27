/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/view_controller_registry.hpp"

#include <algorithm>

#include <QDir>
#include <QFileInfo>

#include "autoviz/common/path_env_utils.hpp"
#include "autoviz/common/plugin_loader.hpp"
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

void ApplyBuiltinType(rendering::ViewControllerType type,
                      rendering::ViewController* controller) {
  if (controller != nullptr) {
    controller->setType(type);
  }
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
  registerBuiltin("Orbit", [](rendering::ViewController* vc) {
    ApplyBuiltinType(rendering::ViewControllerType::kOrbit, vc);
  });
  registerBuiltin("XYOrbit", [](rendering::ViewController* vc) {
    ApplyBuiltinType(rendering::ViewControllerType::kXyOrbit, vc);
  });
  registerBuiltin("TopDown", [](rendering::ViewController* vc) {
    ApplyBuiltinType(rendering::ViewControllerType::kTopDown, vc);
  });
  registerBuiltin("TopDownOrtho", [](rendering::ViewController* vc) {
    ApplyBuiltinType(rendering::ViewControllerType::kTopDownOrtho, vc);
  });
  registerBuiltin("FPS", [](rendering::ViewController* vc) {
    ApplyBuiltinType(rendering::ViewControllerType::kFps, vc);
  });
  registerBuiltin("FPSMotion", [](rendering::ViewController* vc) {
    ApplyBuiltinType(rendering::ViewControllerType::kFpsMotion, vc);
  });
  registerBuiltin("ThirdPersonFollow", [](rendering::ViewController* vc) {
    ApplyBuiltinType(rendering::ViewControllerType::kThirdPersonFollow, vc);
  });
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
      short_name.find("FixedOrientationOrtho") != std::string::npos) {
    return "TopDownOrtho";
  }
  if (short_name.find("TopDown") != std::string::npos) {
    return "TopDown";
  }
  if (short_name.find("FPSMotion") != std::string::npos) {
    return "FPSMotion";
  }
  if (short_name.find("FPS") != std::string::npos ||
      short_name.find("FrameAligned") != std::string::npos) {
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
