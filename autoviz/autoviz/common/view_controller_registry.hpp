/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <QString>

namespace autoviz {
namespace rendering {
class ViewController;
}

namespace common {

using ViewControllerApplier =
    std::function<void(rendering::ViewController*)>;

/** Built-in + dynamically loaded view controllers (RViz pluginlib-style). */
class ViewControllerRegistry {
 public:
  static ViewControllerRegistry& instance();

  void registerBuiltinTypes();
  void registerType(const std::string& type_name, ViewControllerApplier applier);

  const std::vector<std::string>& typeNames() const { return type_names_; }
  const std::vector<std::string>& builtinTypeNames() const {
    return builtin_type_names_;
  }

  void applyByName(const std::string& type_name,
                   rendering::ViewController* controller) const;
  void applyByName(const QString& type_name,
                   rendering::ViewController* controller) const;

  std::string mapRvizClass(const std::string& rviz_class) const;
  bool isKnownType(const std::string& type_name) const;
  std::string defaultTypeName() const { return "Orbit"; }

  void loadPluginsFromEnv();
  void loadPluginsFromPath(const std::string& path);

 private:
  ViewControllerRegistry();

  void registerBuiltin(const std::string& type_name,
                       ViewControllerApplier applier);

  std::vector<std::string> builtin_type_names_;
  std::vector<std::string> type_names_;
  std::unordered_map<std::string, ViewControllerApplier> appliers_;
  std::vector<void*> plugin_handles_;
};

}  // namespace common
}  // namespace autoviz
