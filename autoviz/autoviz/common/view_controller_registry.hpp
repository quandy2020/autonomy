/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file view_controller_registry.hpp
 * @brief Built-in + dynamically loaded view-controller types (pluginlib-style).
 *
 * Maps type names (@c Orbit, @c FPS, …) to appliers that configure a
 * @ref rendering::ViewController. Also maps legacy RViz class names.
 *
 * @see ViewControllerPlugin
 * @see ViewManager
 * @see rendering::ViewController
 */

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

/**
 * @brief Functor that applies a view type onto a live controller instance.
 */
using ViewControllerApplier =
    std::function<void(rendering::ViewController*)>;

/**
 * @class ViewControllerRegistry
 * @brief Process-wide registry of view-controller type appliers.
 */
class ViewControllerRegistry {
 public:
  /**
   * @brief Returns the process-wide singleton.
   * @return Mutable registry reference.
   */
  static ViewControllerRegistry& instance();

  /**
   * @brief Registers built-in types (Orbit, XYOrbit, TopDownOrtho, FPS, …).
   */
  void registerBuiltinTypes();

  /**
   * @brief Registers (or replaces) a type applier.
   *
   * @param type_name Type id string.
   * @param applier Functor configuring the controller.
   */
  void registerType(const std::string& type_name, ViewControllerApplier applier);

  /**
   * @brief All registered type names (built-in + plugins).
   * @return Ordered type name list.
   */
  const std::vector<std::string>& typeNames() const { return type_names_; }

  /**
   * @brief Built-in type names only (excludes dynamically loaded).
   * @return Built-in type name list.
   */
  const std::vector<std::string>& builtinTypeNames() const {
    return builtin_type_names_;
  }

  /**
   * @brief Applies the named type to @p controller.
   *
   * @param type_name Type id (must be known; unknown types are no-ops or
   *        fall back depending on implementation).
   * @param controller Live controller to configure (non-null).
   */
  void applyByName(const std::string& type_name,
                   rendering::ViewController* controller) const;

  /**
   * @brief Qt-string overload of @ref applyByName(const std::string&, …).
   *
   * @param type_name Type id as @c QString.
   * @param controller Live controller to configure.
   */
  void applyByName(const QString& type_name,
                   rendering::ViewController* controller) const;

  /**
   * @brief Maps an RViz plugin class string to an Autoviz type name.
   *
   * @param rviz_class RViz class id (e.g. from a @c .rviz file).
   * @return Autoviz type name, or empty if unmapped.
   */
  std::string mapRvizClass(const std::string& rviz_class) const;

  /**
   * @brief Returns whether @p type_name is registered.
   * @param type_name Type id to test.
   * @return @c true if known.
   */
  bool isKnownType(const std::string& type_name) const;

  /**
   * @brief Default type used when none is specified.
   * @return Always @c "Orbit".
   */
  std::string defaultTypeName() const { return "Orbit"; }

  /**
   * @brief Loads view-controller plugins from @c AUTOVIZ_PLUGIN_PATH.
   */
  void loadPluginsFromEnv();

  /**
   * @brief Loads view-controller plugins from a directory.
   * @param path Directory to scan for shared libraries.
   */
  void loadPluginsFromPath(const std::string& path);

 private:
  ViewControllerRegistry();

  /**
   * @brief Internal helper that also records @p type_name as built-in.
   *
   * @param type_name Type id.
   * @param applier Applier functor.
   */
  void registerBuiltin(const std::string& type_name,
                       ViewControllerApplier applier);

  std::vector<std::string> builtin_type_names_;
  std::vector<std::string> type_names_;
  std::unordered_map<std::string, ViewControllerApplier> appliers_;
  std::vector<void*> plugin_handles_;
};

}  // namespace common
}  // namespace autoviz
