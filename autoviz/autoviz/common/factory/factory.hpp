/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_common::Factory / PluginInfo
 * (Willow Garage / Open Source Robotics Foundation / Bosch).
 *****************************************************************************/

/**
 * @file factory.hpp
 * @brief Abstract plugin factory (RViz @c rviz_common::Factory).
 *
 * Declares plugins and returns @ref PluginInfo metadata. Instantiation is left
 * to typed subclasses (@ref ClassIdRecordingFactory, @ref PluginFactory).
 *
 * @see PluginInfo
 * @see ClassIdRecordingFactory
 * @see PluginFactory
 */

#pragma once

#include <vector>

#include <QString>

#include "autoviz/common/plugin_info.hpp"

namespace autoviz {
namespace common {
namespace factory {

/**
 * @class Factory
 * @brief Abstract base for loadable plugin class factories.
 *
 * Provides enumeration and metadata lookup. Subclasses specialize
 * @c make() / @c makeRaw() for concrete product types.
 */
class Factory {
 public:
  virtual ~Factory() = default;

  /**
   * @brief Lists every declared plugin (built-in and dynamically registered).
   * @return Plugin metadata vector.
   */
  virtual std::vector<PluginInfo> getDeclaredPlugins() = 0;

  /**
   * @brief Looks up metadata for one class id.
   *
   * @param class_id Plugin class id (e.g. @c "autoviz/Grid" or @c "Grid").
   * @return Matching @ref PluginInfo (fields may be empty if unknown).
   */
  virtual PluginInfo getPluginInfo(const QString& class_id) const = 0;
};

}  // namespace factory
}  // namespace common
}  // namespace autoviz
