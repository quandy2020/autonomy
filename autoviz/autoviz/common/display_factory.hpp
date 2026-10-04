/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display_factory.hpp
 * @brief Thin static facade over @ref DisplayRegistry for creating displays.
 *
 * Keeps call sites free of the singleton while providing the same create /
 * default-config / supported-types API used by VisualizationManager.
 *
 * @see DisplayRegistry
 * @see DisplayConfig
 * @see display::Display
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autoviz/common/session_config.hpp"
#include "autoviz/display/display.hpp"

namespace autoviz {
namespace common {

/**
 * @class DisplayFactory
 * @brief Static helpers that forward to @ref DisplayRegistry::instance().
 */
class DisplayFactory {
 public:
  /**
   * @brief Lists all registered display type ids.
   * @return Supported type name list.
   */
  static std::vector<std::string> supportedTypes();

  /**
   * @brief Returns the default @ref DisplayConfig for a type.
   *
   * @param type Display type id.
   * @return Default config (empty/unknown type yields a minimal stub).
   */
  static DisplayConfig defaultForType(const std::string& type);

  /**
   * @brief Instantiates a display from configuration.
   *
   * @param config Type, name, channel, properties, and optional children.
   * @return New display, or @c nullptr if the type is unknown.
   */
  static std::unique_ptr<display::Display> create(const DisplayConfig& config);
};

}  // namespace common
}  // namespace autoviz
