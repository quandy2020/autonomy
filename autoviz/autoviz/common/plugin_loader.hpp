/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plugin_loader.hpp
 * @brief Cross-platform dynamic library loader for Autoviz plugins.
 *
 * Thin wrapper over @c dlopen / @c LoadLibrary used by Display, Tool,
 * ViewController, and Transformer registries when scanning
 * @c AUTOVIZ_PLUGIN_PATH.
 *
 * @see path_env_utils.hpp
 * @see DisplayRegistry::loadPluginsFromPath
 */

#pragma once

#include <string>
#include <vector>

#include <QStringList>

namespace autoviz {
namespace common {

/**
 * @class PluginLoader
 * @brief Static helpers to open shared libraries and resolve symbols.
 *
 * @note Handles are opaque @c void*; ownership stays with the caller until
 *       @ref close(). Failed @ref open() returns @c nullptr.
 */
class PluginLoader {
 public:
  /** Opaque handle to a loaded shared library (@c void*). */
  using Handle = void*;

  /**
   * @brief Filename filters for plugin shared libraries on this platform.
   *
   * @return Qt-style wildcards (e.g. @c "*.so", @c "*.dylib", @c "*.dll").
   */
  static QStringList libraryFilenameFilters();

  /**
   * @brief Loads a plugin shared library from @p path.
   *
   * @param path Absolute or relative filesystem path to the library.
   * @return Library handle, or @c nullptr on failure.
   */
  static Handle open(const std::string& path);

  /**
   * @brief Resolves an exported symbol by name.
   *
   * @param handle Previously opened library handle.
   * @param name Null-terminated C symbol (e.g. @c "autoviz_register_displays").
   * @return Function/data pointer, or @c nullptr if missing.
   */
  static void* symbol(Handle handle, const char* name);

  /**
   * @brief Unloads a previously opened library.
   *
   * @param handle Handle from @ref open(); may be @c nullptr (no-op).
   */
  static void close(Handle handle);
};

}  // namespace common
}  // namespace autoviz
