/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file path_env_utils.hpp
 * @brief Helpers for PATH-style environment variables used by Autoviz plugins
 *        and @c package:// resource resolution.
 *
 * @see PluginLoader
 * @see DisplayRegistry::loadPluginsFromEnv
 */

#pragma once

#include <string>

#include <QStringList>

namespace autoviz {
namespace common {

/**
 * @brief Splits a PATH-style environment variable into discrete entries.
 *
 * Uses @c ':' on Unix and @c ';' on Windows (same convention as @c PATH).
 *
 * @param value Raw environment string (may be empty).
 * @return List of path entries; empty entries are omitted by platform rules.
 */
QStringList splitPathList(const QString& value);

/**
 * @brief Returns plugin search directories from @c AUTOVIZ_PLUGIN_PATH.
 *
 * @return Ordered list of directories to scan for shared libraries.
 * @see PluginLoader::libraryFilenameFilters
 */
QStringList pluginSearchPaths();

/**
 * @brief Returns resource search directories from @c AUTOVIZ_RESOURCE_PATH.
 *
 * Used when resolving @c package:// URIs for URDF meshes and similar assets.
 *
 * @param base_directory Optional extra root prepended to the search list
 *        (e.g. the session file directory). Empty means env-only.
 * @return Ordered list of directories to search.
 */
QStringList resourceSearchPaths(const std::string& base_directory = {});

}  // namespace common
}  // namespace autoviz
