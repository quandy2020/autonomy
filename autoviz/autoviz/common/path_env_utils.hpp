/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include <QStringList>

namespace autoviz {
namespace common {

/** Split a PATH-style environment variable (':' on Unix, ';' on Windows). */
QStringList splitPathList(const QString& value);

/** Search paths from `AUTOVIZ_PLUGIN_PATH` (platform-native separator). */
QStringList pluginSearchPaths();

/** Search paths from `AUTOVIZ_RESOURCE_PATH` for package:// URDF meshes. */
QStringList resourceSearchPaths(const std::string& base_directory = {});

}  // namespace common
}  // namespace autoviz
