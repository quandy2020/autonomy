/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QStringList>

namespace autoviz {
namespace common {

/** Cross-platform dynamic library loader (dlopen / LoadLibrary). */
class PluginLoader {
 public:
  using Handle = void*;

  static QStringList libraryFilenameFilters();

  /** Load a plugin shared library; returns nullptr on failure. */
  static Handle open(const std::string& path);

  static void* symbol(Handle handle, const char* name);
  static void close(Handle handle);
};

}  // namespace common
}  // namespace autoviz
