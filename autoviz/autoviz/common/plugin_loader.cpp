/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/plugin_loader.hpp"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace autoviz {
namespace common {

QStringList PluginLoader::libraryFilenameFilters() {
#if defined(_WIN32)
  return {QStringLiteral("*.dll")};
#elif defined(__APPLE__)
  return {QStringLiteral("*.dylib"), QStringLiteral("lib*.dylib"),
          QStringLiteral("*.so"), QStringLiteral("lib*.so")};
#else
  return {QStringLiteral("*.so"), QStringLiteral("lib*.so")};
#endif
}

PluginLoader::Handle PluginLoader::open(const std::string& path) {
#if defined(_WIN32)
  return LoadLibraryA(path.c_str());
#else
  return dlopen(path.c_str(), RTLD_NOW);
#endif
}

void* PluginLoader::symbol(Handle handle, const char* name) {
  if (handle == nullptr || name == nullptr) {
    return nullptr;
  }
#if defined(_WIN32)
  return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(handle),
                                                name));
#else
  return dlsym(handle, name);
#endif
}

void PluginLoader::close(Handle handle) {
  if (handle == nullptr) {
    return;
  }
#if defined(_WIN32)
  FreeLibrary(static_cast<HMODULE>(handle));
#else
  dlclose(handle);
#endif
}

}  // namespace common
}  // namespace autoviz
