/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "common/component/plugin_loader.hpp"

#include <filesystem>
#include <system_error>
#include <utility>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace atlas {
namespace common {

PluginLoader::PluginLoader(ComponentFactory* factory) : factory_(factory) {}

bool PluginLoader::Load(const std::string& plugin_path) {
  if (factory_ == nullptr || plugin_path.empty()) {
    return false;
  }
  std::error_code path_error;
  const std::filesystem::path canonical_path =
      std::filesystem::weakly_canonical(plugin_path, path_error);
  if (path_error || !loaded_paths_.insert(canonical_path.string()).second) {
    return false;
  }

#if defined(_WIN32)
  HMODULE native_handle = LoadLibraryW(canonical_path.wstring().c_str());
  if (native_handle == nullptr) {
    loaded_paths_.erase(canonical_path.string());
    return false;
  }
  auto close_library = [](void* handle) {
    if (handle != nullptr) {
      FreeLibrary(static_cast<HMODULE>(handle));
    }
  };
  const auto register_components =
      reinterpret_cast<ComponentPluginRegisterFunction>(
          GetProcAddress(native_handle, kComponentPluginRegisterSymbol));
#else
  void* native_handle = dlopen(canonical_path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (native_handle == nullptr) {
    loaded_paths_.erase(canonical_path.string());
    return false;
  }
  auto close_library = [](void* handle) {
    if (handle != nullptr) {
      dlclose(handle);
    }
  };
  dlerror();
  auto register_components = reinterpret_cast<ComponentPluginRegisterFunction>(
      dlsym(native_handle, kComponentPluginRegisterSymbol));
  if (dlerror() != nullptr) {
    register_components = nullptr;
  }
#endif

  std::shared_ptr<void> library(native_handle, close_library);
  if (register_components == nullptr) {
    loaded_paths_.erase(canonical_path.string());
    return false;
  }

  ComponentFactory staged_factory;
  bool accepted = false;
  try {
    accepted = register_components(&staged_factory);
  } catch (...) {
    accepted = false;
  }
  if (!accepted) {
    loaded_paths_.erase(canonical_path.string());
    return false;
  }

  factory_->MergePluginCreators(std::move(staged_factory), library);
  loaded_libraries_.push_back(std::move(library));
  return true;
}

}  // namespace common
}  // namespace atlas
