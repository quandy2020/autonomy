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

/**
 * @file plugin_loader.hpp
 * @brief Dynamic shared-library loader that registers @ref Component creators.
 */

#ifndef ATLAS_COMMON_PLUGIN_LOADER_HPP_
#define ATLAS_COMMON_PLUGIN_LOADER_HPP_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "common/component/component.hpp"
#include "common/component/component_factory.hpp"

namespace atlas {
namespace common {

/** @brief Required plugin entry point symbol. */
inline constexpr char kComponentPluginRegisterSymbol[] =
    "AtlasRegisterComponents";

/** @brief Signature exported with C linkage under the fixed symbol. */
using ComponentPluginRegisterFunction = bool (*)(ComponentFactory*);

/**
 * @brief Export a plugin registration function with C linkage and this form:
 * `ATLAS_COMPONENT_PLUGIN_EXPORT bool AtlasRegisterComponents(
 * ComponentFactory* factory);`
 */

#if defined(_WIN32)
#define ATLAS_COMPONENT_PLUGIN_EXPORT extern "C" __declspec(dllexport)
#else
#define ATLAS_COMPONENT_PLUGIN_EXPORT \
  extern "C" __attribute__((visibility("default")))
#endif

/**
 * @brief Loads a dynamic library and invokes its component registration entry.
 * Plugins export a function named @ref kComponentPluginRegisterSymbol with
 * signature @ref ComponentPluginRegisterFunction. Registrations are first
 * collected in a temporary factory; a failed plugin cannot partially mutate
 * the destination factory.
 */
class PluginLoader {
 public:
  /**
   * @brief Bind this loader to a factory that will receive registrations.
   * @param[in] factory Non-owning pointer; must outlive this loader. May be
   *                    null (then @ref Load always fails).
   */
  explicit PluginLoader(ComponentFactory* factory);

  /**
   * @brief Load a plugin from disk and allow it to register creators.
   * @param[in] plugin_path Absolute or relative path to the shared library.
   * @return True after successful library loading and registration; false on
   *         invalid input, missing entry point, load failure, or rejection.
   */
  bool Load(const std::string& plugin_path);

 private:
  ComponentFactory* factory_ = nullptr;
  std::unordered_set<std::string> loaded_paths_;
  std::vector<std::shared_ptr<void>> loaded_libraries_;
};

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_PLUGIN_LOADER_HPP_
