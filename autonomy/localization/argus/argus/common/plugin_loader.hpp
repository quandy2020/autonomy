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

#ifndef ARGUS_COMMON_PLUGIN_LOADER_HPP_
#define ARGUS_COMMON_PLUGIN_LOADER_HPP_

#include <string>

#include "argus/common/component.hpp"
#include "argus/common/component_factory.hpp"

namespace argus {
namespace common {

/**
 * @brief Loads `.so` / dynamic plugins that register into a @ref ComponentFactory.
 * Current implementation is a stub: @ref Load validates arguments but does not
 * yet call `dlopen`. Wire the platform loader when plugin packaging is ready.
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
     * @return `true` when the path is non-empty and @p factory is non-null
     *         (stub success); `false` if path is empty or factory is null.
     */
    bool Load(const std::string& plugin_path);

private:
    ComponentFactory* factory_ = nullptr;
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_PLUGIN_LOADER_HPP_
