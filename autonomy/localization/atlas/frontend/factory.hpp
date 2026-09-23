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
 * @file factory.hpp
 * @brief Frontend factory registry and CreateFrontend entry point.
 *
 * Maps string mode names (e.g. `"vo"` / `"vio"`) to concrete FrontendBase types;
 * aligned with autonomy::common::Factory for config-driven startup.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FACTORY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FACTORY_HPP_

#include <memory>
#include <string>

#include "autonomy/common/factory.hpp"
#include "autonomy/localization/atlas/common/config.hpp"
#include "autonomy/localization/atlas/frontend/frontend_base.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @brief Frontend factory type alias: mode string key → FrontendBase* factory fn.
 */
using FrontendFactory =
    ::autonomy::common::Factory<std::string, FrontendBase, FrontendBase* (*)()>;

/**
 * @brief Return the process-wide frontend registry singleton.
 * @return Reference to FrontendFactory.
 */
FrontendFactory& FrontendRegistry();

/**
 * @brief Ensure VO / VIO frontends are registered in FrontendRegistry (idempotent).
 * @note First call registers `"vo"` → VisualOdometry, `"vio"` → VisualInertial.
 */
void EnsureFrontendsRegistered();

/**
 * @brief Create a frontend instance for @p config.mode.
 * @param config Atlas config; `mode` is typically `"vo"` or `"vio"`.
 * @return Owned FrontendBase; may be empty on unknown mode / failure (Registry).
 * @note Calls EnsureFrontendsRegistered() first.
 */
std::unique_ptr<FrontendBase> CreateFrontend(const AtlasConfig& config);

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_FACTORY_HPP_
