/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/factory.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/proto/controller_options.pb.h"

namespace autonomy {
namespace map {
namespace costmap_2d {
class Costmap2DWrapper;
}  // namespace costmap_2d
}  // namespace map
namespace transform {
class Buffer;
}  // namespace transform
}  // namespace autonomy

namespace autonomy {
namespace control {

/** Shared inputs for controller plugin creators. */
struct ControllerCreateContext {
    const proto::ControllerOptions& options;
    std::string controller_id;
    std::shared_ptr<transform::Buffer> tf;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper;
};

using ControllerCreator = std::unique_ptr<common::ControllerInterface> (*)(
    const ControllerCreateContext&);

using ControllerFactoryRegistry =
    ::autonomy::common::Factory<std::string, common::ControllerInterface,
                                ControllerCreator>;

/**
 * @brief Process-wide local controller factory (string id → configured plugin).
 *
 * Built-in ids: graceful_controller, nmpc_controller, tdmpc_controller,
 * mppi_controller.
 * @ref ResolveControllerTypeId maps legacy C++ type strings and aliases.
 */
class ControllerFactory
{
public:
    static ControllerFactoryRegistry& RegistryInstance();

    static void EnsureBuiltinsRegistered();

    /** Normalizes plugin type strings to a registered factory id. */
    static std::string ResolveControllerTypeId(const std::string& controller_type);

    static bool HasController(const std::string& id);

    static common::ControllerInterface::SharedPtr Create(
        const std::string& controller_type, const ControllerCreateContext& context);

    static bool Register(const std::string& id, ControllerCreator creator) {
        return RegistryInstance().Register(id, creator);
    }
};

bool RegisterBuiltinControllers(ControllerFactoryRegistry& factory);

/**
 * @brief Creates a configured controller (compat wrapper around @ref
 * ControllerFactory::Create).
 */
common::ControllerInterface::SharedPtr CreateController(
    const std::string& controller_type, const proto::ControllerOptions& options,
    const std::string& controller_id,
    std::shared_ptr<transform::Buffer> tf,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper);

}  // namespace control
}  // namespace autonomy
